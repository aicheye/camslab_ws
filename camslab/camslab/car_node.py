"""The camera node of the fiducial survey. It runs where the camera is plugged in: on the
car, where `ros2 run camslab car` and survey.launch.py upload this file and start it over
ssh, or on this computer. It opens the camera with OpenCV, detects the ArUco markers in
every frame and publishes, in the namespace /<name>:

- `observations` (aruco_msgs/MarkerArray): one Marker per accepted fiducial with its id
  and pose in the estimator's camera frame (x forward, z up), stamped when the frame was
  grabbed, frame_id <name>/camera. Published for every frame, empty when no marker is
  accepted, so its rate is the image rate.
- `camera_info` (sensor_msgs/CameraInfo): the intrinsics in use, once a second.
- `preview/compressed` (sensor_msgs/CompressedImage): the frame as JPEG at --preview-rate
  Hz, with the detections drawn when --draw is set. The only image that leaves the car.

A survey estimates constants, so a frame without a marker costs driving time and a
frame with a wrong pose moves the estimate. The node therefore publishes a marker only
when its pose is well determined and never repeats an old pose: the corners are refined
to subpixel accuracy, a marker whose shortest side is under --min-side pixels is
dropped (the depth error is distance * corner error / side), and so is one whose two
planar pose solutions differ and fit the corners about equally well
(--ambiguity-ratio), because the wrong one is tens of degrees off. The preview labels a dropped marker with the
reason.

Self-contained on purpose: it needs rclpy, numpy, OpenCV with the aruco module and the
aruco_msgs package (ros-<distro>-aruco-msgs), and nothing of camslab, because the car
does not have the workspace. The constants and the
frame convention are those of camslab.aruco; tests/fiducial/test_car_node.py keeps them
equal. Settings come as command line arguments (camslab.car.node_arguments builds them).

    python3 car_node.py --name cologne --device /dev/video0 --width 640 --height 480 \\
        --fps 30 --pixel-format yuyv --dictionary DICT_4X4_50 --marker-size 0.14 \\
        --fx 528 --fy 528 --cx 325 --cy 239 --dist -0.4 0.2 0 0 0 --min-side 30 \\
        --ambiguity-ratio 2 --preview-rate 2 --draw
"""

import argparse
import os
import math
import sys
import time

import numpy as np

# OpenCV's optical frame has z forward, x right and y down; the estimator's camera frame
# has x forward and z up: body x = optical z, body y = -optical x, body z = -optical y.
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
FOURCC = {"yuyv": "YUYV", "mjpeg": "MJPG", "uyvy": "UYVY"}
RATE_PERIOD = 5.0
INFO_PERIOD = 1.0
GRAB_FAILURES = 30          # consecutive failed grabs before the camera is reopened
REOPEN_PERIOD = 2.0         # seconds between attempts to reopen a lost camera
REOPEN_TIMEOUT = 30.0       # seconds of attempts before the node gives up
REFINE_WINDOW = 3           # half side of the subpixel corner search window [px]
SAME_POSE = math.radians(5.0)   # two pose solutions closer than this count as one
SMALL, AMBIGUOUS = "small", "ambiguous"     # why a detected marker is not published


def parse_args(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in argv:                    # from a launch file's Node(arguments=...)
        argv = argv[:argv.index("--ros-args")]
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", required=True, help="robot name, the ROS namespace")
    p.add_argument("--device", default="/dev/video0", help="V4L2 device or index")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--pixel-format", default="yuyv", help="yuyv, mjpeg or uyvy")
    p.add_argument("--dictionary", default="DICT_4X4_50", help="cv2.aruco dictionary name")
    p.add_argument("--marker-size", type=float, default=0.14, help="printed side [m]")
    p.add_argument("--ids", type=int, nargs="*", default=[],
                   help="publish only these marker ids; empty: all. Texture decodes as a "
                        "random id of a 4x4 dictionary now and then, so the lab's ids only")
    p.add_argument("--min-side", type=float, default=30.0,
                   help="drop a marker whose shortest side in the image is shorter [px]; "
                        "0: no limit")
    p.add_argument("--ambiguity-ratio", type=float, default=2.0,
                   help="drop a marker unless the reprojection error of its second planar "
                        "pose solution is at least this many times that of the first; "
                        "0: no check")
    p.add_argument("--fx", type=float, default=0.0)
    p.add_argument("--fy", type=float, default=0.0)
    p.add_argument("--cx", type=float, default=0.0)
    p.add_argument("--cy", type=float, default=0.0)
    p.add_argument("--dist", type=float, nargs="*", default=[],
                   help="distortion coefficients (plumb_bob)")
    p.add_argument("--fov", type=float, default=0.0,
                   help="horizontal field of view [deg] when no calibration is given")
    p.add_argument("--no-detect", dest="detect", action="store_false",
                   help="images only, for the calibration")
    p.add_argument("--preview-rate", type=float, default=2.0,
                   help="JPEG frames per second on preview/compressed; 0: none")
    p.add_argument("--draw", action="store_true", help="draw the detections on the preview")
    p.add_argument("--jpeg-quality", type=int, default=80)
    p.add_argument("--frames", type=int, default=0, help="stop after this many frames; 0: never")
    p.add_argument("--record", default="", metavar="DIR",
                   help="keep every frame on the car: DIR/<name>_<date>_<time>/frames.avi "
                        "(Motion JPEG) and stamps.csv with the stamp of each frame")
    return p.parse_args(argv)


def intrinsics(args):
    """(K, dist) of the arguments: the calibration when fx is given, else the nominal
    matrix of --fov (no distortion), else None."""
    if args.fx > 0.0:
        K = np.array([[args.fx, 0.0, args.cx], [0.0, args.fy, args.cy], [0.0, 0.0, 1.0]])
        dist = np.array(args.dist or [0.0] * 5, float)
        return K, dist
    if args.fov > 0.0:
        f = (args.width / 2.0) / math.tan(math.radians(args.fov) / 2.0)
        K = np.array([[f, 0.0, args.width / 2.0], [0.0, f, args.height / 2.0], [0.0, 0.0, 1.0]])
        return K, np.zeros(5)
    return None


def marker_corners(size):
    """The marker's corners in its own frame, the order of cv2.aruco: top left, top right,
    bottom right, bottom left; z out of the plane."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def observation(rvec, tvec):
    """(R, t) of the marker in the estimator's camera frame from solvePnP's pose in the
    optical frame: C_T_M = C_T_O O_T_M."""
    import cv2

    R_o, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3))
    return OPTICAL_TO_BODY @ R_o, OPTICAL_TO_BODY @ np.asarray(tvec, float).reshape(3)


def quaternion_of(R):
    """(x, y, z, w) of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s,
                0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def shortest_side(quad):
    """The shortest side of a detected marker in the image [px]."""
    quad = np.asarray(quad, float).reshape(4, 2)
    return float(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1).min())


class Detector:
    """ArUco detection and pose estimation with either OpenCV API. detect() returns
    {id: (R, t)} in the estimator's camera frame for the markers whose pose is well
    determined; `rejected` holds {id: SMALL or AMBIGUOUS} for the others, and the corners
    and ids of the last frame stay on the object for draw()."""

    def __init__(self, dictionary, marker_size, K=None, dist=None, ids=(), min_side=30.0,
                 ambiguity_ratio=2.0):
        import cv2

        self.cv2 = cv2
        self.corners = marker_corners(marker_size)
        self.K, self.dist = K, dist
        self.ids = set(int(i) for i in ids)         # empty: every id
        self.min_side, self.ambiguity_ratio = float(min_side), float(ambiguity_ratio)
        self.found_corners, self.found_ids, self.rejected = [], None, {}
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        new_api = hasattr(cv2.aruco, "ArucoDetector")
        params = cv2.aruco.DetectorParameters() if new_api else cv2.aruco.DetectorParameters_create()
        # The corners of the contour are whole pixels, about 0.8 px off; the refinement
        # brings them to about 0.3 px. A fixed window of REFINE_WINDOW keeps that down to
        # markers of 20 px: OpenCV's default shrinks the window with the marker and stops
        # refining under about 30 px.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.cornerRefinementWinSize = REFINE_WINDOW
        if hasattr(params, "relativeCornerRefinmentWinSize"):
            params.relativeCornerRefinmentWinSize = 1.0
        if new_api:
            self._detect = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers
        else:
            self._detect = lambda image: cv2.aruco.detectMarkers(image, aruco_dict, parameters=params)

    def pose(self, quad):
        """(rvec, tvec) of a marker from its corners, or AMBIGUOUS. A planar square has
        two poses that project onto almost the same corners when it is seen head-on or
        from far away; IPPE returns both, sorted by reprojection error. Exactly head-on
        the two coincide, and a pair closer than SAME_POSE is not an ambiguity."""
        n, rvecs, tvecs, errors = self.cv2.solvePnPGeneric(
            self.corners, quad.reshape(4, 2), self.K, self.dist,
            flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
        if not n:
            return None
        errors = np.asarray(errors, float).reshape(-1)
        if n > 1 and errors[1] < self.ambiguity_ratio * errors[0]:
            R0, R1 = (self.cv2.Rodrigues(rvec)[0] for rvec in rvecs[:2])
            between, _ = self.cv2.Rodrigues(R0.T @ R1)
            if float(np.linalg.norm(between)) > SAME_POSE:
                return AMBIGUOUS
        return rvecs[0], tvecs[0]

    def detect(self, image):
        corners, ids, _ = self._detect(image)
        if ids is not None and self.ids:
            keep = [i for i, marker_id in enumerate(ids.reshape(-1)) if int(marker_id) in self.ids]
            corners = tuple(corners[i] for i in keep)
            ids = ids[keep] if keep else None
        self.found_corners, self.found_ids, self.rejected = corners, ids, {}
        found = {}
        if ids is None or self.K is None:
            return found
        for quad, marker_id in zip(corners, ids.reshape(-1)):
            if shortest_side(quad) < self.min_side:
                self.rejected[int(marker_id)] = SMALL
                continue
            pose = self.pose(quad)
            if pose is AMBIGUOUS:
                self.rejected[int(marker_id)] = AMBIGUOUS
            elif pose is not None:
                found[int(marker_id)] = observation(*pose)
        return found

    def draw(self, image, found):
        """The detected markers on `image`, in place: the distance of a published marker
        and the reason of a dropped one."""
        if self.found_ids is None:
            return
        self.cv2.aruco.drawDetectedMarkers(image, self.found_corners, self.found_ids)
        for quad, marker_id in zip(self.found_corners, self.found_ids.reshape(-1)):
            marker_id = int(marker_id)
            if marker_id in found:
                text, color = f"{float(np.linalg.norm(found[marker_id][1])):.2f} m", (0, 255, 255)
            elif marker_id in self.rejected:
                text, color = self.rejected[marker_id], (0, 0, 255)
            else:
                continue
            x, y = quad.reshape(4, 2).min(axis=0)
            self.cv2.putText(image, text, (int(x), int(y) - 6),
                             self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, self.cv2.LINE_AA)


def open_camera(device, width, height, fps, pixel_format):
    """The camera as a cv2.VideoCapture with the requested format, or a RuntimeError
    that says what it gives instead."""
    import cv2
    import glob

    # A USB camera that drops off the bus comes back under the next free number
    # (/dev/video0 as /dev/video1), so when the named device is gone the other video
    # devices are tried in order.
    named = str(device)
    others = [d for d in sorted(glob.glob("/dev/video*")) if d != named] if named.startswith("/dev/video") else []
    cap = None
    for candidate in [named, *others]:
        index = int(candidate) if candidate.isdigit() else candidate
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(index)
        if cap.isOpened():
            if candidate != named:
                print(f"camera {named} is gone; using {candidate}", file=sys.stderr, flush=True)
            device = candidate
            break
        cap.release()
    else:
        raise RuntimeError(f"cannot open camera {named}" + (f" or {', '.join(others)}" if others else ""))
    fourcc = FOURCC.get(str(pixel_format).lower(), str(pixel_format).upper())
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    cap.set(cv2.CAP_PROP_FPS, float(fps))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # the newest frame, not a queue of old ones
    got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != (int(width), int(height)):
        cap.release()
        raise RuntimeError(f"camera {device} gives {got[0]} x {got[1]} images, not {width} x "
                           f"{height}; set image_width and image_height to a format of "
                           f"`v4l2-ctl --list-formats-ext -d {device}`")
    return cap


class FrameRecorder:
    """Writes the camera's frames to a Motion JPEG file and their stamps to a CSV, in a
    thread of its own, so that a slow disk never delays the camera loop: a frame that
    finds the queue full is dropped and counted. Any failure stops the recording and
    leaves the camera node running."""

    def __init__(self, directory, name, width, height, fps, log):
        import queue
        import threading

        self.log = log
        self.path = os.path.join(directory, f"{name}_{time.strftime('%Y%m%d_%H%M%S')}")
        self.size, self.fps = (int(width), int(height)), float(fps)
        self.queue = queue.Queue(maxsize=90)
        self.written = self.dropped = 0
        self.failed = False
        self.thread = threading.Thread(target=self.work, daemon=True)
        self.thread.start()

    def add(self, image, stamp) -> None:
        if self.failed:
            return
        try:
            self.queue.put_nowait((image.copy(), stamp.sec, stamp.nanosec))
        except Exception:               # queue.Full: the disk is behind
            self.dropped += 1

    def work(self) -> None:
        import cv2

        try:
            os.makedirs(self.path, exist_ok=True)
            writer = cv2.VideoWriter(os.path.join(self.path, "frames.avi"), cv2.CAP_OPENCV_MJPEG,
                                     cv2.VideoWriter_fourcc(*"MJPG"), self.fps, self.size)
            if not writer.isOpened():
                raise RuntimeError("cv2.VideoWriter cannot write Motion JPEG here")
            writer.set(cv2.VIDEOWRITER_PROP_QUALITY, 90)
            with open(os.path.join(self.path, "stamps.csv"), "w") as stamps:
                stamps.write("frame,sec,nanosec\n")
                self.log.info(f"recording every frame to {self.path}")
                while True:
                    item = self.queue.get()
                    if item is None:
                        break
                    image, sec, nanosec = item
                    writer.write(image)
                    stamps.write(f"{self.written},{sec},{nanosec}\n")
                    self.written += 1
                    if self.written % 150 == 0:
                        stamps.flush()
            writer.release()
        except Exception as e:          # a full disk, a missing codec: keep the camera going
            self.failed = True
            self.log.error(f"the frame recording stopped: {e}")

    def close(self) -> None:
        if not self.failed:
            self.queue.put(None)
            self.thread.join(5.0)
        self.log.info(f"recorded {self.written} frames to {self.path}, {self.dropped} dropped")


def run(args):
    import cv2
    import rclpy
    from aruco_msgs.msg import Marker, MarkerArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage

    rclpy.init()
    node = Node("camera", namespace=f"/{args.name}")
    log = node.get_logger()
    frame_id = f"{args.name}/camera"
    calibration = intrinsics(args)
    if calibration is None and args.detect:
        log.error("no calibration (--fx --fy --cx --cy --dist) and no --fov: marker poses "
                  "need one; set `camera` or nominal_fov (docs/hardware-survey.md)")
        return 2
    if args.detect and args.fx <= 0.0:
        log.warning(f"camera not calibrated: nominal intrinsics for a {args.fov:.0f} degree field "
                    "of view, marker distances are scaled by the focal-length error")
    try:
        cap = open_camera(args.device, args.width, args.height, args.fps, args.pixel_format)
    except RuntimeError as e:
        log.error(str(e))
        return 2
    K, dist = calibration if calibration is not None else (None, None)
    detector = (Detector(args.dictionary, args.marker_size, K, dist, args.ids, args.min_side,
                         args.ambiguity_ratio) if args.detect else None)
    observations_pub = node.create_publisher(MarkerArray, "observations", 10)
    info_pub = node.create_publisher(CameraInfo, "camera_info", qos_profile_sensor_data)
    preview_pub = node.create_publisher(CompressedImage, "preview/compressed",
                                        qos_profile_sensor_data)
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = int(args.width), int(args.height)
    if K is not None:
        info.distortion_model = "plumb_bob"
        info.k = [float(v) for v in K.reshape(-1)]
        info.d = [float(v) for v in dist]
        info.p = [float(v) for v in np.hstack([K, np.zeros((3, 1))]).reshape(-1)]
    log.info(f"{args.device} {args.width}x{args.height} @ {args.fps:g} fps {args.pixel_format}, "
             + (f"{args.dictionary}, markers of {args.marker_size} m, "
                f"{'ids ' + ' '.join(map(str, args.ids)) if args.ids else 'all ids'}, "
                f"{'calibrated' if args.fx > 0 else f'{args.fov:.0f} deg fov'}, "
                f"side at least {args.min_side:g} px, ambiguity ratio {args.ambiguity_ratio:g}"
                if args.detect
                else "no detection")
             + f", preview {args.preview_rate:g} Hz" + (" drawn" if args.draw else ""))

    recorder = (FrameRecorder(args.record, args.name, args.width, args.height, args.fps, log)
                if args.record else None)
    frames, markers, failures, detect_time, total = 0, 0, 0, 0.0, 0
    dropped = {SMALL: 0, AMBIGUOUS: 0}
    rate_time = last_preview = last_info = time.monotonic()
    preview_period = 1.0 / args.preview_rate if args.preview_rate > 0 else None
    try:
        while rclpy.ok():
            if not cap.grab():
                failures += 1
                if failures < GRAB_FAILURES:
                    time.sleep(0.05)
                    continue
                # The camera dropped off the bus (a loose cable, a USB reset): reopen it
                # while it comes back, and give up after REOPEN_TIMEOUT.
                log.warning(f"camera {args.device} stopped giving frames; reopening")
                cap.release()
                lost = time.monotonic()
                while True:
                    time.sleep(REOPEN_PERIOD)
                    try:
                        cap = open_camera(args.device, args.width, args.height, args.fps,
                                          args.pixel_format)
                        log.info(f"camera {args.device} is back")
                        break
                    except RuntimeError as e:
                        if time.monotonic() - lost > REOPEN_TIMEOUT:
                            log.error(f"{e}; giving up after {REOPEN_TIMEOUT:.0f} s. Check "
                                      "the USB cable and `ls /dev/video*` on the car")
                            return 1
                    rclpy.spin_once(node, timeout_sec=0.0)
                    if not rclpy.ok():
                        return 0
                failures = 0
                continue
            failures = 0
            stamp = node.get_clock().now().to_msg()
            ok, image = cap.retrieve()
            if not ok:
                continue
            if recorder is not None:
                recorder.add(image, stamp)
            now = time.monotonic()
            frames += 1
            total += 1
            found = {}
            if detector is not None:
                t0 = time.monotonic()
                found = detector.detect(image)
                detect_time += time.monotonic() - t0
                markers += len(found)
                for reason in detector.rejected.values():
                    dropped[reason] += 1
                out = MarkerArray()
                out.header.stamp = stamp
                out.header.frame_id = frame_id
                for marker_id, (R, t) in found.items():
                    m = Marker()
                    m.header = out.header
                    m.id = int(marker_id)
                    p = m.pose.pose
                    p.position.x, p.position.y, p.position.z = (float(v) for v in t)
                    qx, qy, qz, qw = quaternion_of(R)
                    p.orientation.x, p.orientation.y = float(qx), float(qy)
                    p.orientation.z, p.orientation.w = float(qz), float(qw)
                    m.confidence = 1.0
                    out.markers.append(m)
                observations_pub.publish(out)
            if preview_period is not None and now - last_preview >= preview_period:
                last_preview = now
                if args.draw and detector is not None:
                    detector.draw(image, found)
                ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp = stamp
                    msg.header.frame_id = frame_id
                    msg.format = "jpeg"
                    msg.data = jpeg.tobytes()
                    preview_pub.publish(msg)
            if now - last_info >= INFO_PERIOD:
                last_info = now
                info.header.stamp = stamp
                info_pub.publish(info)
            if now - rate_time >= RATE_PERIOD:
                dt = now - rate_time
                line = f"{frames / dt:.1f} frames/s"
                if detector is not None:
                    line += (f", {markers / dt:.1f} markers/s, dropped "
                             f"{dropped[SMALL] / dt:.1f} small and "
                             f"{dropped[AMBIGUOUS] / dt:.1f} ambiguous, detection "
                             f"{1e3 * detect_time / max(frames, 1):.0f} ms/frame")
                log.info(line)
                frames, markers, detect_time, rate_time = 0, 0, 0.0, now
                dropped = {SMALL: 0, AMBIGUOUS: 0}
            if args.frames and total >= args.frames:
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if recorder is not None:
            recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
