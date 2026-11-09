"""The camera node of the fiducial survey. It runs where the camera is plugged in: on the
car, where `ros2 run camslab car` and survey.launch.py upload this file and start it over
ssh, or on this computer. It opens the camera with OpenCV, detects the ArUco markers in
every frame and publishes, in the namespace /<name>:

- `observations` (tf2_msgs/TFMessage): one transform <name>/camera -> marker_<id> per
  visible marker, in the estimator's camera frame (x forward, z up), stamped when the
  frame was grabbed. Published for every frame, empty when no marker is seen, so its
  rate is the image rate.
- `camera_info` (sensor_msgs/CameraInfo): the intrinsics in use, once a second.
- `preview/compressed` (sensor_msgs/CompressedImage): the frame as JPEG at --preview-rate
  Hz, with the detections drawn when --draw is set. The only image that leaves the car.

Self-contained on purpose: it needs rclpy, numpy and OpenCV with the aruco module, and
nothing of camslab, because the car does not have the workspace. The constants and the
frame convention are those of camslab.aruco; tests/fiducial/test_car_node.py keeps them
equal. Settings come as command line arguments (camslab.car.node_arguments builds them).

    python3 car_node.py --name cologne --device /dev/video0 --width 640 --height 480 \\
        --fps 30 --pixel-format yuyv --dictionary DICT_4X4_50 --marker-size 0.14 \\
        --fx 528 --fy 528 --cx 325 --cy 239 --dist -0.4 0.2 0 0 0 --preview-rate 2 --draw
"""

import argparse
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


class Detector:
    """ArUco detection and pose estimation with either OpenCV API. detect() returns
    {id: (R, t)} in the estimator's camera frame; corners and ids of the last frame stay
    on the object for draw()."""

    def __init__(self, dictionary, marker_size, K=None, dist=None, ids=()):
        import cv2

        self.cv2 = cv2
        self.corners = marker_corners(marker_size)
        self.K, self.dist = K, dist
        self.ids = set(int(i) for i in ids)         # empty: every id
        self.found_corners, self.found_ids = [], None
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detect = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters()).detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda image: cv2.aruco.detectMarkers(image, aruco_dict, parameters=params)

    def detect(self, image):
        corners, ids, _ = self._detect(image)
        if ids is not None and self.ids:
            keep = [i for i, marker_id in enumerate(ids.reshape(-1)) if int(marker_id) in self.ids]
            corners = tuple(corners[i] for i in keep)
            ids = ids[keep] if keep else None
        self.found_corners, self.found_ids = corners, ids
        found = {}
        if ids is None or self.K is None:
            return found
        for quad, marker_id in zip(corners, ids.reshape(-1)):
            ok, rvec, tvec = self.cv2.solvePnP(self.corners, quad.reshape(4, 2), self.K, self.dist,
                                               flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                found[int(marker_id)] = observation(rvec, tvec)
        return found

    def draw(self, image, found):
        """The detected markers and their distances on `image`, in place."""
        if self.found_ids is None:
            return
        self.cv2.aruco.drawDetectedMarkers(image, self.found_corners, self.found_ids)
        for quad, marker_id in zip(self.found_corners, self.found_ids.reshape(-1)):
            if int(marker_id) in found:
                x, y = quad.reshape(4, 2).min(axis=0)
                d = float(np.linalg.norm(found[int(marker_id)][1]))
                self.cv2.putText(image, f"{d:.2f} m", (int(x), int(y) - 6),
                                 self.cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, self.cv2.LINE_AA)


def open_camera(device, width, height, fps, pixel_format):
    """The camera as a cv2.VideoCapture with the requested format, or a RuntimeError
    that says what it gives instead."""
    import cv2

    index = int(device) if str(device).isdigit() else str(device)
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2) if sys.platform.startswith("linux") else cv2.VideoCapture(index)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {device}")
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


def run(args):
    import cv2
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, CompressedImage
    from tf2_msgs.msg import TFMessage

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
    detector = Detector(args.dictionary, args.marker_size, K, dist, args.ids) if args.detect else None
    observations_pub = node.create_publisher(TFMessage, "observations", 10)
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
                f"{'calibrated' if args.fx > 0 else f'{args.fov:.0f} deg fov'}" if args.detect
                else "no detection")
             + f", preview {args.preview_rate:g} Hz" + (" drawn" if args.draw else ""))

    frames, markers, failures, detect_time, total = 0, 0, 0, 0.0, 0
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
            now = time.monotonic()
            frames += 1
            total += 1
            found = {}
            if detector is not None:
                t0 = time.monotonic()
                found = detector.detect(image)
                detect_time += time.monotonic() - t0
                markers += len(found)
                tfs = []
                for marker_id, (R, t) in found.items():
                    tf = TransformStamped()
                    tf.header.stamp = stamp
                    tf.header.frame_id = frame_id
                    tf.child_frame_id = f"marker_{marker_id}"
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
                        float(v) for v in t)
                    qx, qy, qz, qw = quaternion_of(R)
                    tf.transform.rotation.x, tf.transform.rotation.y = float(qx), float(qy)
                    tf.transform.rotation.z, tf.transform.rotation.w = float(qz), float(qw)
                    tfs.append(tf)
                observations_pub.publish(TFMessage(transforms=tfs))
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
                    line += (f", {markers / dt:.1f} markers/s, detection "
                             f"{1e3 * detect_time / max(frames, 1):.0f} ms/frame")
                log.info(line)
                frames, markers, detect_time, rate_time = 0, 0, 0.0, now
            if args.frames and total >= args.frames:
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
