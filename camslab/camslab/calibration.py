"""Camera intrinsics calibration with the lab's tag grid board: 6 x 6 square tags of
8.8 cm, 2.64 cm apart (a Kalibr AprilGrid). The result is a sensor_msgs/CameraInfo YAML
file in config/cameras/, which survey.launch.py hands to usb_cam through the `camera`
setting.

    ros2 run camslab calibrate --name yahboom_r2_640x480 --device /dev/video0

The tag dictionary (AprilTag 36h11 or an ArUco one) and the order of the ids on the board
are found from the first images, so another grid board needs only --cols, --rows, --tag
and --spacing. OpenCV is imported only where it is used, so the geometry and the file
handling here are testable without it.
"""

import argparse
import datetime
import itertools
import math
import os
import sys
import time

import numpy as np

COLS, ROWS = 6, 6                # tags across, tags down
TAG = 0.088                      # tag side, black border included [m]
SPACING = 0.0264                 # gap between neighbouring tags [m]
DICTIONARIES = ("DICT_APRILTAG_36h11", "DICT_6X6_1000", "DICT_ARUCO_ORIGINAL",
                "DICT_5X5_1000", "DICT_4X4_1000")
MIN_TAGS = 4                     # tags a view needs
NOVELTY = 0.12                   # distance in view_feature() space to accept a new view
TILTED = math.sin(math.radians(20.0))

# The order of the ids on a board: (column-major, columns right to left, rows bottom to
# top) relative to OpenCV's GridBoard, which numbers row by row from the top left. A Kalibr
# AprilGrid numbers row by row from the bottom left: (False, False, True).
LAYOUTS = tuple(itertools.product((False, True), repeat=3))


class Grid:
    """The tag positions of a grid board in its own frame: x right, y down, z into the
    board, the origin at the centre of the top left tag. Corners in the order of
    cv2.aruco: top left, top right, bottom right, bottom left of each upright tag."""

    def __init__(self, cols=COLS, rows=ROWS, tag=TAG, spacing=SPACING, layout=None):
        self.cols, self.rows, self.tag = cols, rows, tag
        self.pitch = tag + spacing
        self.layout = layout

    @property
    def count(self) -> int:
        return self.cols * self.rows

    def cell(self, tag_id: int, layout) -> tuple:
        """(column, row) of a tag id, columns from the left and rows from the top."""
        column_major, flip_x, flip_y = layout
        if column_major:
            c, r = divmod(tag_id, self.rows)
        else:
            r, c = divmod(tag_id, self.cols)
        return (self.cols - 1 - c if flip_x else c, self.rows - 1 - r if flip_y else r)

    def object_points(self, ids, layout=None) -> np.ndarray:
        """(4 n, 3) corners of the tags `ids`, four per tag."""
        layout = layout or self.layout
        h = self.tag / 2.0
        offsets = np.array([[-h, -h], [h, -h], [h, h], [-h, h]])
        points = [np.array(self.cell(int(i), layout)) * self.pitch + offsets for i in ids]
        points = np.vstack(points) if points else np.zeros((0, 2))
        return np.hstack([points, np.zeros((len(points), 1))])

    def find_layout(self, ids, image_points):
        """The layout whose board maps onto `image_points` (four per id) by a homography,
        and the RMS residuals [px] of all layouts. None when the best layout is not
        clearly better than the second, which happens when the tags seen are symmetric."""
        residuals = {layout: homography_residual(self.object_points(ids, layout)[:, :2],
                                                 image_points) for layout in LAYOUTS}
        ranked = sorted(residuals, key=residuals.get)
        best, second = residuals[ranked[0]], residuals[ranked[1]]
        if best < 2.0 and second > 5.0 * max(best, 0.1):
            return ranked[0], residuals
        return None, residuals


def homography_residual(plane: np.ndarray, image: np.ndarray) -> float:
    """RMS distance [px] between `image` and the points of `plane` mapped by the least
    squares homography (direct linear transform on normalised points)."""
    plane = np.asarray(plane, float).reshape(-1, 2)
    image = np.asarray(image, float).reshape(-1, 2)

    def normaliser(p):
        mean, scale = p.mean(axis=0), math.sqrt(2.0) / max(np.linalg.norm(p - p.mean(axis=0),
                                                                         axis=1).mean(), 1e-12)
        return np.array([[scale, 0.0, -scale * mean[0]], [0.0, scale, -scale * mean[1]],
                         [0.0, 0.0, 1.0]])

    Tp, Ti = normaliser(plane), normaliser(image)
    p = (Tp @ np.column_stack([plane, np.ones(len(plane))]).T).T
    q = (Ti @ np.column_stack([image, np.ones(len(image))]).T).T
    rows = []
    for (x, y, _), (u, v, _) in zip(p, q):
        rows.append([-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u])
        rows.append([0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v])
    H = np.linalg.svd(np.array(rows))[2][-1].reshape(3, 3)
    H = np.linalg.inv(Ti) @ H @ Tp
    mapped = (H @ np.column_stack([plane, np.ones(len(plane))]).T).T
    mapped = mapped[:, :2] / mapped[:, 2:]
    return float(np.sqrt(np.mean(np.sum((mapped - image) ** 2, axis=1))))


def cameras_directory() -> str:
    """config/cameras in the source tree when the package is installed with
    --symlink-install, so a new calibration lands where git sees it; otherwise the
    current directory."""
    here = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "cameras")
    return os.path.normpath(here) if os.path.isdir(here) else os.getcwd()


def camera_info_yaml(name: str, width: int, height: int, K: np.ndarray, dist: np.ndarray,
                     comment: str = "") -> str:
    """The calibration file format of camera_info_manager (the one of
    camera_calibration's ost.yaml), plumb_bob distortion."""
    K = np.asarray(K, float).reshape(3, 3)
    dist = np.asarray(dist, float).reshape(-1)[:5]
    dist = np.concatenate([dist, np.zeros(5 - dist.size)])
    P = np.hstack([K, np.zeros((3, 1))])

    def matrix(rows, cols, values):
        data = ", ".join(f"{v:.8g}" for v in np.asarray(values, float).reshape(-1))
        return f"  rows: {rows}\n  cols: {cols}\n  data: [{data}]\n"

    header = "".join(f"# {line}\n" for line in comment.splitlines())
    return (f"{header}image_width: {int(width)}\nimage_height: {int(height)}\n"
            f"camera_name: {name}\n"
            f"camera_matrix:\n{matrix(3, 3, K)}"
            f"distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n{matrix(1, 5, dist)}"
            f"rectification_matrix:\n{matrix(3, 3, np.eye(3))}"
            f"projection_matrix:\n{matrix(3, 4, P)}")


def view_feature(points: np.ndarray, normal: np.ndarray, width: int, height: int) -> np.ndarray:
    """Where a board view is: the centre of its corners as a fraction of the image, its
    size (square root of the area of the corners' bounding box over the image area), and
    the x and y components of the board normal in the optical frame, the sines of its
    tilt about the two image axes."""
    points = np.asarray(points, float).reshape(-1, 2)
    lo, hi = points.min(axis=0), points.max(axis=0)
    size = math.sqrt((hi[0] - lo[0]) * (hi[1] - lo[1]) / (width * height))
    centre = points.mean(axis=0) / (width, height)
    return np.array([centre[0], centre[1], size, normal[0], normal[1]])


def is_new_view(feature: np.ndarray, accepted: list, threshold: float = NOVELTY) -> bool:
    return all(np.linalg.norm(feature - f) > threshold for f in accepted)


class TagDetector:
    """cv2.aruco tag detection, both the 4.7+ ArucoDetector API and the older module
    functions. The corners are refined with the AprilTag edge fit, which on rendered
    boards gives a reprojection error of 0.2 px against 0.7 px for CORNER_REFINE_SUBPIX,
    for ArUco tags as well."""

    def __init__(self, dictionary: str):
        import cv2

        self.cv2, self.name = cv2, dictionary
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        refinement = getattr(cv2.aruco, "CORNER_REFINE_APRILTAG",
                             cv2.aruco.CORNER_REFINE_SUBPIX)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = refinement
            self._detect = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = refinement
            self._detect = lambda image: cv2.aruco.detectMarkers(image, aruco_dict,
                                                                 parameters=params)

    def detect(self, grey, count: int):
        """(ids (n,), corners (4 n, 2)) of the tags with an id below `count`."""
        corners, ids, _ = self._detect(grey)
        if ids is None:
            return np.zeros(0, int), np.zeros((0, 2), np.float32)
        ids = ids.reshape(-1)
        keep = [k for k in np.argsort(ids) if ids[k] < count]
        if not keep:
            return np.zeros(0, int), np.zeros((0, 2), np.float32)
        return ids[keep], np.vstack([corners[k].reshape(4, 2) for k in keep]).astype(np.float32)


class Calibrator:
    """Finds the dictionary and the id layout of the board on the first clear views, then
    collects views that differ from the ones already kept and calibrates on them. The
    tilt of a view comes from solvePnP with the camera matrix of a 60 degree field of
    view, which is good enough to tell views apart."""

    def __init__(self, grid: Grid, width: int, height: int, dictionary: str = ""):
        self.grid, self.width, self.height = grid, width, height
        self.detector = TagDetector(dictionary) if dictionary else None
        self.object_points, self.image_points, self.features = [], [], []
        f = (width / 2.0) / math.tan(math.radians(30.0))
        self.K0 = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
        self.messages = []

    def _choose_dictionary(self, grey):
        best = None
        for name in DICTIONARIES:
            detector = TagDetector(name)
            n = len(detector.detect(grey, self.grid.count)[0])
            if n >= MIN_TAGS and (best is None or n > best[0]):
                best = (n, detector)
        if best is not None:
            self.detector = best[1]
            self.messages.append(f"tags are {self.detector.name}")

    def add(self, grey: np.ndarray):
        """The feature of the view in `grey` when it was kept, else None."""
        if self.detector is None:
            self._choose_dictionary(grey)
            if self.detector is None:
                return None
        ids, img = self.detector.detect(grey, self.grid.count)
        if len(ids) < MIN_TAGS:
            return None
        if self.grid.layout is None:
            layout, _ = self.grid.find_layout(ids, img)
            if layout is None:
                return None
            self.grid.layout = layout
            self.messages.append(f"id layout (column-major, flip x, flip y) = {layout}")
        obj = self.grid.object_points(ids).astype(np.float32)
        cv2 = self.detector.cv2
        ok, rvec, _ = cv2.solvePnP(obj, img, self.K0, None)
        if not ok:
            return None
        normal = cv2.Rodrigues(rvec)[0][:, 2]
        feature = view_feature(img, normal, self.width, self.height)
        if not is_new_view(feature, self.features):
            return None
        self.object_points.append(obj)
        self.image_points.append(img.reshape(-1, 1, 2))
        self.features.append(feature)
        return feature

    def tilted_views(self) -> int:
        return sum(1 for f in self.features if math.hypot(f[3], f[4]) > TILTED)

    def calibrate(self):
        """(rms reprojection error [px], K, distortion) over the kept views."""
        rms, K, dist, _, _ = self.detector.cv2.calibrateCamera(
            self.object_points, self.image_points, (self.width, self.height), None, None)
        return rms, K, dist.reshape(-1)


def frames_from_device(device: str, width: int, height: int):
    import cv2

    capture = cv2.VideoCapture(int(device) if device.isdigit() else device)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        sys.exit(f"cannot open {device}; is usb_cam still running?")
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                sys.exit(f"{device} stopped delivering images")
            yield frame
    finally:
        capture.release()


def frames_from_directory(directory: str):
    import cv2

    for name in sorted(os.listdir(directory)):
        frame = cv2.imread(os.path.join(directory, name))
        if frame is not None:
            yield frame


def run(args) -> None:
    import cv2

    grid = Grid(args.cols, args.rows, args.tag, args.spacing)
    frames = (frames_from_directory(args.images) if args.images
              else frames_from_device(args.device, args.width, args.height))
    calibrator = None
    last = 0.0
    if args.save:
        os.makedirs(args.save, exist_ok=True)
    print(f"Show the {grid.cols} x {grid.rows} tag board at different places, distances and "
          f"tilts; {args.views} views are kept.")
    for frame in frames:
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if calibrator is None:
            calibrator = Calibrator(grid, grey.shape[1], grey.shape[0], args.dictionary)
        now = time.monotonic()
        if args.images or now - last > args.interval:
            feature = calibrator.add(grey)
            for message in calibrator.messages:
                print(message)
            calibrator.messages.clear()
            if feature is not None:
                last = now
                n = len(calibrator.features)
                tilt = math.degrees(math.asin(min(1.0, math.hypot(feature[3], feature[4]))))
                print(f"view {n}/{args.views}: {len(calibrator.object_points[-1]) // 4} tags at "
                      f"({feature[0]:.2f}, {feature[1]:.2f}), size {feature[2]:.2f}, "
                      f"tilt {tilt:.0f} deg")
                if args.save:
                    cv2.imwrite(os.path.join(args.save, f"view_{n:03d}.png"), frame)
        if args.show:
            if calibrator.detector is not None:
                _, img = calibrator.detector.detect(grey, grid.count)
                for u, v in img:
                    cv2.circle(frame, (int(u), int(v)), 3, (0, 255, 0), -1)
            cv2.imshow("calibrate", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        if len(calibrator.features) >= args.views:
            break
    if calibrator is None or len(calibrator.features) < 5:
        sys.exit("fewer than 5 board views; nothing calibrated")
    if calibrator.tilted_views() < 5:
        print(f"warning: {calibrator.tilted_views()} views tilted by more than 20 degrees; "
              "the focal length needs tilted views")
    rms, K, dist = calibrator.calibrate()
    print(f"reprojection error {rms:.3f} px over {len(calibrator.features)} views")
    print(f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}  "
          f"distortion {np.array2string(dist, precision=4)}")
    if rms > 1.0:
        print("warning: reprojection error above 1 px; check the board is flat and the "
              "images are sharp")
    path = args.out or os.path.join(cameras_directory(), f"{args.name}.yaml")
    comment = (f"{args.name}: calibration of {datetime.date.today().isoformat()} with the "
               f"{grid.cols} x {grid.rows} {calibrator.detector.name} board, "
               f"{len(calibrator.features)} views, reprojection error {rms:.3f} px.\n"
               "Written by `ros2 run camslab calibrate` (docs/hardware-survey.md).")
    with open(path, "w") as f:
        f.write(camera_info_yaml("camera", calibrator.width, calibrator.height, K, dist,
                                 comment))
    print(f"wrote {path}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="calibrate", description=__doc__.split("\n\n")[0])
    parser.add_argument("--name", required=True,
                        help="calibration name, written to config/cameras/<name>.yaml; "
                             "the `camera` setting of survey.yaml")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--images", help="calibrate from the images in this directory "
                                         "instead of the camera")
    parser.add_argument("--views", type=int, default=30, help="views to keep")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="minimum time between kept views [s]")
    parser.add_argument("--save", help="directory to save the kept views in")
    parser.add_argument("--out", help="output file instead of config/cameras/<name>.yaml")
    parser.add_argument("--show", action="store_true", help="show the image (needs a display)")
    parser.add_argument("--cols", type=int, default=COLS, help="tags across")
    parser.add_argument("--rows", type=int, default=ROWS, help="tags down")
    parser.add_argument("--tag", type=float, default=TAG, help="tag side [m]")
    parser.add_argument("--spacing", type=float, default=SPACING, help="gap between tags [m]")
    parser.add_argument("--dictionary", default="",
                        help="cv2.aruco dictionary; empty: found from the images")
    run(parser.parse_args(argv))


if __name__ == "__main__":
    main()
