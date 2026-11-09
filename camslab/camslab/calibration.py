"""Camera intrinsics calibration with the lab's AprilTag board: a 6 x 6 grid of 36
AprilTag 36h11 tags, ids 0 to 35, tags of 8.8 cm with 2.64 cm between them (a Kalibr
AprilGrid). The result is a sensor_msgs/CameraInfo YAML file and two pictures,
before_after.png and distortion_map.png. The `aruco` node and survey.launch.py read the
file through the `camera` setting of config/survey.yaml.

From this computer, with the car's camera streaming (docs/hardware-survey.md):

    ros2 run camslab car calibrate --name yahboom_r2_640x480

On a machine with the camera plugged in, without ROS:

    python3 calibration.py --name yahboom_r2_640x480 --device /dev/video0

The width of the tags' black border (2 bits as Kalibr prints them, 1 bit as OpenCV does)
and the order of the ids on the board are found from the first images. Another 36h11
grid needs only --cols, --rows, --tag and --spacing. OpenCV is imported only where it is
used, so the geometry and the file handling here are testable without it.
"""

import argparse
import datetime
import itertools
import math
import os
import sys
import time

import numpy as np

COLS, ROWS = 6, 6                # tags across, tags down: 36 tags, ids 0 to 35
TAG = 0.088                      # tag side, black border included [m]
SPACING = 0.0264                 # gap between neighbouring tags [m]
DICTIONARY = "DICT_APRILTAG_36h11"
BORDER_BITS = (2, 1)             # Kalibr prints a 2 bit black border, OpenCV 1 bit
MIN_TAGS = 4                     # tags a view needs
MIN_VIEWS = 5                    # views a calibration needs
VIEWS = 20                       # views kept at least; the coverage decides when it is done
INTERVAL = 0.5                   # minimum time between kept views [s]
STATUS_PERIOD = 2.0              # seconds between status lines and latest.png
NOVELTY = 0.08                   # distance in view_feature() space to accept a new view
TILTED = math.sin(math.radians(20.0))
# Coverage bins of the kept views, see Coverage.
GRID = 3                         # the image in GRID x GRID areas the board centre must visit
SIZE_BINS = (("far", 0.0), ("mid", 0.3), ("near", 0.5))   # name, lowest board size (view_feature)
TILT_BINS = ("flat", "left", "right", "up", "down")
TILT_FLAT = math.sin(math.radians(12.0))
PER_BIN = 2                      # views every size and tilt bin needs
SUBPIX_WINDOW = 4                # half window [px] of the corner refinement on a Kalibr grid

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


class Coverage:
    """Where the kept views were, from their view_feature(): the board centre in a
    GRID x GRID grid of the image, the board's size as far, mid or near, and its tilt as
    flat or toward one of four sides (by the sign of the board normal's x and y, which
    only need to be told apart). Complete when every grid area has a view and every size
    and tilt bin has PER_BIN, so that the calibration sees the distortion everywhere in
    the image and the focal length is pinned by views tilted both ways."""

    def __init__(self):
        self.areas = np.zeros((GRID, GRID), int)        # [row, column]
        self.sizes = {name: 0 for name, _ in SIZE_BINS}
        self.tilts = {name: 0 for name in TILT_BINS}

    @staticmethod
    def bins(feature) -> tuple:
        """((row, column), size name, tilt name) of a view feature."""
        column = min(int(feature[0] * GRID), GRID - 1)
        row = min(int(feature[1] * GRID), GRID - 1)
        size = next(name for name, low in reversed(SIZE_BINS) if feature[2] >= low)
        nx, ny = feature[3], feature[4]
        if math.hypot(nx, ny) < TILT_FLAT:
            tilt = "flat"
        elif abs(nx) >= abs(ny):
            tilt = "right" if nx > 0 else "left"
        else:
            tilt = "down" if ny > 0 else "up"
        return (row, column), size, tilt

    def add(self, feature) -> str:
        """Counts a view; returns its bins as text."""
        (row, column), size, tilt = self.bins(feature)
        self.areas[row, column] += 1
        self.sizes[size] += 1
        self.tilts[tilt] += 1
        return f"area {row + 1},{column + 1}, {size}, {tilt}"

    def missing(self) -> list:
        """What is still needed, one item per shortfall."""
        out = []
        empty = int((self.areas == 0).sum())
        if empty:
            out.append(f"{empty} of {GRID * GRID} image areas")
        for name, n in list(self.sizes.items()) + list(self.tilts.items()):
            if n < PER_BIN:
                out.append(f"{name} {PER_BIN - n}")
        return out

    def complete(self) -> bool:
        return not self.missing()

    def text(self) -> str:
        filled = GRID * GRID - int((self.areas == 0).sum())
        sizes = " ".join(f"{name} {n}" for name, n in self.sizes.items())
        tilts = " ".join(f"{name} {n}" for name, n in self.tilts.items())
        return f"areas {filled}/{GRID * GRID}, {sizes}, {tilts}"


class TagDetector:
    """cv2.aruco tag detection, both the 4.7+ ArucoDetector API and the older module
    functions, for tags whose black border is `border_bits` wide. With `refine` the
    corners are refined, by the method the board allows: on a Kalibr AprilGrid (2 bit
    border) a black square touches every tag corner, which makes it a saddle point that
    cv2.cornerSubPix with a 4 px window locates to 0.2 px on rendered boards; on a plain
    board (1 bit border) the corners are L shaped and the AprilTag edge fit is used,
    also 0.2 px. The AprilTag fit is not used on the grid because on a webcam frame of
    the lab's board it kept 17 of 36 tags where the plain detector kept all 36. Without
    `refine` detection is a little faster, which the border search uses."""

    def __init__(self, dictionary: str, border_bits: int = 1, refine: bool = True):
        import cv2

        self.cv2, self.name, self.border_bits = cv2, dictionary, border_bits
        self.found = 0                   # tags in the last image, any id
        self.subpix = refine and border_bits >= 2
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if refine and not self.subpix:
            refinement = getattr(cv2.aruco, "CORNER_REFINE_APRILTAG", cv2.aruco.CORNER_REFINE_SUBPIX)
        else:
            refinement = cv2.aruco.CORNER_REFINE_NONE
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = refinement
            params.markerBorderBits = border_bits
            self._detect = cv2.aruco.ArucoDetector(aruco_dict, params).detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = refinement
            params.markerBorderBits = border_bits
            self._detect = lambda image: cv2.aruco.detectMarkers(image, aruco_dict,
                                                                 parameters=params)

    def detect(self, grey, count: int):
        """(ids (n,), corners (4 n, 2)) of the tags with an id below `count`."""
        corners, ids, _ = self._detect(grey)
        self.found = 0 if ids is None else len(ids)
        if ids is None:
            return np.zeros(0, int), np.zeros((0, 2), np.float32)
        ids = ids.reshape(-1)
        keep = [k for k in np.argsort(ids) if ids[k] < count]
        if not keep:
            return np.zeros(0, int), np.zeros((0, 2), np.float32)
        points = np.vstack([corners[k].reshape(4, 2) for k in keep]).astype(np.float32)
        if self.subpix:
            cv2 = self.cv2
            points = points.reshape(-1, 1, 2)
            cv2.cornerSubPix(grey, points, (SUBPIX_WINDOW, SUBPIX_WINDOW), (-1, -1),
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
            points = points.reshape(-1, 2)
        return ids[keep], points


class Calibrator:
    """Finds the border width and the id layout of the board on the first clear views,
    then collects views that differ from the ones already kept and calibrates on them.
    The tilt of a view comes from solvePnP with the camera matrix of a 60 degree field of
    view, which is good enough to tell views apart."""

    def __init__(self, grid: Grid, width: int, height: int, dictionary: str = DICTIONARY,
                 border_bits: int = 0):
        self.grid, self.width, self.height = grid, width, height
        self.detector = None
        if border_bits:
            self.detector = TagDetector(dictionary, border_bits)
        self.candidates = [TagDetector(dictionary, bits, refine=False)
                           for bits in ([border_bits] if border_bits else BORDER_BITS)]
        self.object_points, self.image_points, self.features = [], [], []
        f = (width / 2.0) / math.tan(math.radians(30.0))
        self.K0 = np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])
        self.messages = []
        self.last_ids, self.last_corners = np.zeros(0, int), np.zeros((0, 2))

    def _choose_dictionary(self, grey):
        best = None
        for detector in self.candidates:
            n = len(detector.detect(grey, self.grid.count)[0])
            if n >= MIN_TAGS and (best is None or n > best[0]):
                best = (n, detector)
        if best is not None:
            self.detector = TagDetector(best[1].name, best[1].border_bits)
            self.messages.append(f"tags have a {self.detector.border_bits} bit border")

    def status(self) -> str:
        """The tags found in the last image, by the chosen detector or, while the border
        width is unknown, by each candidate."""
        if self.detector is not None:
            return f"{self.detector.found} tags"
        found = [f"{d.found} with a {d.border_bits} bit border" for d in self.candidates if d.found]
        return "no tags found" if not found else "tags: " + ", ".join(found)

    @property
    def views(self) -> int:
        return len(self.features)

    def add(self, grey: np.ndarray, keep: bool = True):
        """Detects the tags in `grey` (for status()) and, with `keep`, keeps the view when
        it is new; the feature of the kept view, else None."""
        if self.detector is None:
            self._choose_dictionary(grey)
            if self.detector is None:
                return None
        ids, img = self.detector.detect(grey, self.grid.count)
        self.last_ids, self.last_corners = ids, img
        if len(ids) < MIN_TAGS:
            return None
        if self.grid.layout is None:
            layout, _ = self.grid.find_layout(ids, img)
            if layout is None:
                return None
            self.grid.layout = layout
            self.messages.append(f"id layout (column-major, flip x, flip y) = {layout}")
        if not keep:
            return None
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


def distortion_displacement(K, dist, width: int, height: int) -> np.ndarray:
    """(height, width, 2): how far [px] undistortion moves each pixel of the camera image,
    the ideal pinhole position minus the measured one."""
    import cv2

    u, v = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    pixels = np.stack([u, v], axis=-1).reshape(-1, 1, 2)
    ideal = cv2.undistortPoints(pixels, np.asarray(K, float), np.asarray(dist, float),
                                P=np.asarray(K, float))
    return (ideal - pixels).reshape(height, width, 2)


def nice_step(value: float) -> float:
    """1, 2 or 5 times a power of ten, about a fifth of `value`."""
    if value <= 0.0:
        return 1.0
    power = 10.0 ** math.floor(math.log10(value / 5.0))
    return next(m * power for m in (1.0, 2.0, 5.0, 10.0) if m * power >= value / 5.0)


def distortion_map(K, dist, width: int, height: int) -> np.ndarray:
    """A BGR image of the lens distortion: colour is the displacement magnitude [px] with
    white contour lines, arrows every 40 px point where undistortion moves the pixel
    (scaled so the longest is 30 px), a cross marks the principal point, and a colour bar
    with the scale is below."""
    import cv2

    displacement = distortion_displacement(K, dist, width, height)
    magnitude = np.linalg.norm(displacement, axis=-1)
    top = max(float(magnitude.max()), 1e-6)
    colours = cv2.applyColorMap((255.0 * magnitude / top).astype(np.uint8), cv2.COLORMAP_JET)
    step = nice_step(top)
    band = np.floor(magnitude / step)
    edge = np.zeros_like(band, bool)
    edge[:, 1:] |= band[:, 1:] != band[:, :-1]
    edge[1:, :] |= band[1:, :] != band[:-1, :]
    colours[edge] = 255
    scale = 30.0 / top
    for y in range(20, height, 40):
        for x in range(20, width, 40):
            dx, dy = displacement[y, x] * scale
            cv2.arrowedLine(colours, (x, y), (int(round(x + dx)), int(round(y + dy))),
                            (0, 0, 0), 1, cv2.LINE_AA, tipLength=0.3)
    cx, cy = int(round(K[0][2])), int(round(K[1][2]))
    cv2.drawMarker(colours, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
    legend = np.full((70, width, 3), 255, np.uint8)
    bar = np.linspace(0, 255, width - 40).astype(np.uint8)[None, :].repeat(14, axis=0)
    legend[8:22, 20:width - 20] = cv2.applyColorMap(bar, cv2.COLORMAP_JET)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(legend, "0 px", (20, 40), font, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(legend, f"{top:.1f} px", (width - 80, 40), font, 0.45, (0, 0, 0), 1,
                cv2.LINE_AA)
    cv2.putText(legend, f"undistortion displacement; contours every {step:g} px, arrows "
                f"x{scale:.1f}, + principal point", (20, 60), font, 0.4, (0, 0, 0), 1,
                cv2.LINE_AA)
    return np.vstack([colours, legend])


def before_after(frame: np.ndarray, K, dist) -> np.ndarray:
    """The camera image and the same image undistorted with the same camera matrix, side
    by side; straight edges in the scene are straight on the right."""
    import cv2

    undistorted = cv2.undistort(frame, np.asarray(K, float), np.asarray(dist, float))
    out = np.hstack([frame, np.full((frame.shape[0], 8, 3), 255, np.uint8), undistorted])
    for text, x in (("original", 10), ("undistorted", frame.shape[1] + 18)):
        cv2.putText(out, text, (x, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, text, (x, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
                    cv2.LINE_AA)
    return out


def annotate(frame: np.ndarray, corners, lines, coverage: Coverage = None) -> np.ndarray:
    """A BGR copy of `frame` with the tag corners as green dots, `lines` of text in the
    top left corner and, with a `coverage`, its grid of image areas in the top right,
    the areas with a view filled."""
    import cv2

    image = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    for u, v in np.asarray(corners, float).reshape(-1, 2):
        cv2.circle(image, (int(u), int(v)), 3, (0, 255, 0), -1)
    if coverage is not None:
        cell = max(12, image.shape[1] // 32)
        x0, y0 = image.shape[1] - 8 - GRID * cell, 8
        for row in range(GRID):
            for column in range(GRID):
                corner = (x0 + column * cell, y0 + row * cell)
                far = (corner[0] + cell - 1, corner[1] + cell - 1)
                if coverage.areas[row, column]:
                    cv2.rectangle(image, corner, far, (0, 200, 0), -1)
                cv2.rectangle(image, corner, far, (255, 255, 255), 1)
    for i, text in enumerate(lines):
        y = 20 + 18 * i
        cv2.putText(image, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                    cv2.LINE_AA)
    return image


def cameras_dir() -> str:
    """camslab/config/cameras in the source tree when this file is there (a colcon
    --symlink-install workspace or a plain checkout), else the installed share."""
    here = os.path.dirname(os.path.realpath(__file__))
    source = os.path.normpath(os.path.join(here, "..", "config", "cameras"))
    if os.path.isdir(source):
        return source
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("camslab"), "config", "cameras")


def set_camera_setting(settings_file: str, name: str) -> bool:
    """Sets `camera: <name>` in a settings file, keeping its comment; False when the file
    has no `camera` line."""
    with open(settings_file) as f:
        lines = f.read().splitlines(keepends=True)
    for i, line in enumerate(lines):
        if line.startswith("camera:"):
            _, _, comment = line.partition("#")
            lines[i] = f"camera: {name}".ljust(33) + (f"# {comment.strip()}\n" if comment else "\n")
            with open(settings_file, "w") as f:
                f.writelines(lines)
            return True
    return False


class Session:
    """One calibration run: feeds camera images to a Calibrator at most every `interval`
    seconds, keeps the view nearest the camera for the report, and writes the files.
    Driven by run() below from a camera device or a directory, and by `car calibrate`
    from a ROS image topic."""

    def __init__(self, name: str, grid: Grid = None, views: int = VIEWS,
                 interval: float = INTERVAL, report: str = "", save: str = "",
                 dictionary: str = DICTIONARY, border_bits: int = 0):
        self.name, self.grid = name, grid or Grid()
        self.views, self.interval = views, interval
        self.dictionary, self.border_bits = dictionary, border_bits
        self.report = os.path.abspath(os.path.expanduser(
            report or os.path.join("~", "camslab_calibration", name)))
        self.save = os.path.abspath(os.path.expanduser(save)) if save else ""
        self.calibrator = None
        self.frame = None                   # last image, BGR or grey
        self.count = 0                      # images fed
        self.last_kept = -math.inf          # time of the last kept view
        self.largest = (0.0, None)          # (board size, image) of the view nearest the camera
        self.coverage = Coverage()
        self.log = []                       # lines for the caller to print
        os.makedirs(self.report, exist_ok=True)
        if self.save:
            os.makedirs(self.save, exist_ok=True)

    @property
    def kept(self) -> int:
        return self.calibrator.views if self.calibrator else 0

    def done(self) -> bool:
        """At least `views` views and the coverage complete."""
        return self.kept >= self.views and self.coverage.complete()

    def feed(self, frame: np.ndarray, now: float, keep: bool = True) -> bool:
        """Processes one image: the tags are detected in every image, for the status and
        the picture, and the view is kept when `keep` is set, `interval` seconds have
        passed since the last kept view and it is new. True when it was kept."""
        import cv2

        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if self.calibrator is None:
            self.calibrator = Calibrator(self.grid, grey.shape[1], grey.shape[0],
                                         self.dictionary, self.border_bits)
            self.log.append(f"images of {grey.shape[1]} x {grey.shape[0]}")
        self.frame, self.count = frame, self.count + 1
        feature = self.calibrator.add(grey, keep and now - self.last_kept >= self.interval)
        self.log += self.calibrator.messages
        self.calibrator.messages.clear()
        if feature is None:
            return False
        self.last_kept = now
        n = self.kept
        tilt = math.degrees(math.asin(min(1.0, math.hypot(feature[3], feature[4]))))
        bins = self.coverage.add(feature)
        self.log.append(f"view {n}: {len(self.calibrator.object_points[-1]) // 4} tags, "
                        f"{bins}, tilt {tilt:.0f} deg")
        if feature[2] > self.largest[0]:
            self.largest = (feature[2], frame.copy())
        if self.save:
            cv2.imwrite(os.path.join(self.save, f"view_{n:03d}.png"), frame)
        return True

    def status(self) -> str:
        if self.calibrator is None:
            return "no image yet"
        return f"{self.calibrator.status()}, {self.kept} views kept, {self.coverage.text()}"

    def needed(self) -> str:
        """What is still needed before done(), or "" when nothing is."""
        missing = list(self.coverage.missing())
        if self.kept < self.views:
            missing.insert(0, f"{self.views - self.kept} more views")
        return ", ".join(missing)

    def picture(self, lines=()) -> np.ndarray:
        """The last image with the corners found in it, the coverage grid and the status
        drawn on it."""
        corners = self.calibrator.last_corners if self.calibrator else []
        needed = self.needed()
        return annotate(self.frame, corners,
                        [self.status(), *([f"need: {needed}"] if needed else []), *lines],
                        self.coverage)

    def write_latest(self, lines=()) -> None:
        import cv2

        if self.frame is not None:
            cv2.imwrite(os.path.join(self.report, "latest.png"), self.picture(lines))

    def finish(self):
        """Calibrates on the kept views and writes <name>.yaml, before_after.png and
        distortion_map.png into the report directory. Returns (rms [px], K, dist, yaml
        path); raises RuntimeError with fewer than MIN_VIEWS views."""
        import cv2

        self.write_latest()
        if self.calibrator is None:
            raise RuntimeError("no image from the camera")
        if self.kept < MIN_VIEWS:
            raise RuntimeError(f"{self.kept} board views, {MIN_VIEWS} needed; nothing "
                               f"calibrated. Last status: {self.calibrator.status()}; see "
                               f"{self.report}/latest.png")
        if self.calibrator.tilted_views() < 5:
            self.log.append(f"warning: {self.calibrator.tilted_views()} views tilted by more "
                            "than 20 degrees; the focal length needs tilted views")
        rms, K, dist = self.calibrator.calibrate()
        self.log.append(f"reprojection error {rms:.3f} px over {self.kept} views")
        self.log.append(f"fx {K[0, 0]:.1f}  fy {K[1, 1]:.1f}  cx {K[0, 2]:.1f}  cy {K[1, 2]:.1f}  "
                        f"distortion {np.array2string(dist, precision=4)}")
        if rms > 1.0:
            self.log.append("warning: reprojection error above 1 px; check the board is flat "
                            "and the images are sharp")
        width, height = self.calibrator.width, self.calibrator.height
        comment = (f"{self.name}: calibration of {datetime.date.today().isoformat()} with the "
                   f"{self.grid.cols} x {self.grid.rows} {self.calibrator.detector.name} board, "
                   f"{self.kept} views, reprojection error {rms:.3f} px.\n"
                   "Written by `ros2 run camslab car calibrate` (docs/hardware-survey.md).")
        path = os.path.join(self.report, f"{self.name}.yaml")
        with open(path, "w") as f:
            f.write(camera_info_yaml("camera", width, height, K, dist, comment))
        frame = self.largest[1]
        if frame.ndim == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(os.path.join(self.report, "before_after.png"), before_after(frame, K, dist))
        cv2.imwrite(os.path.join(self.report, "distortion_map.png"),
                    distortion_map(K, dist, width, height))
        self.log.append(f"wrote {self.report}/: {self.name}.yaml, before_after.png, "
                        "distortion_map.png")
        return rms, K, dist, path

    def install(self, settings_file: str = "") -> str:
        """Copies the calibration into config/cameras/ and names it in the `camera` setting
        of `settings_file`. Returns the installed path."""
        import shutil

        target = os.path.join(cameras_dir(), f"{self.name}.yaml")
        shutil.copyfile(os.path.join(self.report, f"{self.name}.yaml"), target)
        self.log.append(f"installed {target}")
        if settings_file and set_camera_setting(settings_file, self.name):
            self.log.append(f"set camera: {self.name} in {settings_file}")
        return target


INSTRUCTIONS = ("Show the board at different places, distances and tilts: near enough to fill "
                "most of the image and far enough to see all of it, in every corner and along "
                "every edge, tilted 20 to 45 degrees toward each side. Views are kept until the "
                "board has been seen in every part of the image, far, mid and near, flat and "
                "tilted each way; the status says what is still missing.")


def frames_from_device(device: str, width: int, height: int):
    import cv2

    capture = cv2.VideoCapture(int(device) if device.isdigit() else device)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        sys.exit(f"cannot open {device}; is another camera node still running?")
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


def flush(session: Session) -> None:
    for line in session.log:
        print(line, flush=True)
    session.log.clear()


def run(args) -> None:
    """Calibrates from a camera device or a directory of images."""
    import cv2

    session = Session(args.name, Grid(args.cols, args.rows, args.tag, args.spacing), args.views,
                      0.0 if args.images else args.interval, args.report, args.save,
                      args.dictionary, args.border_bits)
    frames = (frames_from_directory(args.images) if args.images
              else frames_from_device(args.device, args.width, args.height))
    print(f"{INSTRUCTIONS} A status line every {STATUS_PERIOD:.0f} s shows what the camera "
          f"sees, and {session.report}/latest.png the last image. Ctrl-C stops; with 5 views "
          "or more it still calibrates.", flush=True)
    status_time, status_count = time.monotonic(), 0
    try:
        for frame in frames:
            now = time.monotonic()
            session.feed(frame, now)
            flush(session)
            if now - status_time > STATUS_PERIOD:
                rate = (session.count - status_count) / (now - status_time)
                needed = session.needed()
                print(f"  {rate:.1f} images/s, {session.status()}"
                      f"{'; need ' + needed if needed else ''}", flush=True)
                session.write_latest()
                status_time, status_count = now, session.count
            if args.show:
                cv2.imshow("calibrate", session.picture())
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if session.done():
                break
    except KeyboardInterrupt:
        print("stopped", flush=True)
    try:
        session.finish()
    except RuntimeError as e:
        flush(session)
        sys.exit(str(e))
    if args.install:
        session.install(args.install if os.path.isfile(args.install) else "")
    flush(session)


def main(argv=None):
    parser = argparse.ArgumentParser(prog="calibrate", description=__doc__.split("\n\n")[0])
    parser.add_argument("--name", required=True,
                        help="calibration name, <camera model>_<width>x<height>; the "
                             "`camera` setting of survey.yaml")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--images", help="calibrate from the images in this directory "
                                         "instead of the camera")
    parser.add_argument("--views", type=int, default=VIEWS, help="views to keep at least")
    parser.add_argument("--interval", type=float, default=INTERVAL,
                        help="minimum time between kept views [s]")
    parser.add_argument("--save", help="directory to save the kept views in")
    parser.add_argument("--report", help="output directory; default "
                                         "~/camslab_calibration/<name>")
    parser.add_argument("--install", nargs="?", const="-", default="",
                        help="copy the result into config/cameras/; with a settings file, "
                             "set its `camera` too")
    parser.add_argument("--show", action="store_true", help="show the image (needs a display)")
    parser.add_argument("--cols", type=int, default=COLS, help="tags across")
    parser.add_argument("--rows", type=int, default=ROWS, help="tags down")
    parser.add_argument("--tag", type=float, default=TAG, help="tag side [m]")
    parser.add_argument("--spacing", type=float, default=SPACING, help="gap between tags [m]")
    parser.add_argument("--dictionary", default=DICTIONARY, help="cv2.aruco dictionary")
    parser.add_argument("--border-bits", type=int, default=0,
                        help="width of the tags' black border in bits, 2 for a Kalibr "
                             "AprilGrid, 1 for tags printed by OpenCV; 0: found from the "
                             "images")
    run(parser.parse_args(argv))


if __name__ == "__main__":
    main()
