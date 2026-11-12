"""ArUco markers as the pose sensor of camslab.fiducial on a real robot: the observation
C_T_Mj of every marker in an image, expressed in the camera frame convention of the
estimator (x forward, z up, like the robot frame), and the robot pose interpolated to the
image time. OpenCV is imported only where it is used, so the geometry here is testable
without it.
"""

import math
from collections import deque

import numpy as np

from .fiducial import I3, pose, rotation_angle, skew

# OpenCV's optical frame has z forward, x right and y down. The estimator's camera frame
# has x forward and z up; the extrinsic X = R_T_C is estimated for that frame:
# body x = optical z, body y = -optical x, body z = -optical y.
OPTICAL_TO_BODY = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rodrigues(rvec: np.ndarray) -> np.ndarray:
    """The rotation matrix of a rotation vector, as cv2.Rodrigues."""
    rvec = np.asarray(rvec, float).reshape(3)
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return I3.copy()
    K = skew(rvec / theta)
    return I3 + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def log_rotation(R: np.ndarray) -> np.ndarray:
    """The rotation vector of a rotation matrix, the inverse of rodrigues(); undefined at
    a rotation by pi."""
    theta = rotation_angle(R)
    if theta < 1e-12:
        return np.zeros(3)
    return theta / (2.0 * math.sin(theta)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                                                      R[1, 0] - R[0, 1]])


def marker_pose(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """C_T_M in the estimator's camera frame from the (rvec, tvec) of cv2.solvePnP, which
    give the marker pose in the optical frame: C_T_M = C_T_O O_T_M."""
    return pose(OPTICAL_TO_BODY @ rodrigues(rvec),
                OPTICAL_TO_BODY @ np.asarray(tvec, float).reshape(3))


def marker_corners(size: float) -> np.ndarray:
    """The four corners of a marker of side `size` in its own frame, the order of
    cv2.aruco: top left, top right, bottom right, bottom left, z out of the plane."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def intrinsics_of_file(path: str):
    """(K, dist, (width, height)) of a camera_info_manager calibration file, the format
    written by camslab.calibration."""
    import yaml

    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], float).reshape(3, 3)
    dist = np.array(data["distortion_coefficients"]["data"], float).reshape(-1)
    return K, dist, (int(data["image_width"]), int(data["image_height"]))


def nominal_intrinsics(width: int, height: int, horizontal_fov_deg: float) -> np.ndarray:
    """A camera matrix for an uncalibrated camera: the focal length from the image width
    and a nominal horizontal field of view, the principal point at the image centre. The
    marker distances then carry the focal-length error until the camera is calibrated."""
    f = (width / 2.0) / math.tan(math.radians(horizontal_fov_deg) / 2.0)
    return np.array([[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]])


class Detector:
    """ArUco detection and pose estimation with OpenCV, both the 4.7+ ArucoDetector API
    and the older module functions. `dictionary` is a cv2.aruco name such as
    "DICT_4X4_50"; `marker_size` is the side of the printed marker [m]."""

    def __init__(self, dictionary: str = "DICT_4X4_50", marker_size: float = 0.14):
        import cv2

        self.cv2 = cv2
        self.marker_size = float(marker_size)
        self.corners = marker_corners(self.marker_size)
        self.K = None
        self.dist = None
        self.corners_found, self.ids_found = [], None   # of the last detect(), for drawing
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
            self._detect = self.detector.detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda image: cv2.aruco.detectMarkers(image, aruco_dict,
                                                                 parameters=params)

    def set_intrinsics(self, K, dist) -> None:
        """The camera matrix (3, 3) and distortion coefficients of sensor_msgs/CameraInfo."""
        self.K = np.asarray(K, float).reshape(3, 3)
        self.dist = np.asarray(dist, float).reshape(-1)

    def detect(self, image) -> dict:
        """{marker id: C_T_M} for the markers in `image` (grey or BGR). Empty until
        set_intrinsics() was called, but corners_found and ids_found are filled anyway."""
        corners, ids, _ = self._detect(image)
        self.corners_found, self.ids_found = corners, ids
        found = {}
        if ids is None or self.K is None:
            return found
        for quad, marker_id in zip(corners, ids.reshape(-1)):
            ok, rvec, tvec = self.cv2.solvePnP(self.corners, quad.reshape(4, 2), self.K,
                                               self.dist, flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                found[int(marker_id)] = marker_pose(rvec, tvec)
        return found

    def draw(self, image) -> None:
        """Draws the markers of the last detect() on `image` (BGR), in place."""
        if self.ids_found is not None:
            self.cv2.aruco.drawDetectedMarkers(image, self.corners_found, self.ids_found)


class PoseHistory:
    """The robot poses of the last `horizon` seconds, for the pose at an image time:
    position interpolated linearly, rotation along the geodesic between the two
    neighbouring poses."""

    def __init__(self, horizon: float = 2.0):
        self.horizon = horizon
        self.entries = deque()

    def push(self, t: float, T: np.ndarray) -> None:
        self.entries.append((float(t), np.array(T, float)))
        while self.entries and self.entries[0][0] < t - self.horizon:
            self.entries.popleft()

    def latest(self):
        return self.entries[-1][1] if self.entries else None

    def at(self, t: float):
        """W_T_R at time t, or None when t is outside the stored interval."""
        if not self.entries or t < self.entries[0][0] or t > self.entries[-1][0]:
            return None
        after = next(i for i, (ti, _) in enumerate(self.entries) if ti >= t)
        t1, T1 = self.entries[after]
        if after == 0 or t1 == t:
            return T1.copy()
        t0, T0 = self.entries[after - 1]
        s = (t - t0) / (t1 - t0)
        R0, R1 = T0[:3, :3], T1[:3, :3]
        R = R0 @ rodrigues(s * log_rotation(R0.T @ R1))
        return pose(R, (1.0 - s) * T0[:3, 3] + s * T1[:3, 3])

    def angular_speed(self, t: float, span: float = 0.1):
        """The robot's turning rate [rad/s] around time t: the rotation between the poses
        at t - span / 2 and t + span / 2, both moved into the stored interval, over the
        time between them. None when the history is empty or holds one instant."""
        if not self.entries:
            return None
        first, last = self.entries[0][0], self.entries[-1][0]
        t0 = min(max(t - span / 2.0, first), last)
        t1 = min(max(t + span / 2.0, first), last)
        if t1 <= t0:
            return None
        R0, R1 = self.at(t0)[:3, :3], self.at(t1)[:3, :3]
        return float(np.linalg.norm(log_rotation(R0.T @ R1))) / (t1 - t0)


class Measurements:
    """What the estimator receives on hardware: the robot pose at the time of each
    camera image and the markers of that image. The node runs its estimator on a timer
    that is faster than the camera, so next() answers every tick:

    - a tick with a new image gets the pose at the image time minus `camera_delay` and
      the image's markers, or the pose and no markers when the robot turned faster
      than `max_yaw_rate` (PoseHistory.angular_speed);
    - a tick without one gets the pose of the last image again and no markers. The
      estimator measures its travel between the poses it is given, so they must all be
      of image times: the newest pose is about 0.1 s ahead of the image's, and
      alternating between the two counts every step back and forth;
    - an image whose time has no pose (the car's clock is off, or the motion capture
      stopped) is dropped, because a marker paired with a pose of another time is a
      wrong measurement.

    `used`, `turning` and `unpaired` count the images with markers that were given to
    the estimator, skipped in a turn and dropped without a pose."""

    def __init__(self, camera_delay: float = 0.0, max_yaw_rate: float = 0.0,
                 horizon: float = 2.0):
        self.camera_delay, self.max_yaw_rate = float(camera_delay), float(max_yaw_rate)
        self.history = PoseHistory(horizon)
        self.pending = None             # (image time, {j: C_T_Mj}) of the newest image
        self.last = None                # the pose given with the last image
        self.used = self.turning = self.unpaired = 0

    def pose(self, t: float, T: np.ndarray) -> None:
        """A robot pose W_T_R measured at time t."""
        self.history.push(t, T)

    def image(self, stamp: float, found: dict) -> None:
        """A camera image taken at `stamp` with the markers {j: C_T_Mj}, empty when it
        shows none. Only the newest image waits for the next tick."""
        self.pending = (float(stamp), dict(found))

    def next(self) -> tuple:
        """(W_T_R, {j: C_T_Mj}) for the next estimator update; W_T_R is None before
        the first pose."""
        held = self.last if self.last is not None else self.history.latest()
        if self.pending is None:
            return held, {}
        stamp, found = self.pending
        self.pending = None
        t = stamp - self.camera_delay
        T_R = self.history.at(t)
        if T_R is None:
            self.unpaired += bool(found)
            return held, {}
        self.last = T_R
        if found and self.max_yaw_rate > 0.0:
            rate = self.history.angular_speed(t)
            if rate is not None and rate > self.max_yaw_rate:
                self.turning += 1
                return T_R, {}
        self.used += bool(found)
        return T_R, found
