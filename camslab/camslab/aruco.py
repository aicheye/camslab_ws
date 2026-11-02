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
        set_intrinsics() was called."""
        if self.K is None:
            return {}
        corners, ids, _ = self._detect(image)
        found = {}
        if ids is None:
            return found
        for quad, marker_id in zip(corners, ids.reshape(-1)):
            ok, rvec, tvec = self.cv2.solvePnP(self.corners, quad.reshape(4, 2), self.K,
                                               self.dist, flags=self.cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                found[int(marker_id)] = marker_pose(rvec, tvec)
        return found


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
