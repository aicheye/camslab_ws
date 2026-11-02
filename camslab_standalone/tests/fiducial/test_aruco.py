"""camslab.aruco: the ArUco observations in the estimator's camera frame and the robot
pose at the image time. No OpenCV needed."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camslab.aruco import (OPTICAL_TO_BODY, PoseHistory, log_rotation, marker_corners,
                           marker_pose, rodrigues)
from camslab.fiducial import Scene, inverse, pose, robot_pose, rotation_angle
from camslab.types import State


def test_rodrigues_matches_scipy():
    rng = np.random.default_rng(0)
    for _ in range(5):
        rvec = rng.normal(size=3)
        assert rodrigues(rvec) == pytest.approx(Rotation.from_rotvec(rvec).as_matrix())
        assert log_rotation(rodrigues(rvec)) == pytest.approx(rvec)


def test_optical_axis_becomes_the_camera_x_axis():
    """A marker 1 m ahead of the camera is at z = 1 in the optical frame and at x = 1 in
    the estimator's camera frame; the frame change is a rotation."""
    T = marker_pose(np.zeros(3), [0.0, 0.0, 1.0])
    assert T[:3, 3] == pytest.approx([1.0, 0.0, 0.0])
    assert OPTICAL_TO_BODY @ OPTICAL_TO_BODY.T == pytest.approx(np.eye(3))
    assert np.linalg.det(OPTICAL_TO_BODY) == pytest.approx(1.0)


def test_marker_pose_inverts_a_simulated_observation():
    """What solvePnP would return for a simulated observation, mapped back, is that
    observation: the simulated sensor and the real one agree on the camera frame."""
    scene = Scene()
    T_R = robot_pose(State.from_pose(2.0, 1.0, 0.3))
    for j, C_T_M in scene.observe(T_R).items():
        O_T_M = pose(OPTICAL_TO_BODY.T, np.zeros(3)) @ C_T_M      # the optical frame's view
        rvec = log_rotation(O_T_M[:3, :3])
        assert marker_pose(rvec, O_T_M[:3, 3]) == pytest.approx(C_T_M)


def test_marker_corners_are_a_square_in_the_marker_plane():
    c = marker_corners(0.14)
    assert c.shape == (4, 3) and np.all(c[:, 2] == 0.0)
    assert np.linalg.norm(c[0] - c[1]) == pytest.approx(0.14)
    assert np.linalg.norm(c[0] - c[2]) == pytest.approx(0.14 * np.sqrt(2.0))


def test_pose_history_interpolates_between_samples():
    history = PoseHistory(horizon=1.0)
    history.push(0.0, robot_pose(State.from_pose(0.0, 0.0, 0.0)))
    history.push(0.1, robot_pose(State.from_pose(0.1, 0.0, 0.2)))
    T = history.at(0.05)
    assert T[:3, 3] == pytest.approx([0.05, 0.0, 0.0])
    assert rotation_angle(inverse(robot_pose(State.from_pose(0.0, 0.0, 0.1))) @ T) == pytest.approx(0.0, abs=1e-9)
    assert history.at(0.1) == pytest.approx(history.latest())
    assert history.at(0.2) is None and history.at(-0.1) is None
    history.push(1.5, robot_pose(State.from_pose(1.0, 0.0, 0.0)))
    assert history.at(0.05) is None            # older than the horizon, dropped
