"""camslab.aruco: the ArUco observations in the estimator's camera frame and the robot
pose at the image time. No OpenCV needed."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camslab.aruco import (OPTICAL_TO_BODY, Measurements, PoseHistory, log_rotation, marker_corners,
                           marker_pose, nominal_intrinsics, rodrigues)
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


def test_pose_history_gives_the_turning_rate():
    history = PoseHistory(horizon=2.0)
    assert history.angular_speed(0.0) is None
    for k in range(101):                    # 1 s at 100 Hz, turning at 0.8 rad/s
        t = 0.01 * k
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("z", 0.8 * t).as_matrix()
        history.push(t, T)
    assert history.angular_speed(0.5) == pytest.approx(0.8)
    # at the ends the interval is cut to the stored poses
    assert history.angular_speed(1.0) == pytest.approx(0.8)
    assert history.angular_speed(0.0) == pytest.approx(0.8)
    assert history.angular_speed(5.0) is None


def straight_drive(measurements, speed=0.14, seconds=10.0, latency=0.02, images=30.0):
    """The estimator node's loop on a straight drive: poses at 100 Hz, a timer at 50 Hz,
    images at `images` Hz that arrive `latency` after they were taken. Yields what
    next() gives each tick."""
    line = lambda t: pose(np.eye(3), [speed * t, 0.0, 0.0])
    image_times = np.arange(0.0, seconds, 1.0 / images)
    sent, shown = 0, 0
    for now in np.arange(0.0, seconds, 0.02):
        while sent * 0.01 <= now:
            measurements.pose(sent * 0.01, line(sent * 0.01))
            sent += 1
        while shown < len(image_times) and image_times[shown] + latency <= now:
            # stamped when taken; the pose that belongs to it is camera_delay earlier
            measurements.image(image_times[shown] + measurements.camera_delay, {0: np.eye(4)})
            shown += 1
        yield measurements.next()


def test_the_poses_given_to_the_estimator_are_all_of_image_times():
    """The estimator adds up the distance between the poses it is given. Mixing the
    newest pose with the older pose of an image counted every step back and forth, 3.4
    times the distance driven, which shrank the travel of d_step and l_buffer."""
    from camslab.fiducial import Estimator

    measurements = Measurements(camera_delay=0.05)
    estimator = Estimator(1, {})
    given = []
    for T_R, found in straight_drive(measurements):
        estimator.ingest(T_R, {})
        given.append((T_R[0, 3], bool(found)))
    assert estimator.travel == pytest.approx(0.14 * 10.0, rel=0.03)
    x = np.array([g[0] for g in given])
    assert (np.diff(x) >= -1e-12).all()                 # the pose never steps back
    assert measurements.used == sum(g[1] for g in given) > 250
    # a marker is given with the pose of its own image: image k was taken at k / 30 s
    with_marker = x[[g[1] for g in given]]
    assert np.abs(with_marker / 0.14 * 30.0 - np.round(with_marker / 0.14 * 30.0)).max() < 1e-6


def test_an_image_without_a_pose_or_in_a_turn_gives_no_marker():
    measurements = Measurements(camera_delay=0.0, max_yaw_rate=0.5)
    assert measurements.next() == (None, {})
    for k in range(101):
        t = 0.01 * k
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler("z", (0.2 if t < 0.5 else 0.8) * t).as_matrix()
        measurements.pose(t, T)
    measurements.image(0.25, {0: np.eye(4)})
    T_R, found = measurements.next()
    assert list(found) == [0] and T_R == pytest.approx(measurements.history.at(0.25))
    assert measurements.next()[1] == {} and measurements.next()[0] == pytest.approx(T_R)
    measurements.image(0.75, {0: np.eye(4)})            # turning at 0.8 rad/s
    T_turn, found = measurements.next()
    assert found == {} and T_turn == pytest.approx(measurements.history.at(0.75))
    measurements.image(5.0, {0: np.eye(4)})             # the car's clock is ahead
    T_held, found = measurements.next()
    assert found == {} and T_held == pytest.approx(T_turn)
    measurements.image(0.3, {})                         # no marker in the image
    assert measurements.next()[1] == {}
    assert (measurements.used, measurements.turning, measurements.unpaired) == (1, 1, 1)


def test_pure_translation_pairs_form_with_motion_capture_noise():
    """A pure-translation pair is two robot poses whose orientations differ by less than
    straight_tol. The simulation's 0.0005 rad is below the orientation noise of a
    motion capture pose, so on hardware almost no pair passed it and the orientation law
    never switched on; the survey's value passes the pairs of a straight drive."""
    from camslab import car
    from camslab.fiducial import Estimator

    rng = np.random.default_rng(0)
    scene = Scene()
    noise = np.radians(0.1)                             # per axis, each pose
    stored = {}
    for tol in (5e-4, car.DEFAULTS["straight_tol"]):
        estimator = Estimator(scene.n, {}, straight_tol=tol)
        for k in range(300):                            # 3 m straight, a pose every 10 mm
            T_R = robot_pose(State.from_pose(0.01 * k, 0.0, 0.0))
            measured = T_R.copy()
            measured[:3, :3] = T_R[:3, :3] @ rodrigues(rng.normal(0.0, noise, 3))
            estimator.ingest(measured, scene.observe(T_R))
        stored[tol] = len(estimator.H_x)
    assert stored[5e-4] <= 3
    assert stored[car.DEFAULTS["straight_tol"]] >= 60


def test_nominal_intrinsics_span_the_field_of_view():
    K = nominal_intrinsics(640, 480, 90.0)
    assert K[0, 2] == 320.0 and K[1, 2] == 240.0 and K[0, 0] == K[1, 1]
    assert K[0, 0] == pytest.approx(320.0)          # 90 degrees: f = half width
