"""camslab_standalone.fiducial.recorded: a survey written as a rosbag2 recording from the
simulator, read back without ROS and fitted in a batch."""

import math

import numpy as np
import pytest

from camslab import fiducial
from camslab.aruco import PoseHistory
from camslab.fiducial import Noise, Scene
from camslab.types import Params, State
from camslab_standalone.fiducial import recorded
from camslab_standalone.fiducial.offline import RecordingObserver
from camslab_standalone.tracking.simulate import simulate

pytest.importorskip("rosbags")

DELAY = 0.06
SETTINGS = {"marker_ids": [4, 7, 9], "marker_height": [0.0, 0.0, 0.4],
            "marker_height_known": [True, True, False], "camera_delay": DELAY,
            "max_yaw_rate": 0.0}
NOISE = Noise(marker_position=0.004, marker_rotation=math.radians(0.3))


def quaternion(R):
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(R).as_quat()


def write_bag(path, name="angostura", ids=(4, 7, 9), delay=DELAY, dt=0.05, t_max=35.0, seed=0):
    """A recording of the simulator on the lab scene: the mocap pose at every step, an
    image at every step stamped `delay` after its pose, and the online estimate. Returns
    the true parameters."""
    from rosbags.rosbag2 import Writer

    store = recorded.typestore()
    types = store.types
    Time, Header = types["builtin_interfaces/msg/Time"], types["std_msgs/msg/Header"]
    Point, Quaternion = types["geometry_msgs/msg/Point"], types["geometry_msgs/msg/Quaternion"]
    Pose, PoseStamped = types["geometry_msgs/msg/Pose"], types["geometry_msgs/msg/PoseStamped"]
    PoseWithCovariance = types["geometry_msgs/msg/PoseWithCovariance"]
    Marker, MarkerArray = types["aruco_msgs/msg/Marker"], types["aruco_msgs/msg/MarkerArray"]
    Layout, Dimension = types["std_msgs/msg/MultiArrayLayout"], types["std_msgs/msg/MultiArrayDimension"]
    Float64MultiArray = types["std_msgs/msg/Float64MultiArray"]

    def header(t, frame):
        return Header(stamp=Time(sec=int(t), nanosec=int(round((t % 1.0) * 1e9)) % 10**9), frame_id=frame)

    def pose_msg(T):
        q = quaternion(T[:3, :3])
        return Pose(position=Point(x=T[0, 3], y=T[1, 3], z=T[2, 3]),
                    orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))

    scene = Scene()
    observer = RecordingObserver(scene, noise=NOISE, rng=np.random.default_rng(seed))
    params = Params(shape="bowtie", x0=3.0, y0=3.0, a=2.0, b=1.2, period=40.0)
    rows = np.array(simulate(State.from_pose(3.0, 3.0, 0.5), params, dt, t_max, observer), float)
    columns = fiducial.columns(scene.n)
    t0 = 1_800_000_000.0
    with Writer(path, version=8) as writer:
        pose_c = writer.add_connection(f"/{name}/pose", "geometry_msgs/msg/PoseStamped", typestore=store)
        obs_c = writer.add_connection(f"/{name}/observations", "aruco_msgs/msg/MarkerArray", typestore=store)
        est_c = writer.add_connection(f"/{name}/estimate", "std_msgs/msg/Float64MultiArray", typestore=store)
        for row, (T_R, observations) in zip(rows, observer.data):
            t = t0 + row[0]
            stamp_ns = int(t * 1e9)
            msg = PoseStamped(header=header(t, "map"), pose=pose_msg(T_R))
            writer.write(pose_c, stamp_ns, store.serialize_cdr(msg, pose_c.msgtype))
            h = header(t + delay, f"{name}/camera")
            markers = [Marker(header=h, id=ids[j], confidence=1.0,
                              pose=PoseWithCovariance(pose=pose_msg(T), covariance=np.zeros(36)))
                       for j, T in observations.items()]
            # a marker that is not in marker_ids, as a wall poster would decode
            markers.append(Marker(header=h, id=33, confidence=1.0,
                                  pose=PoseWithCovariance(pose=pose_msg(np.eye(4)), covariance=np.zeros(36))))
            msg = MarkerArray(header=h, markers=markers)
            writer.write(obs_c, stamp_ns + int(delay * 1e9), store.serialize_cdr(msg, obs_c.msgtype))
            values = np.nan_to_num(row[10:10 + len(columns)], nan=float("nan"))
            msg = Float64MultiArray(
                layout=Layout(dim=[Dimension(label=",".join(columns), size=len(columns), stride=len(columns))],
                              data_offset=0), data=values)
            writer.write(est_c, stamp_ns + int(delay * 1e9), store.serialize_cdr(msg, est_c.msgtype))
    return observer.truth, len(rows)


@pytest.fixture(scope="module")
def survey(tmp_path_factory):
    path = tmp_path_factory.mktemp("bag") / "survey"
    truth, samples = write_bag(path)
    recording = recorded.read_bag(str(path))
    return truth, samples, recording, recorded.analyse(recording, SETTINGS)


def test_the_recording_is_read_without_ros(survey):
    truth, samples, recording, _ = survey
    assert recording.name == "angostura"
    assert len(recording.pose_time) == samples and recording.poses.shape == (samples, 4, 4)
    assert len(recording.images) == samples
    stamp, markers = recording.images[10]
    assert set(markers) == {4, 7, 9, 33} and markers[4].shape == (4, 4)
    assert stamp == pytest.approx(recording.pose_time[10] + DELAY, abs=1e-6)
    assert recording.columns == fiducial.columns(3)
    assert recording.estimates.shape == (samples, len(recording.columns))


def test_the_pose_track_interpolates_like_the_estimator_node(survey):
    _, _, recording, _ = survey
    track = recorded.PoseTrack(recording.pose_time, recording.poses)
    history = PoseHistory(horizon=math.inf)
    for t, T in zip(recording.pose_time, recording.poses):
        history.push(t, T)
    for t in recording.pose_time[0] + np.array([0.0, 0.013, 7.777, 20.5]):
        assert track.at(t) == pytest.approx(history.at(t), abs=1e-12)
        assert track.angular_speed(t) == pytest.approx(history.angular_speed(t), abs=1e-9)
    assert track.at(recording.pose_time[0] - 1.0) is None and track.at(recording.pose_time[-1] + 1.0) is None


def test_the_batch_fit_recovers_the_truth_and_the_noise(survey):
    truth, samples, _, result = survey
    assert result["ids"] == [4, 7, 9] and result["unseen"] == []
    assert result["counts"] == {"used": samples, "empty": 0, "turning": 0, "no pose": 0}
    e = fiducial.errors(result["batch"]["estimate"], truth, 3)
    assert e[0] < 2e-3 and e[1] < 5e-3 and e[2] < 5e-3 and e[3] < 5e-3
    assert result["batch"]["rms_position"] == pytest.approx(NOISE.marker_position, rel=0.15)
    assert result["batch"]["rms_rotation"] == pytest.approx(NOISE.marker_rotation, rel=0.15)
    assert all(d <= r for d, r in zip(e, result["batch_radii"]))


def test_the_online_estimate_is_compared_with_the_batch(survey):
    _, samples, _, result = survey
    online = result["online"]
    assert online["distance"].shape == (samples, 4) and online["radii"].shape == (samples, 4)
    assert online["time"][0] == 0.0 and online["time"][-1] == pytest.approx(35.0, abs=0.01)
    assert online["distance"][-1][1] < online["distance"][0][1]    # it approached the batch optimum
    recorded.report(result)


def test_the_delay_scan_finds_the_camera_delay(survey):
    _, _, _, result = survey
    best = min(result["scan"], key=lambda row: row[1])
    assert best[0] == pytest.approx(DELAY, abs=1e-9)
    worst = max(result["scan"], key=lambda row: row[1])
    assert worst[1] > 2.0 * best[1]


def test_turning_images_and_unseen_markers_are_left_out(survey):
    _, _, recording, _ = survey
    result = recorded.analyse(recording, {**SETTINGS, "max_yaw_rate": 0.2,
                                          "marker_ids": [4, 7, 9, 12],
                                          "marker_height": [0.0, 0.0, 0.4, 0.1],
                                          "marker_height_known": [True, True, False, True]})
    assert result["counts"]["turning"] > 0
    assert result["ids"] == [4, 7, 9] and result["unseen"] == [12]
    assert result["online"] is None                 # the recorded estimate has three markers


def test_a_bag_without_the_topics_is_refused(tmp_path):
    from rosbags.rosbag2 import Writer

    store = recorded.typestore()
    with Writer(tmp_path / "empty", version=8) as writer:
        writer.add_connection("/angostura/pose", "geometry_msgs/msg/PoseStamped", typestore=store)
    with pytest.raises(ValueError, match="no observations topic"):
        recorded.read_bag(str(tmp_path / "empty"))
