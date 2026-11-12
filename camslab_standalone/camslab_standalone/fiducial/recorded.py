"""Batch check of a recorded survey. On hardware the true camera mount and marker poses
are unknown, so the reference for the online estimate is the batch least squares over
every recorded observation (offline.batch_fit).

    ros2 bag record /<name>/pose /<name>/observations /<name>/estimate
    uv run python -m camslab_standalone.fiducial.recorded <bag directory> [--out PNG]

The recording is read without ROS. The robot pose is taken at the image time minus
`camera_delay`, and images taken above `max_yaw_rate` are skipped, as the estimator node
does; both, and the marker ids and heights, come from camslab/config/survey.yaml. The
program prints

- the residuals of the fit: the spread of the camera's marker positions and
  orientations, which is the measurement noise of this camera,
- the online estimate against the batch optimum, with the radii of both,
- the residual for a range of camera delays, whose minimum is the delay of this camera,

and plots the distance of the online estimate to the batch optimum over the run.
"""

import argparse
import math
import os
from dataclasses import dataclass

import numpy as np

import camslab
from camslab import car
from camslab.aruco import log_rotation, rodrigues
from camslab.fiducial import ERROR_FIELDS, RADIUS_FIELDS, Noise, errors, pose

from .offline import batch_fit, batch_radii

# The message definitions of aruco_msgs (ros-<distro>-aruco-msgs), which the reader does
# not know by itself.
ARUCO_MARKER = """
std_msgs/Header header
uint32 id
geometry_msgs/PoseWithCovariance pose
float64 confidence
"""
ARUCO_MARKER_ARRAY = """
std_msgs/Header header
aruco_msgs/Marker[] markers
"""
# The measurement noise assumed for the first fit: the values of the rendered images of
# docs/hardware-survey.md and a marker height measured to 2 mm. The second fit uses the
# residuals of the first.
ASSUMED = Noise(marker_position=0.013, marker_rotation=math.radians(0.7), height=0.002)
DELAYS = np.arange(-0.10, 0.1001, 0.02)     # scanned around the configured camera_delay [s]
YAW_RATE_SPAN = 0.1                         # as PoseHistory.angular_speed [s]


def typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register({**get_types_from_msg(ARUCO_MARKER, "aruco_msgs/msg/Marker"),
                    **get_types_from_msg(ARUCO_MARKER_ARRAY, "aruco_msgs/msg/MarkerArray")})
    return store


def stamp_seconds(stamp) -> float:
    return stamp.sec + 1e-9 * stamp.nanosec


def quaternion_rotation(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose_of(msg) -> np.ndarray:
    return pose(quaternion_rotation(msg.orientation),
                [msg.position.x, msg.position.y, msg.position.z])


@dataclass
class Recording:
    """A survey as recorded: the robot poses (time [s], W_T_R), the images (time [s],
    {marker id: C_T_M}) and the online estimates (time [s], row of `columns`)."""

    name: str
    pose_time: np.ndarray
    poses: np.ndarray
    images: list
    estimate_time: np.ndarray
    estimates: np.ndarray
    columns: tuple


def read_bag(path: str) -> Recording:
    """The pose, observations and estimate topics of a rosbag2 directory. The robot name
    is the namespace of the observations topic. A ValueError names a missing topic."""
    from rosbags.rosbag2 import Reader

    store = typestore()
    pose_time, poses, images, estimate_time, estimates, columns = [], [], [], [], [], ()
    with Reader(path) as reader:
        topics = {c.topic: c for c in reader.connections}
        found = [t for t in topics if t.endswith("/observations")
                 and topics[t].msgtype == "aruco_msgs/msg/MarkerArray"]
        if not found:
            raise ValueError(f"{path} has no observations topic; it holds {sorted(topics)}")
        name = found[0][:-len("/observations")].strip("/")
        wanted = {f"/{name}/pose": "pose", f"/{name}/observations": "observations",
                  f"/{name}/estimate": "estimate"}
        missing = [t for t in wanted if t not in topics]
        if missing:
            raise ValueError(f"{path} has no {' and no '.join(missing)}; record them with "
                             f"`ros2 bag record {' '.join(wanted)}`")
        for connection, received, raw in reader.messages(
                connections=[topics[t] for t in wanted]):
            msg = store.deserialize_cdr(raw, connection.msgtype)
            kind = wanted[connection.topic]
            if kind == "pose":
                stamp = stamp_seconds(msg.header.stamp)
                pose_time.append(stamp if stamp > 0.0 else 1e-9 * received)
                poses.append(pose_of(msg.pose))
            elif kind == "observations":
                images.append((stamp_seconds(msg.header.stamp),
                               {int(m.id): pose_of(m.pose.pose) for m in msg.markers}))
            else:
                if msg.layout.dim:
                    columns = tuple(msg.layout.dim[0].label.split(","))
                estimate_time.append(1e-9 * received)
                estimates.append(np.asarray(msg.data, float))
    order = np.argsort(pose_time, kind="stable")
    width = max((len(e) for e in estimates), default=0)
    rows = np.array([e for e in estimates if len(e) == width]).reshape(-1, width)
    times = np.array([t for t, e in zip(estimate_time, estimates) if len(e) == width])
    return Recording(name, np.array(pose_time)[order], np.array(poses).reshape(-1, 4, 4)[order],
                     images, times, rows, columns)


class PoseTrack:
    """The recorded robot poses with the interpolation of camslab.aruco.PoseHistory:
    position linear, rotation along the geodesic."""

    def __init__(self, time: np.ndarray, poses: np.ndarray):
        keep = np.concatenate([[True], np.diff(time) > 0.0]) if len(time) else np.zeros(0, bool)
        self.time, self.poses = time[keep], poses[keep]

    def at(self, t: float):
        """W_T_R at time t, or None outside the recording."""
        if len(self.time) == 0 or t < self.time[0] or t > self.time[-1]:
            return None
        k = int(np.searchsorted(self.time, t))
        if k == 0 or self.time[k] == t:
            return self.poses[k].copy()
        t0, t1, T0, T1 = self.time[k - 1], self.time[k], self.poses[k - 1], self.poses[k]
        s = (t - t0) / (t1 - t0)
        R0, R1 = T0[:3, :3], T1[:3, :3]
        return pose(R0 @ rodrigues(s * log_rotation(R0.T @ R1)),
                    (1.0 - s) * T0[:3, 3] + s * T1[:3, 3])

    def angular_speed(self, t: float, span: float = YAW_RATE_SPAN):
        """The turning rate [rad/s] around time t, as PoseHistory.angular_speed."""
        if len(self.time) == 0:
            return None
        t0 = min(max(t - span / 2.0, self.time[0]), self.time[-1])
        t1 = min(max(t + span / 2.0, self.time[0]), self.time[-1])
        if t1 <= t0:
            return None
        R0, R1 = self.at(t0)[:3, :3], self.at(t1)[:3, :3]
        return float(np.linalg.norm(log_rotation(R0.T @ R1))) / (t1 - t0)


def measurements(recording: Recording, ids, delay: float, max_yaw_rate: float) -> tuple:
    """(data, counts): data is what the estimator node gives the estimator, a list of
    (W_T_R at the image time minus delay, {j: C_T_Mj}) with j the index of the marker id
    in `ids`; counts says how many images were used, had no marker of `ids`, were taken
    in a turn above max_yaw_rate, or had no pose."""
    track = PoseTrack(recording.pose_time, recording.poses)
    index = {int(marker_id): j for j, marker_id in enumerate(ids)}
    data, counts = [], {"used": 0, "empty": 0, "turning": 0, "no pose": 0}
    for stamp, markers in recording.images:
        found = {index[i]: T for i, T in markers.items() if i in index}
        if not found:
            counts["empty"] += 1
            continue
        t = stamp - delay
        T_R = track.at(t)
        if T_R is None:
            counts["no pose"] += 1
            continue
        if max_yaw_rate > 0.0:
            rate = track.angular_speed(t)
            if rate is not None and rate > max_yaw_rate:
                counts["turning"] += 1
                continue
        counts["used"] += 1
        data.append((T_R, found))
    return data, counts


def select(estimate: np.ndarray, n: int, keep) -> np.ndarray:
    """The entries of an estimate of n markers for the camera and the markers `keep`."""
    estimate = np.asarray(estimate, float)
    m = estimate[6:6 + 3 * n].reshape(n, 3)[list(keep)]
    t = estimate[6 + 3 * n:6 + 6 * n].reshape(n, 3)[list(keep)]
    return np.concatenate([estimate[:6], m.ravel(), t.ravel()])


def fit(data: list, heights: dict, n: int, start: np.ndarray) -> dict:
    """offline.batch_fit from `start` and from zero, the one with the lower cost, fitted
    again with the measurement noise set to its own residuals so that the covariance
    belongs to this camera."""
    first = min((batch_fit(data, heights, n, ASSUMED, s) for s in (start, np.zeros_like(start))),
                key=lambda f: f["cost"])
    noise = Noise(marker_position=max(first["rms_position"], 1e-6),
                  marker_rotation=max(first["rms_rotation"], 1e-6), height=ASSUMED.height)
    return batch_fit(data, heights, n, noise, first["estimate"])


def analyse(recording: Recording, settings: dict) -> dict:
    """The batch fit of a recording against its online estimate. Markers of
    `marker_ids` that no used image shows are left out. Returns the numbers that
    report() prints."""
    ids = [int(i) for i in settings["marker_ids"]]
    delay = float(settings.get("camera_delay", 0.0))
    max_yaw_rate = float(settings.get("max_yaw_rate", 0.0))
    data, counts = measurements(recording, ids, delay, max_yaw_rate)
    if not data:
        raise ValueError(f"no image of the recording can be used: {counts}")
    seen = sorted({j for _, found in data for j in found})
    renumber = {j: k for k, j in enumerate(seen)}
    known = settings.get("marker_height_known") or [False] * len(ids)
    heights = {renumber[j]: float(settings["marker_height"][j]) for j in seen if known[j]}

    def renumbered(d):
        return [(T_R, {renumber[j]: T for j, T in found.items()}) for T_R, found in d]

    n_all, n = len(ids), len(seen)
    width = 6 + 6 * n_all
    # The recorded estimate belongs to these markers only if it has as many: its columns
    # start with the 6 + 6 n estimated parameters, named "<parameter>_hat".
    recorded_width = (sum(c.endswith("_hat") for c in recording.columns) if recording.columns
                      else recording.estimates.shape[1])
    online = recording.estimates[:, :width] if recorded_width == width else None
    start = select(online[-1], n_all, seen) if online is not None and len(online) else np.zeros(6 + 6 * n)
    batch = fit(renumbered(data), heights, n, start)
    result = {"name": recording.name, "ids": [ids[j] for j in seen],
              "unseen": [ids[j] for j in range(n_all) if j not in seen], "counts": counts,
              "per_marker": {ids[j]: sum(j in found for _, found in data) for j in seen},
              "delay": delay, "batch": batch, "batch_radii": batch_radii(batch["cov"], n),
              "n": n, "online": None}
    if online is not None and len(online):
        distance = np.array([errors(select(row, n_all, seen), batch["estimate"], n) for row in online])
        radii = np.full((len(online), 4), np.nan)
        if all(name in recording.columns for name in RADIUS_FIELDS):
            radii = recording.estimates[:, [recording.columns.index(r) for r in RADIUS_FIELDS]]
        result["online"] = {"time": recording.estimate_time - recording.estimate_time[0],
                            "estimate": select(online[-1], n_all, seen),
                            "distance": distance, "radii": radii}
    scan = []
    for d in delay + DELAYS:
        scanned, used = measurements(recording, ids, float(d), max_yaw_rate)
        if len(scanned) < 0.5 * len(data):
            continue
        scanned_fit = batch_fit(renumbered(scanned), heights, n, ASSUMED, batch["estimate"])
        scan.append((float(d), scanned_fit["rms_position"], scanned_fit["rms_rotation"]))
    result["scan"] = scan
    return result


def report(result: dict) -> None:
    n, batch, counts = result["n"], result["batch"], result["counts"]
    print(f"{result['name']}: {counts['used']} images used, {counts['turning']} skipped in a "
          f"turn, {counts['empty']} without a marker, {counts['no pose']} without a pose")
    print("images per marker: " + ", ".join(f"id {i}: {c}" for i, c in result["per_marker"].items())
          + (f"; never seen: {result['unseen']}" if result["unseen"] else ""))
    print(f"residuals of the batch fit: position {1e3 * batch['rms_position']:.1f} mm, "
          f"orientation {math.degrees(batch['rms_rotation']):.2f} deg per axis")
    s_min = batch["sigma_min"]
    print(f"batch Jacobian: smallest singular value {s_min:.3e}"
          + ("  (a direction of the parameters is not determined by this run)" if s_min < 1e-3 else ""))
    estimate = batch["estimate"]
    print("batch estimate:")
    print("  camera orientation x   " + np.array2string(estimate[0:3], precision=4))
    print("  camera position t_X [m]" + np.array2string(estimate[3:6], precision=4))
    for k, marker_id in enumerate(result["ids"]):
        print(f"  marker {marker_id} position [m]  "
              + np.array2string(estimate[6 + 3 * n + 3 * k:9 + 3 * n + 3 * k], precision=4))
    print(f"{'':28s} " + " ".join(f"{name:>10s}" for name in ERROR_FIELDS))
    print(f"{'batch radius':28s} " + " ".join(f"{v:10.2e}" for v in result["batch_radii"]))
    online = result["online"]
    if online is None:
        print("the recording has no online estimate of the markers of marker_ids")
    else:
        distance, radii = online["distance"][-1], online["radii"][-1]
        print(f"{'online radius, end of run':28s} " + " ".join(f"{v:10.2e}" for v in radii))
        print(f"{'online to batch, end of run':28s} " + " ".join(f"{v:10.2e}" for v in distance))
        inside = [bool(d <= r) if np.isfinite(r) else None for d, r in zip(distance, radii)]
        print(f"{'inside the online radius':28s} "
              + " ".join(f"{'-' if i is None else 'yes' if i else 'no':>10s}" for i in inside))
    if result["scan"]:
        best = min(result["scan"], key=lambda row: row[1])
        print("camera_delay [s]   position residual [mm]   orientation residual [deg]")
        for d, rms_p, rms_r in result["scan"]:
            marks = [name for name, here in (("lowest", d == best[0]),
                                             ("survey.yaml", abs(d - result["delay"]) < 1e-9)) if here]
            mark = "   <- " + ", ".join(marks) if marks else ""
            print(f"  {d:+.2f}             {1e3 * rms_p:8.1f}                 {math.degrees(rms_r):8.2f}{mark}")


def figure(result: dict):
    """The distance of the online estimate to the batch optimum over the run, in the
    layout of Fig. 1 of the paper."""
    from .figures import error_figure

    titles = ("Camera orientation $\\|\\hat{x} - x_{batch}\\|$",
              "Camera position $\\|\\hat{t}_X - t_{X,batch}\\|$ [m]",
              "Fiducial orientation $\\max_j \\|\\hat{m}_j - m_{j,batch}\\|$",
              "Fiducial position $\\max_j \\|\\hat{t}_j - t_{j,batch}\\|$ [m]")
    online = result["online"]
    return error_figure(online["time"], np.maximum(online["distance"], 1e-12), online["radii"],
                        titles, "Distance to the batch optimum", "99.73 % radius",
                        f"Online estimate against the batch optimum, {result['name']}")


def default_settings_file() -> str:
    return os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(camslab.__file__)),
                                         "..", "config", "survey.yaml"))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="camslab_standalone.fiducial.recorded",
                                 description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 directory of the run")
    ap.add_argument("--out", metavar="PNG", help="write the figure instead of showing it")
    args = ap.parse_args(argv)
    settings = car.load_settings(default_settings_file())
    try:
        result = analyse(read_bag(args.bag), settings)
    except ValueError as e:
        raise SystemExit(str(e))
    report(result)
    if result["online"] is None:
        return
    fig = figure(result)
    if args.out:
        fig.savefig(args.out, dpi=200)
        print(f"figure written to {args.out}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
