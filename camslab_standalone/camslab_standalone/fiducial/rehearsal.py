"""Rehearsal of a hardware survey: the simulator with the limits of the real camera, on
the path and the planned stand positions of camslab/config/survey.yaml.

    uv run python -m camslab_standalone.fiducial.rehearsal [--out PNG]

The simulated camera of camslab.fiducial sees every marker at any distance. The camera
node on the car publishes a marker only when it is whole in the image, faces the camera
and its shortest side is at least `min_marker_pixels` long, and the estimator skips
images taken above `max_yaw_rate` (docs/hardware-survey.md, "Which images are used").
With these limits a marker is used from a few stretches of the path only, and whether
the estimator gets pure-translation pairs and rotation pairs of every marker depends on
where the stands are. The rehearsal applies the same rules to the simulated camera,
with the calibration named by `camera`, adds the measurement noise of ASSUMED_NOISE, and
reports

- how many images of each marker are used and how many pairs the estimator stores,
- whether the adaptive laws switch on, and the errors at the end of every lap,
- a stand whose orientation is near the singularity of the Cayley coordinates,

and draws the floor plan: the path, the stands, and where each marker is in use.

It does not model lens distortion, markers hiding each other, the ambiguity check of
the camera node (it drops about one accepted marker in ten on rendered images), or the
car's tracking error on the floor.
"""

import argparse
import math
import os
from dataclasses import dataclass, fields as dataclass_fields

import numpy as np

import camslab
from camslab import car, fiducial
from camslab.aruco import intrinsics_of_file, marker_corners, nominal_intrinsics
from camslab.fiducial import Marker, Noise, Scene, rotation_angle, rpy_rotation
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State

from ..tracking.simulate import simulate
from .offline import RecordingObserver

# The printed stands of hardware/marker_mounts: how far the marker's normal is tilted up
# [deg] and how far the marker is turned about its normal [deg].
STANDS = {"upright": (0.0, 0.0), "overhang": (-30.0, 0.0), "diamond": (0.0, 45.0),
          "faceup": (20.0, 30.0)}
# The marker frame of cv2.aruco for a marker that faces along the world x axis: its x
# axis to the right (world y), its y axis up, its z axis out of the marker.
FACING_X = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
# The R2's camera: 0.12 m above the floor (hardware/marker_mounts/README.md). The
# position ahead of the rear axle and the small misalignment are assumptions; the survey
# estimates both.
CAMERA_CAYLEY = (0.01, 0.02, -0.03)
CAMERA_POSITION = (0.20, 0.0, 0.12)
# What the camera node's poses scatter by on rendered images, the pooled values of
# docs/hardware-survey.md. The real camera's noise is measured by the batch check of a
# recorded run (camslab_standalone.fiducial.recorded).
ASSUMED_NOISE = Noise(marker_position=0.013, marker_rotation=math.radians(0.7))
MAX_OBLIQUE = math.radians(65.0)    # the detection was tested up to here on rendered images
CAYLEY_LIMIT = math.radians(150.0)  # a marker orientation beyond this is near the singularity
LAPS = 4


def rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def stand_rotation(stand: str, facing: float) -> np.ndarray:
    """World orientation of the marker on `stand` when the stand faces along `facing`
    [rad], the direction of the marker's normal on the floor."""
    up, roll = STANDS[stand]
    return (rotation_z(facing) @ FACING_X @ rotation_x(-math.radians(up))
            @ rotation_z(math.radians(roll)))


def stand_marker(stand: str, x: float, y: float, facing: float, height: float,
                 known_height: bool) -> Marker:
    """The fiducial.Marker of a stand at (x, y) [m] facing along `facing` [rad], with the
    marker centre `height` above the floor."""
    R = stand_rotation(stand, facing)
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    rpy = (math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0]))
    if np.abs(rpy_rotation(*rpy) - R).max() > 1e-9:
        raise ValueError(f"stand {stand} facing {math.degrees(facing):.0f} deg has no roll, "
                         "pitch, yaw")
    return Marker((x, y, height), rpy, known_height)


@dataclass
class CameraScene(Scene):
    """A Scene whose sensor accepts a marker as the camera node on the car does: every
    corner in the image of the pinhole camera K, the marker facing the camera within
    MAX_OBLIQUE of its normal, and its shortest side in the image at least `min_side`
    pixels. `blind` drops every marker, for an image taken in a fast turn."""

    K: np.ndarray = None
    width: int = 640
    height: int = 480
    marker_size: float = 0.14
    min_side: float = 30.0
    blind: bool = False

    def accepted(self, T: np.ndarray) -> bool:
        """Whether the camera node publishes a marker at C_T_M = T."""
        corners = T[:3, :3] @ marker_corners(self.marker_size).T + T[:3, 3:4]
        if (corners[0] <= 0.05).any():
            return False
        u = self.K[0, 2] - self.K[0, 0] * corners[1] / corners[0]
        v = self.K[1, 2] - self.K[1, 1] * corners[2] / corners[0]
        if (u < 0).any() or (u > self.width).any() or (v < 0).any() or (v > self.height).any():
            return False
        to_camera = -T[:3, :3].T @ T[:3, 3]
        distance = float(np.linalg.norm(to_camera))
        if to_camera[2] <= 0.0 or math.acos(to_camera[2] / distance) > MAX_OBLIQUE:
            return False
        quad = np.stack([u, v], axis=1)
        return float(np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1).min()) >= self.min_side

    def observe(self, T_R: np.ndarray) -> dict:
        if self.blind:
            return {}
        return {j: T for j, T in super().observe(T_R).items() if self.accepted(T)}


class Rehearsal(RecordingObserver):
    """The fiducial observer on a CameraScene that also skips the images taken while the
    robot turns faster than max_yaw_rate, as the estimator node does."""

    def __init__(self, scene: CameraScene, max_yaw_rate: float, **kwargs):
        super().__init__(scene, **kwargs)
        self.max_yaw_rate = max_yaw_rate
        self.last_q = None
        self.turning = 0

    def observe(self, state: State, dt: float) -> list:
        rate = 0.0
        if self.last_q is not None and dt > 0.0:
            cross = self.last_q[0] * state.q[1] - self.last_q[1] * state.q[0]
            rate = abs(math.atan2(cross, float(self.last_q @ state.q))) / dt
        self.last_q = state.q.copy()
        self.scene.blind = self.max_yaw_rate > 0.0 and rate > self.max_yaw_rate
        self.turning += self.scene.blind
        return super().observe(state, dt)


def config_dir() -> str:
    return os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(camslab.__file__)),
                                         "..", "config"))


def camera_intrinsics(settings: dict) -> np.ndarray:
    """K of the calibration `camera`, or of nominal_fov when there is none."""
    camera = str(settings.get("camera") or "")
    width, height = int(settings["image_width"]), int(settings["image_height"])
    if not camera:
        return nominal_intrinsics(width, height, float(settings.get("nominal_fov") or 60.0))
    path = (os.path.expanduser(camera) if os.sep in camera or camera.endswith(".yaml")
            else os.path.join(config_dir(), "cameras", f"{camera}.yaml"))
    K, _, _ = intrinsics_of_file(path)
    return K


def survey_params(settings: dict) -> Params:
    """The controller's Params with the `controller` block of the settings."""
    names = {f.name for f in dataclass_fields(Params)}
    values = {k: v for k, v in (settings.get("controller") or {}).items() if k in names}
    if "control_points" in values:
        values["control_points"] = tuple(values["control_points"])
    # Without a direction, the one the fit takes for an area along the x axis (fit_bowtie).
    values.setdefault("theta0", math.pi)
    return Params(**values)


BEYOND = 0.6    # how far past a corner a stand is put [m]


def corner_stands(params: Params, beyond: float = BEYOND) -> list:
    """The stands as the guide places them: one past each corner of the bowtie, on the
    line the car arrives on, facing the car. In the frame of the path, in the order of
    the marker ids: past the ends of the two diagonals, then of the two short sides,
    so that markers 0 and 2 alone stand at opposite corners."""
    a, b = params.a, params.b
    diagonal = math.atan2(b, a)
    u = beyond * math.cos(diagonal), beyond * math.sin(diagonal)
    return [{"stand": "upright", "x": -a - u[0], "y": b + u[1], "facing": -math.degrees(diagonal)},
            {"stand": "overhang", "x": a + u[0], "y": b + u[1], "facing": math.degrees(diagonal) - 180.0},
            {"stand": "diamond", "x": a, "y": -b - beyond, "facing": 90.0},
            {"stand": "faceup", "x": -a, "y": -b - beyond, "facing": 90.0}]


def placed_stands(settings: dict) -> list:
    """The stands in motion capture coordinates, as {stand, x, y, facing}: those of
    corner_stands(), or of a `stands` setting, which gives each position [m] and facing
    [deg] in the frame of the path. Its origin is the path's centre (x0, y0) and its x
    axis points along theta0, so the layout moves and turns with the path."""
    ids = settings["marker_ids"]
    corners = corner_stands(survey_params(settings))
    stands = settings.get("stands") or [corners[int(i)] for i in ids if 0 <= int(i) < len(corners)]
    if len(stands) != len(ids):
        raise ValueError(f"`stands` has {len(stands)} entries, marker_ids has {len(ids)}; the "
                         "rehearsal needs one planned stand per marker")
    params = survey_params(settings)
    c, s = math.cos(params.theta0), math.sin(params.theta0)
    return [{"stand": str(stand["stand"]),
             "x": params.x0 + c * float(stand["x"]) - s * float(stand["y"]),
             "y": params.y0 + s * float(stand["x"]) + c * float(stand["y"]),
             "facing": math.degrees(params.theta0) + float(stand["facing"])}
            for stand in stands]


def survey_scene(settings: dict) -> CameraScene:
    """The planned stands of the settings in front of the car's camera."""
    markers = [stand_marker(s["stand"], s["x"], s["y"], math.radians(s["facing"]), float(h), bool(k))
               for s, h, k in zip(placed_stands(settings), settings["marker_height"],
                                  settings["marker_height_known"])]
    return CameraScene(camera_cayley=CAMERA_CAYLEY, camera_position=CAMERA_POSITION,
                       markers=markers, K=camera_intrinsics(settings),
                       width=int(settings["image_width"]), height=int(settings["image_height"]),
                       marker_size=float(settings["marker_size"]),
                       min_side=float(settings.get("min_marker_pixels", 30.0)))


def rehearse(settings: dict, laps: int = LAPS, dt: float = 0.05, seed: int = 0,
             noise: Noise | None = ASSUMED_NOISE) -> dict:
    """`laps` laps of the survey path in front of the planned stands. Returns what
    report() prints and figure() draws."""
    params = survey_params(settings)
    scene = survey_scene(settings)
    observer = Rehearsal(scene, float(settings.get("max_yaw_rate", 0.0)), noise=noise,
                         rng=np.random.default_rng(seed),
                         straight_tol=float(settings.get("straight_tol", car.DEFAULTS["straight_tol"])))
    start = reference(make_shape(params), 0.0)
    initial = State(p=np.array(start.p, float), q=np.array(start.q, float))
    rows = np.array(simulate(initial, params, dt, laps * params.period, observer), dtype=float)
    column = {name: 10 + k for k, name in enumerate(observer.fields)}
    used = np.array([[j in found for j in range(scene.n)] for _, found in observer.data])
    heading = np.unwrap(np.arctan2(rows[:, 4], rows[:, 3]))
    per_lap = int(round(params.period / dt))
    estimator = observer.estimator
    return {
        "params": params, "scene": scene, "ids": [int(i) for i in settings["marker_ids"]],
        "stands": placed_stands(settings), "time": rows[:, 0], "position": rows[:, 1:3],
        "used": used, "turning": observer.turning,
        "speed": float(np.abs(rows[:, 5]).max()),
        "yaw_rate": float(np.abs(np.gradient(heading, rows[:, 0])).max()),
        "tracking": float(rows[per_lap // 4:, 7].max()),
        "angles": [rotation_angle(m.pose()[:3, :3]) for m in scene.markers],
        "pairs": (len(estimator.H_x), len(estimator.H_t), len(estimator.H_h)),
        "ready": estimator.ready(),
        "laps": [(lap, rows[min(lap * per_lap, len(rows) - 1), [column[n] for n in fiducial.ERROR_FIELDS]])
                 for lap in range(1, laps + 1)],
        "radii": estimator.radii(),
        "range": float(scene.K[0, 0] * scene.marker_size / scene.min_side),
    }


def report(result: dict) -> None:
    params = result["params"]
    print(f"{params.shape} a = {params.a:g} m, b = {params.b:g} m, lap {params.period:g} s: "
          f"speed up to {result['speed']:.2f} m/s, turning up to {result['yaw_rate']:.2f} rad/s, "
          f"tracking error up to {1e3 * result['tracking']:.0f} mm")
    print(f"a marker is accepted up to {result['range']:.2f} m head-on; {result['turning']} of "
          f"{len(result['time'])} images skipped in a turn")
    print("marker  stand     x [m]  y [m]  facing [deg]  images used  rotation angle [deg]")
    for j, (marker_id, stand) in enumerate(zip(result["ids"], result["stands"])):
        angle = result["angles"][j]
        print(f"  {marker_id:<5d} {stand['stand']:<9s} {float(stand['x']):5.2f}  {float(stand['y']):5.2f}  "
              f"{float(stand['facing']):8.0f}      {int(result['used'][:, j].sum()):6d}       "
              f"{math.degrees(angle):6.0f}"
              + ("   near the Cayley singularity at 180: turn the stand" if angle > CAYLEY_LIMIT else ""))
    n_x, n_t, n_h = result["pairs"]
    ready_x, ready_t = result["ready"]
    print(f"stored: {n_x} orientation regressions, {n_t} rotation pairs, {n_h} heights; "
          f"orientation law {'on' if ready_x else 'OFF'}, translation law {'on' if ready_t else 'OFF'}")
    print("errors at the end of each lap:")
    print("  lap   camera orientation   camera position [mm]   marker orientation   marker position [mm]")
    for lap, e in result["laps"]:
        print(f"  {lap:3d}   {e[0]:18.4f}   {1e3 * e[1]:20.1f}   {e[2]:18.4f}   {1e3 * e[3]:20.1f}")
    r = result["radii"]
    print(f"  99.73 % radius {r[0]:9.4f}   {1e3 * r[1]:20.1f}   {r[2]:18.4f}   {1e3 * r[3]:20.1f}")
    never = [result["ids"][j] for j in range(len(result["ids"])) if not result["used"][:, j].any()]
    if never:
        print(f"markers never accepted: {never}; move their stands toward the path or turn them")


def figure(result: dict):
    """The floor plan: the path, the stands with the direction they face, and the
    stretches of the path on which each marker is in use."""
    import matplotlib.pyplot as plt

    params, position, used = result["params"], result["position"], result["used"]
    lap = int(round(params.period / (result["time"][1] - result["time"][0])))
    last = slice(len(position) - lap - 1, len(position))        # the last lap
    fig, ax = plt.subplots(figsize=(8.0, 5.2), constrained_layout=True)
    ax.plot(position[last, 0], position[last, 1], color="0.6", lw=1.0, label="Path")
    start = reference(make_shape(params), 0.0)
    ax.annotate("", xy=tuple(start.p + 0.45 * start.q), xytext=tuple(start.p),
                arrowprops={"arrowstyle": "->", "color": "0.3", "lw": 1.5})
    ax.annotate("start", xy=tuple(start.p), xytext=(0, -14), textcoords="offset points",
                ha="center", fontsize=8, color="0.3")
    for j, (marker_id, stand) in enumerate(zip(result["ids"], result["stands"])):
        color = f"C{j}"
        x, y, facing = float(stand["x"]), float(stand["y"]), math.radians(float(stand["facing"]))
        stretch = np.where(used[last, j][:, None], position[last], np.nan)
        ax.plot(stretch[:, 0], stretch[:, 1], color=color, lw=3.0, alpha=0.8,
                label=f"Marker {marker_id} in use")
        plate = 0.1 * np.array([-math.sin(facing), math.cos(facing)])
        ax.plot([x - plate[0], x + plate[0]], [y - plate[1], y + plate[1]], color=color, lw=4.0)
        ax.annotate("", xy=(x + 0.35 * math.cos(facing), y + 0.35 * math.sin(facing)), xytext=(x, y),
                    arrowprops={"arrowstyle": "->", "color": color, "lw": 1.5})
        ax.annotate(f"{marker_id} {stand['stand']}\n({x:.2f}, {y:.2f}), {float(stand['facing']):.0f}°",
                    xy=(x, y), xytext=(0, -26 if y < params.y0 else 12), textcoords="offset points",
                    ha="center", fontsize=8, color=color)
    ax.set_aspect("equal")
    ax.margins(0.12)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Stand layout for the {params.shape} of {2 * params.a:g} m by {2 * params.b:g} m",
                 fontsize=11)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=5)
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(prog="camslab_standalone.fiducial.rehearsal", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", metavar="PNG", help="write the floor plan instead of showing it")
    args = ap.parse_args(argv)
    settings = car.load_settings(os.path.join(config_dir(), "survey.yaml"))
    try:
        result = rehearse(settings)
    except ValueError as e:
        raise SystemExit(str(e))
    report(result)
    fig = figure(result)
    if args.out:
        fig.savefig(args.out, dpi=200)
        print(f"floor plan written to {args.out}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


if __name__ == "__main__":
    main()
