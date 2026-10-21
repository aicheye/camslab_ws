"""Fixed-step closed loop of the controller, the unicycle, and an optional adaptive estimator."""

import math

import numpy as np
from camslab import adaptive, fiducial

from camslab.controller import control
from ..common.unicycle import clamp, step
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State

ESTIMATORS = ("path", "fiducial")


class PathObserver:
    """camslab.adaptive: the circle that fits the path, from p and the distance xi."""

    plot = None

    def __init__(self, params: Params):
        self.path = adaptive.Path(params)
        self.estimator = adaptive.Estimator(params)
        self.fields = list(adaptive.COLUMNS)

    def observe(self, state: State, dt: float) -> list:
        self.estimator.update(state.p, self.path.xi(state.p), dt)
        return [*self.estimator.estimate(), *self.path.truth(state.p)]


class FiducialObserver:
    """camslab.fiducial: the camera extrinsic and the marker poses, from the robot pose
    and the markers the camera sees."""

    plot = list(fiducial.PLOT_FIELDS)

    def __init__(self, scene: fiducial.Scene | None = None, **estimator_kwargs):
        self.scene = fiducial.Scene() if scene is None else scene
        self.estimator = fiducial.Estimator(self.scene.n, self.scene.known_heights(),
                                            **estimator_kwargs)
        self.truth = self.scene.truth()
        self.fields = list(fiducial.columns(self.scene.n))

    def observe(self, state: State, dt: float) -> list:
        T_R = fiducial.robot_pose(state)
        self.estimator.update(T_R, self.scene.observe(T_R), dt)
        estimate = self.estimator.estimate()
        return [*estimate, *self.truth, *fiducial.errors(estimate, self.truth, self.scene.n)]


def make_observer(kind: str, params: Params):
    """An observer for `kind` ("path" or "fiducial"), or None for "" and None."""
    if not kind:
        return None
    if kind == "path":
        return PathObserver(params)
    if kind == "fiducial":
        return FiducialObserver()
    raise ValueError(f"estimator must be one of {ESTIMATORS}, got {kind!r}")


def estimate_fields(observer) -> list[str]:
    """Column names appended to each sample row when an observer runs."""
    return [] if observer is None else list(observer.fields)


def simulate(initial: State, params: Params, dt: float, t_max: float, observer=None):
    """Track params.shape from t = 0 to t_max.

    A reference past kappa_max or w_max is tracked anyway; clamp() keeps the commands
    inside the limits, so the robot lags p* there. Returns rows of [t, x, y, qx, qy, v, w,
    dist, xr, yr], the column order of camslab_webui.server.SAMPLE_FIELDS, with
    dist = ||p* - p|| and (xr, yr) = p*.

    An observer only observes: the controller tracks the true trajectory. Each row then
    also ends with observer.observe(state, dt), named by estimate_fields(), with None in
    place of NaN.
    """
    shape = make_shape(params)
    state = initial
    v = w = 0.0
    samples = []
    for k in range(int(round(t_max / dt)) + 1):
        t = k * dt
        ref = reference(shape, t)
        extra = []
        if observer is not None:
            extra = [float(x) if math.isfinite(x) else None for x in observer.observe(state, dt)]
        v, w = clamp(*control(state, ref, params), params)
        dist = float(np.linalg.norm(ref.p - state.p))
        samples.append([t, float(state.p[0]), float(state.p[1]),
                        float(state.q[0]), float(state.q[1]), float(v), float(w), dist,
                        float(ref.p[0]), float(ref.p[1]), *extra])
        state = step(state, v, w, dt)
    return samples
