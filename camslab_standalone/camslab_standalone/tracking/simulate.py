"""Fixed-step closed loop of the controller, the unicycle, and an optional adaptive estimator."""

import math

import numpy as np
from camslab import adaptive, fiducial

from camslab.controller import control
from ..common.unicycle import clamp, step
from ..fiducial.noise import NOISE_FIELDS, noisy_heights, noisy_measurements
from camslab.fiducial import Noise
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State

ESTIMATORS = ("path", "fiducial")


class PathObserver:
    """camslab.adaptive: the circle that fits the path, from p and the distance xi."""

    plots = adaptive.PLOTS

    def __init__(self, params: Params):
        self.path = adaptive.Path(params)
        self.estimator = adaptive.Estimator(params)
        self.fields = list(adaptive.COLUMNS)

    def observe(self, state: State, dt: float) -> list:
        self.estimator.update(state.p, self.path.xi(state.p), dt)
        return [*self.estimator.estimate(), *self.path.truth(state.p)]


class FiducialObserver:
    """camslab.fiducial: the camera extrinsic and the marker poses, from the robot pose
    and the markers the camera sees. With a Noise the estimator receives the robot pose,
    the observations and the known heights with that measurement noise, drawn from rng,
    and predicts its covariance for it; the controller still tracks the true state."""

    def __init__(self, scene: fiducial.Scene | None = None, noise: Noise | None = None,
                 rng=None, **estimator_kwargs):
        self.scene = fiducial.Scene() if scene is None else scene
        self.noise = noise
        self.rng = np.random.default_rng() if rng is None else rng
        heights = self.scene.known_heights()
        if noise is not None:
            heights = noisy_heights(heights, noise, self.rng)
        self.estimator = fiducial.Estimator(self.scene.n, heights, noise=noise,
                                            **estimator_kwargs)
        self.truth = self.scene.truth()
        # The estimator's columns, then the noise drawn at each sample.
        self.fields = list(fiducial.columns(self.scene.n)) + list(NOISE_FIELDS)
        self.plots = fiducial.plots(self.estimator) + [
            {"caption": "Measurement noise per sample",
             "fields": list(NOISE_FIELDS), "log": True}]

    def observe(self, state: State, dt: float) -> list:
        T_R = fiducial.robot_pose(state)
        observations = self.scene.observe(T_R)
        drawn = [math.nan] * len(NOISE_FIELDS)
        if self.noise is not None:
            T_R, observations, drawn = noisy_measurements(T_R, observations, self.noise,
                                                          self.rng)
        self.estimator.update(T_R, observations, dt)
        estimate = self.estimator.estimate()
        return [*estimate, *self.truth, *fiducial.errors(estimate, self.truth, self.scene.n),
                *self.estimator.excitation(), *self.estimator.radii(), *drawn]


def make_observer(kind: str, params: Params, rng=None, noise: Noise | None = None):
    """An observer for `kind` ("path" or "fiducial"), or None for "" and None. With an
    rng the fiducial observer gets a random camera and random markers from
    fiducial.random_scene(); without one, the fixed lab scene. noise is added to what
    the fiducial observer measures."""
    if not kind:
        return None
    if kind == "path":
        return PathObserver(params)
    if kind == "fiducial":
        return FiducialObserver(None if rng is None else fiducial.random_scene(rng),
                                noise=noise, rng=rng)
    raise ValueError(f"estimator must be one of {ESTIMATORS}, got {kind!r}")


def estimate_fields(observer) -> list[str]:
    """Column names appended to each sample row when an observer runs."""
    return [] if observer is None else list(observer.fields)


def simulate_steps(initial: State, params: Params, dt: float, t_max: float, observer=None):
    """Track params.shape from t = 0 to t_max, one row per step as it is computed.

    A reference past kappa_max or w_max is tracked anyway; clamp() keeps the commands
    inside the limits, so the robot lags p* there. Yields rows of [t, x, y, qx, qy, v, w,
    dist, xr, yr], the column order of camslab_webui.server.SAMPLE_FIELDS, with
    dist = ||p* - p|| and (xr, yr) = p*.

    An observer only observes: the controller tracks the true trajectory. Each row then
    also ends with observer.observe(state, dt), named by estimate_fields(), with None in
    place of NaN.
    """
    shape = make_shape(params)
    state = initial
    v = w = 0.0
    for k in range(int(round(t_max / dt)) + 1):
        t = k * dt
        ref = reference(shape, t)
        extra = []
        if observer is not None:
            extra = [float(x) if math.isfinite(x) else None for x in observer.observe(state, dt)]
        v, w = clamp(*control(state, ref, params), params)
        dist = float(np.linalg.norm(ref.p - state.p))
        yield [t, float(state.p[0]), float(state.p[1]),
               float(state.q[0]), float(state.q[1]), float(v), float(w), dist,
               float(ref.p[0]), float(ref.p[1]), *extra]
        state = step(state, v, w, dt)


def simulate(initial: State, params: Params, dt: float, t_max: float, observer=None):
    """All rows of simulate_steps() as a list."""
    return list(simulate_steps(initial, params, dt, t_max, observer))
