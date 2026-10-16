"""Fixed-step closed loop of the controller, the unicycle, and an optional adaptive estimator."""

import math

import numpy as np
from camslab.adaptive import COLUMNS, Estimator, Path

from camslab.controller import control
from ..common.unicycle import clamp, step
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State


def estimate_fields(estimator: Estimator | None) -> list[str]:
    """Column names appended to each sample row when an estimator runs: theta_hat, then
    the true circle at the closest point of the path."""
    return [] if estimator is None else list(COLUMNS)


def simulate(initial: State, params: Params, dt: float, t_max: float,
             estimator: Estimator | None = None):
    """Track params.shape from t = 0 to t_max.

    A reference past kappa_max or w_max is tracked anyway; clamp() keeps the commands
    inside the limits, so the robot lags p* there. Returns rows of [t, x, y, qx, qy, v, w,
    dist, xr, yr], the column order of camslab_webui.server.SAMPLE_FIELDS, with
    dist = ||p* - p|| and (xr, yr) = p*.

    With an estimator, which only observes, each row also ends with theta_hat and the true
    circle (estimate_fields(), None where the path is straight).
    """
    shape = make_shape(params)
    path = Path(params) if estimator is not None else None
    state = initial
    v = w = 0.0
    samples = []
    for k in range(int(round(t_max / dt)) + 1):
        t = k * dt
        ref = reference(shape, t)
        theta_hat = []
        if estimator is not None:
            estimator.update(state.p, path.xi(state.p), dt)
            theta_hat = [float(x) if math.isfinite(x) else None
                         for x in (*estimator.estimate(), *path.truth(state.p))]
        v, w = clamp(*control(state, ref, params), params)
        dist = float(np.linalg.norm(ref.p - state.p))
        samples.append([t, float(state.p[0]), float(state.p[1]),
                        float(state.q[0]), float(state.q[1]), float(v), float(w), dist,
                        float(ref.p[0]), float(ref.p[1]), *theta_hat])
        state = step(state, v, w, dt)
    return samples
