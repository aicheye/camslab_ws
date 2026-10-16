"""Fixed-step closed loop of the controller, the unicycle, and an optional adaptive estimator."""

import numpy as np
from camslab.adaptive.base import Estimator, Measurement
from camslab.adaptive.paths import distance_to_path

from camslab.controller import control
from ..common.unicycle import clamp, step
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State


def estimate_fields(estimator: Estimator | None) -> list[str]:
    """Column names appended to each sample row when an estimator runs: "<field>_hat"."""
    return [] if estimator is None else [f + "_hat" for f in estimator.fields]


def simulate(initial: State, params: Params, dt: float, t_max: float,
             estimator: Estimator | None = None, mode: str = "open"):
    """Track params.shape from t = 0 to t_max.

    A reference past kappa_max or w_max is tracked anyway; clamp() keeps the commands
    inside the limits, so the robot lags p* there. Returns rows of [t, x, y, qx, qy, v, w,
    dist, xr, yr], the column order of camslab_webui.server.SAMPLE_FIELDS, with
    dist = ||p* - p|| and (xr, yr) = p*.

    With an estimator, each row also ends with theta_hat (estimate_fields()), and mode
    picks the reference: "open" tracks the true trajectory while the estimator observes,
    "closed" tracks estimator.reference(), built from theta_hat.
    """
    if mode not in ("open", "closed"):
        raise ValueError(f"mode must be open or closed, got {mode!r}")
    shape = make_shape(params)
    path = params.to_dict()
    state = initial
    v = w = 0.0
    samples = []
    for k in range(int(round(t_max / dt)) + 1):
        t = k * dt
        ref = reference(shape, t)
        theta_hat = []
        if estimator is not None:
            m = Measurement(t=t, p=state.p, q=state.q, v=v, w=w,
                            xi=distance_to_path(path, state.p))
            estimator.update(m, dt)
            theta_hat = [float(x) for x in estimator.estimate()]
            if mode == "closed":
                r = estimator.reference(m, dt)
                if r is None:
                    raise ValueError(f"estimator {estimator.name!r} has no closed-loop mode")
                ref = r
        v, w = clamp(*control(state, ref, params), params)
        dist = float(np.linalg.norm(ref.p - state.p))
        samples.append([t, float(state.p[0]), float(state.p[1]),
                        float(state.q[0]), float(state.q[1]), float(v), float(w), dist,
                        float(ref.p[0]), float(ref.p[1]), *theta_hat])
        state = step(state, v, w, dt)
    return samples
