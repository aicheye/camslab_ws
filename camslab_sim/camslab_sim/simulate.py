"""Fixed-step closed loop: sample the reference and the controller, hold the command over
dt, step the plant."""

import numpy as np

from .controller import control
from .plant import step
from .trajectory import check_limits, make_shape, reference
from .types import Params, State


def simulate(initial: State, params: Params, dt: float, t_max: float):
    """Track params.shape from t = 0 to t_max.

    Raises ValueError from check_limits() if the reference breaks kappa_max or w_max.
    Returns rows of [t, x, y, qx, qy, v, w, dist, xr, yr], the column order of
    camslab_webui.server.SAMPLE_FIELDS, with dist = ||p* - p|| and (xr, yr) = p*.
    """
    shape = make_shape(params)
    check_limits(shape, params, t_max)
    state = initial
    samples = []
    for k in range(int(round(t_max / dt)) + 1):
        t = k * dt
        ref = reference(shape, t)
        v, w = control(state, ref, params)
        dist = float(np.linalg.norm(ref.p - state.p))
        samples.append([t, float(state.p[0]), float(state.p[1]),
                        float(state.q[0]), float(state.q[1]), float(v), float(w), dist,
                        float(ref.p[0]), float(ref.p[1])])
        state = step(state, v, w, dt)
    return samples
