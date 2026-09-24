"""Fixed-step closed loop: sample the controller, hold the command over dt, step the plant."""

import numpy as np

from .controller import control
from .plant import step
from .types import Params, State


def simulate(initial: State, goal, params: Params, dt: float, t_max: float):
    """Run until ||p* - p|| < epsilon or t >= t_max.

    Returns rows of [t, x, y, qx, qy, v, w, dist], the column order of
    gtg_webui.server.SAMPLE_FIELDS.
    """
    goal = np.asarray(goal, dtype=float)
    state = initial
    samples = []
    for k in range(int(round(t_max / dt)) + 1):
        v, w = control(state, goal, params)
        dist = float(np.linalg.norm(goal - state.p))
        samples.append([k * dt, float(state.p[0]), float(state.p[1]),
                        float(state.q[0]), float(state.q[1]), float(v), float(w), dist])
        if dist < params.epsilon:
            break
        state = step(state, v, w, dt)
    return samples
