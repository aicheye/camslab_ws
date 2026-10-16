"""Unicycle plant."""

import math

import numpy as np

from camslab.types import Params, State


def step(state: State, v: float, w: float, dt: float) -> State:
    """Return the state dt seconds later, with v and w held constant over dt."""

    a = math.cos(w * dt)
    b = math.sin(w * dt)

    R = np.array(
        [
            [a, -b],
            [b, a],
        ]
    )
    q_next = np.dot(R, state.q)

    p_next: np.ndarray
    if abs(w) > 1e-9:
        M = np.array(
            [
                [b, a - 1],
                [-a + 1, b],
            ]
        )
        p_next = state.p + (v / w) * (np.dot(M, state.q))
    else:
        p_next = state.p + v * dt * state.q

    new_state = State(p_next, q_next)

    return new_state


def clamp(v: float, w: float, params: Params):
    """Clamp to |v| <= v_max and |w| <= min(w_max, kappa_max |v|), the turning limit of the
    Ackermann car. Same clamp as the ROS controller node."""
    v = min(max(v, -params.v_max), params.v_max)
    w_limit = min(params.w_max, params.kappa_max * abs(v))
    return v, min(max(w, -w_limit), w_limit)
