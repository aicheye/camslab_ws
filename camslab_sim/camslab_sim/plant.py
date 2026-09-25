"""Unicycle plant.

    dp/dt = v q
    dq/dt = w S q,    S = [[0, -1], [1, 0]]
"""

import math

import numpy as np

from .types import State


def derivatives(state: State, v: float, w: float):
    """Return (dp/dt, dq/dt) for the inputs v [m/s] and w [rad/s]."""

    S = np.array([[0.0, -1.0], [1.0, 0.0]])

    p_dot = v * state.q
    q_dot = w * np.dot(S, state.q)

    return (p_dot, q_dot)


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
