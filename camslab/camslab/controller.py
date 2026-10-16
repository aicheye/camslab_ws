"""Trajectory tracking control law."""

import numpy as np

from .trajectory import Reference
from .types import Params, State


def control(state: State, ref: Reference, params: Params):
    """Return the command (v [m/s], w [rad/s]) that tracks ref."""

    e = ref.p - state.p
    S = np.array([[0, -1], [1, 0]])

    v = (ref.v * ref.q + params.k_par * e) @ state.q
    w = ref.w + (params.k_perp * ref.v * e + params.k_q * ref.q) @ (S @ state.q)

    return (v, w)
