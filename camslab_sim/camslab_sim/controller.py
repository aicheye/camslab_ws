"""Trajectory tracking control law.

With the reference p*, q*, v*, w* from trajectory.reference() and e = p* - p:

    v = < v* q* + k_par e, q >
    w = w* + < k_perp v* e + k_q q*, S q >

<a, b> is the dot product a . b, and S = [[0, -1], [1, 0]] rotates a vector by +90 deg,
the same S as in plant.py. k_par, k_perp, k_q > 0.
"""

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
