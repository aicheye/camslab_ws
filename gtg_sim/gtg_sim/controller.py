"""Go-to-goal control law.

With a = p* - p:

    v = v*              if ||a|| > D
        Kv ||a||        otherwise
    w = Kw atan2(sin(e), cos(e))
    e = (bearing of a) - (heading of q)
"""

import math

import numpy as np

from .types import Params, State


def control(state: State, goal: np.ndarray, params: Params):
    """Return the command (v [m/s], w [rad/s]) for the goal p* = goal."""

    a = goal - state.p
    d = np.linalg.norm(a)

    v: float
    if d > params.d_switch:
        v = params.v_star
    else:
        v = params.k_v * d

    e = math.atan2(a[1], a[0]) - math.atan2(state.q[1], state.q[0])
    w = params.k_w * math.atan2(math.sin(e), math.cos(e))

    return (v, w)
