"""Closed-loop tracking."""

import math

import numpy as np
import pytest

from camslab_standalone.tracking.simulate import simulate
from camslab.types import Params, State

INITIAL = [(2.0, 1.0, 0.0), (1.0, 0.5, 90.0), (3.0, 2.5, 180.0), (0.5, 2.0, -135.0)]


SETTLE = {"circle": 30.0, "gerono": 30.0, "ellipse": 30.0, "spline": 50.0}


@pytest.mark.parametrize("shape", SETTLE)
@pytest.mark.parametrize("initial", INITIAL)
def test_converges_to_reference(shape, initial):
    x, y, theta_deg = initial
    settle = SETTLE[shape]
    samples = simulate(State.from_pose(x, y, math.radians(theta_deg)), Params(shape=shape),
                       dt=0.01, t_max=settle + 10.0)
    dist = np.array([row[7] for row in samples])
    t = np.array([row[0] for row in samples])
    assert dist[t >= settle].max() < 0.05
