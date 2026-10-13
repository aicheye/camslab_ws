"""Closed-loop tracking. Fails with NotImplementedError until trajectory.py and
controller.py are written."""

import math

import numpy as np
import pytest

from camslab_sim.simulate import simulate
from camslab_sim.types import Params, State

# Initial poses away from the trajectory start (2, 1), in several headings.
INITIAL = [(2.0, 1.0, 0.0), (1.0, 0.5, 90.0), (3.0, 2.5, 180.0), (0.5, 2.0, -135.0)]


# Time by which ||p* - p|| < 0.05 m [s]. The default spline's p* moves at 0.17 to 0.29 m/s,
# slower than the other shapes, so a robot 2 m away and facing away takes longer to reach it.
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
