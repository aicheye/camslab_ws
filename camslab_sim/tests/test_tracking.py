"""Closed-loop tracking. Fails with NotImplementedError until trajectory.py and
controller.py are written."""

import math

import numpy as np
import pytest

from camslab_sim.simulate import simulate
from camslab_sim.types import Params, State

# Initial poses away from the trajectory start (2, 1), in several headings.
INITIAL = [(2.0, 1.0, 0.0), (1.0, 0.5, 90.0), (3.0, 2.5, 180.0), (0.5, 2.0, -135.0)]


@pytest.mark.parametrize("shape", ["circle", "gerono"])
@pytest.mark.parametrize("initial", INITIAL)
def test_converges_to_reference(shape, initial):
    x, y, theta_deg = initial
    samples = simulate(State.from_pose(x, y, math.radians(theta_deg)), Params(shape=shape),
                       dt=0.01, t_max=40.0)
    dist = np.array([row[7] for row in samples])
    t = np.array([row[0] for row in samples])
    assert dist[t >= 30.0].max() < 0.05
