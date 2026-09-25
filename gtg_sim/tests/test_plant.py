"""Checks of step() in plant.py against known solutions."""

import math

import numpy as np
import pytest

from gtg_sim.plant import step
from gtg_sim.types import State


def test_heading_stays_unit_length():
    state = State.from_pose(0.0, 0.0, 0.0)
    for _ in range(1000):
        state = step(state, 0.5, 1.0, 0.01)
    assert np.linalg.norm(state.q) == pytest.approx(1.0, abs=1e-6)


def test_straight_line():
    state = State.from_pose(1.0, -2.0, math.pi / 2)
    for _ in range(100):
        state = step(state, 0.5, 0.0, 0.01)
    assert state.p == pytest.approx([1.0, -1.5], abs=1e-6)


def test_constant_turn_matches_circle():
    # v = 1, w = 1 from the origin: a unit circle centred on (0, 1).
    state = State.from_pose(0.0, 0.0, 0.0)
    for _ in range(157):
        state = step(state, 1.0, 1.0, 0.01)
    assert state.p == pytest.approx([math.sin(1.57), 1 - math.cos(1.57)], abs=1e-3)
