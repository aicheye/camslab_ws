"""Adaptive estimators, alone and inside simulate()."""

import math

import numpy as np
import pytest
from camslab.adaptive.base import Estimator, Measurement
from camslab.adaptive.circle import CircleEstimator
from camslab.adaptive.filters import filter_matrices
from camslab.adaptive.paths import distance_to_path
from camslab.adaptive.registry import ESTIMATORS, make_estimator

from camslab_offline.common.unicycle import step
from camslab_offline.tracking.simulate import estimate_fields, simulate
from camslab.trajectory import make_shape, reference
from camslab.types import Params, State

PATH = Params().to_dict()


def test_registry_entries_are_complete():
    for name, cls in ESTIMATORS.items():
        assert cls.name == name
        assert len(cls.fields) > 0 and len(cls.shapes) > 0
    with pytest.raises(ValueError):
        make_estimator("no such estimator", PATH)


def test_circle_rejects_other_shapes():
    with pytest.raises(ValueError):
        make_estimator("circle", {**PATH, "shape": "gerono"})


def test_filter_matrices_match_slide_47():
    F, G, H = filter_matrices(2.0, 2)
    assert F == pytest.approx(np.array([[-2.0, 1.0], [0.0, -2.0]]))
    assert G == pytest.approx(np.array([0.0, 1.0]))
    assert H == pytest.approx(np.array([[1.0, 0.0], [-2.0, 1.0], [4.0, -4.0]]))


def test_distance_is_zero_on_the_circle():
    shape = make_shape(Params())
    for t in np.linspace(0.0, 20.0, 7):
        assert distance_to_path(PATH, shape.position(t)) == pytest.approx(0.0, abs=1e-9)


def test_circle_estimator_converges_with_excitation():
    est = CircleEstimator(PATH)
    state, dt = State.from_pose(3.3, 2.0, math.pi / 2), 0.02
    for k in range(int(180.0 / dt)):
        t = k * dt
        v, w = 0.3, 0.3 / 1.3 + 0.5 * (math.sin(0.7 * t) + math.sin(1.9 * t))
        est.update(Measurement(t, state.p, state.q, v, w, distance_to_path(PATH, state.p)), dt)
        state = step(state, v, w, dt)
    assert est.estimate() == pytest.approx(est.truth(), abs=0.05)


def test_open_loop_rows_carry_the_estimate():
    est = make_estimator("circle", PATH)
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(), 0.02, 60.0, est, "open")
    assert len(estimate_fields(est)) == 3
    assert all(len(row) == 10 + 3 for row in rows)
    assert rows[-1][11:13] == pytest.approx(est.truth()[1:3], abs=0.05)


class TrueCircle(Estimator):
    """Closed-loop plumbing check: theta_hat is the true circle and reference() is the
    true trajectory, so the closed loop must track like the open loop."""
    name, fields, shapes = "true", ("r",), ("circle",)

    def __init__(self, path):
        super().__init__(path)
        self.shape = make_shape(Params(**{k: v for k, v in path.items()}))

    def update(self, m, dt):
        pass

    def estimate(self):
        return np.array([1.0])

    def reference(self, m, dt):
        return reference(self.shape, m.t)


def test_closed_loop_tracks_the_estimator_reference():
    initial = State.from_pose(2.5, 0.5, 0.0)
    closed = simulate(initial, Params(), 0.02, 40.0, TrueCircle(PATH), "closed")
    opened = simulate(initial, Params(), 0.02, 40.0)
    assert np.array(closed)[:, :10] == pytest.approx(np.array(opened), abs=1e-9)


def test_closed_loop_runs_with_the_circle_estimator():
    rows = simulate(State.from_pose(3.0, 2.5, math.pi), Params(), 0.02, 30.0,
                    make_estimator("circle", PATH), "closed")
    assert np.all(np.isfinite(np.array(rows)))
