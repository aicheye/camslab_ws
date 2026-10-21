"""The adaptive path estimator, alone and inside simulate()."""

import math

import numpy as np
import pytest
from camslab.adaptive import COLUMNS, Estimator, Path, filter_matrices

from camslab_offline.common.unicycle import step
from camslab_offline.tracking.simulate import (PathObserver, estimate_fields, make_observer,
                                               simulate)
from camslab.trajectory import make_shape
from camslab.types import Params, State

PATH = Path(Params())
CIRCLE = [1.0, 2.0, 2.0]


def test_filter_matrices_match_slide_47():
    F, G, H = filter_matrices(2.0, 2)
    assert F == pytest.approx(np.array([[-2.0, 1.0], [0.0, -2.0]]))
    assert G == pytest.approx(np.array([0.0, 1.0]))
    assert H == pytest.approx(np.array([[1.0, 0.0], [-2.0, 1.0], [4.0, -4.0]]))


def test_xi_is_zero_on_the_circle():
    shape = make_shape(Params())
    for t in np.linspace(0.0, 20.0, 7):
        assert PATH.xi(shape.position(t)) == pytest.approx(0.0, abs=1e-5)


def test_xi_is_signed_like_the_circle():
    for p in ([2.0, 2.0], [3.5, 2.0], [2.0, 0.2]):
        expected = np.linalg.norm(np.array(p) - CIRCLE[1:]) - CIRCLE[0]
        assert PATH.xi(np.array(p)) == pytest.approx(expected, abs=1e-5)


def test_truth_is_the_circle_everywhere_on_the_circle_shape():
    for p in ([2.0, 2.0], [3.5, 2.0], [2.0, 0.2]):
        assert PATH.truth(np.array(p)) == pytest.approx(CIRCLE, abs=1e-6)


def test_truth_follows_the_closest_point_of_the_ellipse():
    """At the ends of the major axis the osculating circle has r = b^2 / a, and at the
    ends of the minor axis r = a^2 / b."""
    params = Params(shape="ellipse", theta0=0.0)
    path, (x0, y0, a, b) = Path(params), (params.x0, params.y0, params.a, params.b)
    assert path.truth(np.array([x0 + a + 0.1, y0])) == pytest.approx(
        [b * b / a, x0 + a - b * b / a, y0], abs=1e-3)
    assert path.truth(np.array([x0, y0 + b + 0.1])) == pytest.approx(
        [a * a / b, x0, y0 + b - a * a / b], abs=1e-3)


def test_truth_is_nan_on_a_straight_side():
    params = Params(shape="rectangle", theta0=0.0)
    assert np.all(np.isnan(Path(params).truth(np.array([params.x0 + params.a + 0.1,
                                                        params.y0]))))


@pytest.mark.parametrize("shape", ["gerono", "ellipse", "rectangle", "spline"])
def test_estimator_runs_on_every_shape(shape):
    params = Params(shape=shape)
    rows = simulate(State.from_pose(2.0, 1.0, 0.0), params, 0.02, 20.0, PathObserver(params))
    assert all(len(row) == 10 + len(COLUMNS) for row in rows)
    assert np.all(np.isfinite(np.array([row[:13] for row in rows])))


def test_estimator_does_not_change_the_tracking():
    initial = State.from_pose(2.5, 0.5, 0.0)
    observed = simulate(initial, Params(), 0.02, 40.0, PathObserver(Params()))
    plain = simulate(initial, Params(), 0.02, 40.0)
    assert np.array(observed)[:, :10] == pytest.approx(np.array(plain), abs=1e-12)


def test_circle_estimator_converges_with_excitation():
    est = Estimator(Params())
    state, dt = State.from_pose(3.3, 2.0, math.pi / 2), 0.02
    for k in range(int(180.0 / dt)):
        t = k * dt
        v, w = 0.3, 0.3 / 1.3 + 0.5 * (math.sin(0.7 * t) + math.sin(1.9 * t))
        est.update(state.p, PATH.xi(state.p), dt)
        state = step(state, v, w, dt)
    assert est.estimate() == pytest.approx(CIRCLE, abs=0.05)


def test_open_loop_rows_carry_the_estimate():
    observer = make_observer("path", Params())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(), 0.02, 60.0, observer)
    assert estimate_fields(observer) == list(COLUMNS)
    assert rows[-1][11:13] == pytest.approx(CIRCLE[1:], abs=0.05)
    assert rows[-1][13:16] == pytest.approx(CIRCLE, abs=1e-6)


def test_make_observer_rejects_unknown_kinds():
    assert make_observer("", Params()) is None
    with pytest.raises(ValueError):
        make_observer("kalman", Params())
