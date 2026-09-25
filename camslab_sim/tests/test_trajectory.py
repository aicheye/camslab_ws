"""Checks of trajectory.py. They fail with NotImplementedError until it is written.

The derivative tests compare velocity() and acceleration() with central differences
(f(t + h) - f(t - h)) / 2h of position() and velocity(), whose error is about h^2.
"""

import math

import numpy as np
import pytest

from camslab_sim.trajectory import Circle, Gerono, check_limits, reference
from camslab_sim.types import Params

H = 1e-5
TIMES = [0.0, 0.7, 3.1, 11.0, 26.5]

SHAPES = {
    "circle": lambda: Circle(Params(x0=1.0, y0=-0.5, theta0=0.4, circle_speed=0.3, kappa=1.2)),
    "circle_cw": lambda: Circle(Params(theta0=2.0, circle_speed=0.5, kappa=-0.8)),
    "straight": lambda: Circle(Params(theta0=-1.0, circle_speed=0.4, kappa=0.0)),
    "gerono": lambda: Gerono(Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, period=30.0)),
}


def central_difference(f, t):
    return (f(t + H) - f(t - H)) / (2 * H)


@pytest.mark.parametrize("name", SHAPES)
@pytest.mark.parametrize("t", TIMES)
def test_velocity_is_derivative_of_position(name, t):
    shape = SHAPES[name]()
    assert shape.velocity(t) == pytest.approx(central_difference(shape.position, t), abs=1e-6)


@pytest.mark.parametrize("name", SHAPES)
@pytest.mark.parametrize("t", TIMES)
def test_acceleration_is_derivative_of_velocity(name, t):
    shape = SHAPES[name]()
    assert shape.acceleration(t) == pytest.approx(
        central_difference(shape.velocity, t), abs=1e-6)


def test_circle_starts_at_its_pose():
    params = Params(x0=1.0, y0=-0.5, theta0=0.4)
    ref = reference(Circle(params), 0.0)
    assert ref.p == pytest.approx([1.0, -0.5])
    assert ref.q == pytest.approx([math.cos(0.4), math.sin(0.4)])


@pytest.mark.parametrize("name", ["circle", "circle_cw", "straight"])
@pytest.mark.parametrize("t", TIMES)
def test_circle_has_constant_speed_and_curvature(name, t):
    shape = SHAPES[name]()
    ref = reference(shape, t)
    assert ref.v == pytest.approx(shape.v)
    assert ref.kappa == pytest.approx(shape.kappa)


def test_gerono_is_centred_and_periodic():
    params = Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, period=30.0)
    shape = Gerono(params)
    assert shape.position(0.0) == pytest.approx([2.0, 1.0])
    for t in TIMES:
        assert shape.position(t + params.period) == pytest.approx(shape.position(t))


def test_gerono_long_axis_has_half_length_a():
    params = Params(x0=0.0, y0=0.0, theta0=0.0, a=1.5, period=30.0)
    xs = [Gerono(params).position(t)[0] for t in np.linspace(0, 30, 3001)]
    assert max(xs) == pytest.approx(1.5, abs=1e-3)
    assert min(xs) == pytest.approx(-1.5, abs=1e-3)


@pytest.mark.parametrize("name", SHAPES)
@pytest.mark.parametrize("t", TIMES)
def test_reference_is_consistent(name, t):
    shape = SHAPES[name]()
    ref = reference(shape, t)
    assert np.linalg.norm(ref.q) == pytest.approx(1.0)
    assert ref.v == pytest.approx(np.linalg.norm(shape.velocity(t)))
    assert ref.v * ref.q == pytest.approx(shape.velocity(t))
    assert ref.w == pytest.approx(ref.kappa * ref.v)
    # w* is the rate of change of the heading of q*: the angle from q*(t - H) to
    # q*(t + H), divided by 2H.
    before, after = reference(shape, t - H).q, reference(shape, t + H).q
    angle = math.atan2(before[0] * after[1] - before[1] * after[0], before @ after)
    assert ref.w == pytest.approx(angle / (2 * H), abs=1e-5)


def test_limits_reject_tight_circle():
    params = Params(shape="circle", circle_speed=0.3, kappa=5.0, kappa_max=2.9)
    with pytest.raises(ValueError, match="kappa_max"):
        check_limits(Circle(params), params, 10.0)


def test_limits_reject_fast_turn():
    params = Params(shape="circle", circle_speed=1.0, kappa=2.5, kappa_max=2.9, w_max=2.0)
    with pytest.raises(ValueError, match="w_max"):
        check_limits(Circle(params), params, 10.0)


def test_limits_accept_defaults():
    for shape in ("circle", "gerono"):
        params = Params(shape=shape)
        check_limits(Circle(params) if shape == "circle" else Gerono(params), params, 40.0)
