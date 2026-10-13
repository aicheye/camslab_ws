"""Checks of trajectory.py. They fail with NotImplementedError until it is written.

The derivative tests compare velocity() and acceleration() with central differences
(f(t + h) - f(t - h)) / 2h of position() and velocity(), whose error is about h^2.
"""

import dataclasses
import math

import numpy as np
import pytest

from camslab_sim.trajectory import (Circle, Ellipse, Gerono, Spline, Rectangle, check_limits,
                                    make_shape, reference)
from camslab_sim.types import Params

H = 1e-5
# Six control points, so each spline segment lasts 30 / 6 = 5 s.
SPLINE_POINTS = (1.0, 0.8, 2.5, 1.2, 4.0, 0.8, 4.4, 2.5, 2.5, 3.2, 0.6, 2.5)
TIMES = [0.0, 0.7, 3.1, 11.0, 26.5]

SHAPES = {
    "circle": lambda: Circle(Params(x0=1.0, y0=-0.5, theta0=0.4, circle_speed=0.3, kappa=1.2)),
    "circle_cw": lambda: Circle(Params(theta0=2.0, circle_speed=0.5, kappa=-0.8)),
    "straight": lambda: Circle(Params(theta0=-1.0, circle_speed=0.4, kappa=0.0)),
    "gerono": lambda: Gerono(Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, period=30.0, phi0=0.8)),
    "ellipse": lambda: Ellipse(
        Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, b=1.0, period=30.0, phi0=0.8)),
    # Speed 4 (1.5 + 1.0) / 30 = 1/3 m/s, corners at t = 3, 12, 18 and 27 s, none of them in TIMES.
    "rectangle": lambda: Rectangle(
        Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, b=1.0, period=30.0)),
    "spline": lambda: Spline(Params(control_points=SPLINE_POINTS, period=30.0)),
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


@pytest.mark.parametrize("phi0", [0.8, -2.0])
def test_gerono_phi0_is_periodic_and_shifts_the_start(phi0):
    params = Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, period=30.0, phi0=phi0)
    shape = Gerono(params)
    assert shape.position(0.0) != pytest.approx([2.0, 1.0])
    for t in TIMES:
        assert shape.position(t + params.period) == pytest.approx(shape.position(t))
    full_lap = Gerono(dataclasses.replace(params, phi0=phi0 + 2 * np.pi))
    assert full_lap.position(0.0) == pytest.approx(shape.position(0.0))


def test_gerono_long_axis_has_half_length_a():
    params = Params(x0=0.0, y0=0.0, theta0=0.0, a=1.5, period=30.0)
    xs = [Gerono(params).position(t)[0] for t in np.linspace(0, 30, 3001)]
    assert max(xs) == pytest.approx(1.5, abs=1e-3)
    assert min(xs) == pytest.approx(-1.5, abs=1e-3)


def test_ellipse_starts_at_end_of_a_axis_and_is_periodic():
    params = Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, b=1.0, period=30.0)
    shape = Ellipse(params)
    assert shape.position(0.0) == pytest.approx([2.0 + 1.5 * math.cos(0.3),
                                                 1.0 + 1.5 * math.sin(0.3)])
    for t in TIMES:
        assert shape.position(t + params.period) == pytest.approx(shape.position(t))


def test_ellipse_has_half_axes_a_and_b():
    params = Params(x0=0.0, y0=0.0, theta0=0.0, a=1.5, b=1.0, period=30.0)
    points = np.array([Ellipse(params).position(t) for t in np.linspace(0, 30, 3001)])
    assert points[:, 0].max() == pytest.approx(1.5, abs=1e-3)
    assert points[:, 0].min() == pytest.approx(-1.5, abs=1e-3)
    assert points[:, 1].max() == pytest.approx(1.0, abs=1e-3)
    assert points[:, 1].min() == pytest.approx(-1.0, abs=1e-3)


def test_ellipse_curvature_at_axis_ends():
    # kappa* is a / b^2 at the ends of the a axis and b / a^2 at the ends of the b axis,
    # positive because the lap is counter-clockwise.
    params = Params(x0=0.0, y0=0.0, theta0=0.3, a=1.5, b=1.0, period=30.0)
    shape = Ellipse(params)
    assert reference(shape, 0.0).kappa == pytest.approx(1.5 / 1.0**2)
    assert reference(shape, params.period / 4).kappa == pytest.approx(1.0 / 1.5**2)


def test_rectangle_starts_mid_side_and_turns_at_its_corners():
    params = Params(x0=2.0, y0=1.0, theta0=0.3, a=1.5, b=1.0, period=30.0)
    shape = Rectangle(params)
    c, s = math.cos(0.3), math.sin(0.3)
    frame = lambda u, v: [2.0 + u * c - v * s, 1.0 + u * s + v * c]  # noqa: E731
    assert shape.position(0.0) == pytest.approx(frame(1.5, 0.0))
    corners = {3.0: (1.5, 1.0), 12.0: (-1.5, 1.0), 18.0: (-1.5, -1.0), 27.0: (1.5, -1.0)}
    for t, (u, v) in corners.items():
        assert shape.position(t) == pytest.approx(frame(u, v))
    for t in TIMES:
        assert shape.position(t + params.period) == pytest.approx(shape.position(t))
        ref = reference(shape, t)
        assert ref.v == pytest.approx(4 * (1.5 + 1.0) / 30.0)
        assert ref.kappa == pytest.approx(0.0)


def test_spline_is_periodic():
    shape = SHAPES["spline"]()
    for t in TIMES:
        assert shape.position(t + 30.0) == pytest.approx(shape.position(t))
        assert shape.velocity(t + 30.0) == pytest.approx(shape.velocity(t))


@pytest.mark.parametrize("join", [5.0, 10.0, 25.0, 30.0])
def test_spline_is_smooth_where_segments_meet(join):
    # Position, velocity and acceleration just before a segment ends match those just
    # after the next one starts.
    shape = SHAPES["spline"]()
    for f in (shape.position, shape.velocity, shape.acceleration):
        assert f(join - 1e-9) == pytest.approx(f(join + 1e-9), abs=1e-6)


def test_spline_stays_inside_its_control_points():
    # The control points are a convex polygon here, so every p*(t) is inside it: on the
    # left of each edge, going counter-clockwise.
    square = (0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0)
    shape = Spline(Params(control_points=square, period=20.0))
    corners = np.array(square).reshape(-1, 2)
    for t in np.linspace(0.0, 20.0, 401):
        p = shape.position(t)
        for a, b in zip(corners, np.roll(corners, -1, axis=0)):
            edge, to_p = b - a, p - a
            assert edge[0] * to_p[1] - edge[1] * to_p[0] > 0.0


def test_spline_needs_four_points():
    with pytest.raises(ValueError, match="at least 4"):
        Spline(Params(control_points=(0.0, 0.0, 1.0, 0.0, 1.0, 1.0)))


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
    for shape in ("circle", "gerono", "ellipse", "spline"):
        params = Params(shape=shape)
        check_limits(make_shape(params), params, 40.0)
