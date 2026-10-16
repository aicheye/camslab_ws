"""Reference trajectories p*(t)."""

import math
from dataclasses import dataclass

import numpy as np

from .types import Params


@dataclass
class Reference:
    p: np.ndarray
    q: np.ndarray
    v: float
    w: float
    kappa: float


class Circle:
    """Circle of curvature kappa driven at constant speed circle_speed, starting at (x0, y0)
    with heading theta0. kappa = 0 is a straight line, so position(t) must not divide by
    kappa. kappa > 0 turns left (counter-clockwise).
    """

    def __init__(self, params: Params):
        self.p0 = np.array([params.x0, params.y0])
        self.theta0 = params.theta0
        self.v = params.circle_speed
        self.kappa = params.kappa

    def heading(self, t: float) -> np.ndarray:
        return np.array(
            [
                math.cos(self.theta0 + self.kappa * self.v * t),
                math.sin(self.theta0 + self.kappa * self.v * t),
            ]
        )

    def position(self, t: float) -> np.ndarray:
        if abs(self.kappa) < 1e-9:
            return self.p0 + self.velocity(t) * t

        C = np.array(
            [
                self.p0[0] - math.sin(self.theta0) / self.kappa,
                self.p0[1] + math.cos(self.theta0) / self.kappa,
            ]
        )
        S = np.array([[0, 1], [-1, 0]])
        return (1 / self.kappa) * S @ self.heading(t) + C

    def velocity(self, t: float) -> np.ndarray:
        return self.v * self.heading(t)

    def acceleration(self, t: float) -> np.ndarray:
        R = np.array([[0, -1], [1, 0]])
        return self.kappa * self.v * self.v * R @ self.heading(t)


class Gerono:
    """Lemniscate of Gerono (a figure eight) centred on (x0, y0), long axis along theta0,
    half-length a, one lap every `period` seconds, at lap angle phi0 when t = 0. The
    crossing at (x0, y0) is lap angle 0, so p*(0) = (x0, y0) when phi0 = 0. p*(t + period)
    = p*(t).

    Reference: https://en.wikipedia.org/wiki/Lemniscate_of_Gerono (parametric form).
    """

    def __init__(self, params: Params):
        self.center = np.array([params.x0, params.y0])
        self.theta0 = params.theta0
        self.a = params.a
        self.period = params.period
        self.phi0 = params.phi0

        R = np.array(
            [
                [math.cos(self.theta0), -math.sin(self.theta0)],
                [math.sin(self.theta0), math.cos(self.theta0)],
            ]
        )

    def position(self, t: float) -> np.ndarray:
        R = np.array(
            [
                [math.cos(self.theta0), -math.sin(self.theta0)],
                [math.sin(self.theta0), math.cos(self.theta0)],
            ]
        )

        arg = self.phi0 + (2 * math.pi / self.period) * t
        i = np.array([self.a * math.sin(arg), (self.a / 2) * math.sin(2 * arg)])

        return R @ i + self.center

    def velocity(self, t: float) -> np.ndarray:
        R = np.array(
            [
                [math.cos(self.theta0), -math.sin(self.theta0)],
                [math.sin(self.theta0), math.cos(self.theta0)],
            ]
        )

        r = 2 * math.pi / self.period
        arg = self.phi0 + r * t
        i = np.array([r * self.a * math.cos(arg), r * self.a * math.cos(2 * arg)])

        return R @ i

    def acceleration(self, t: float) -> np.ndarray:
        R = np.array(
            [
                [math.cos(self.theta0), -math.sin(self.theta0)],
                [math.sin(self.theta0), math.cos(self.theta0)],
            ]
        )

        r = 2 * math.pi / self.period
        arg = self.phi0 + r * t
        i = np.array(
            [-r * r * self.a * math.sin(arg), -2 * r * r * self.a * math.sin(2 * arg)]
        )

        return R @ i


class Ellipse:
    """Ellipse centred on (x0, y0), half-axis a along theta0 and half-axis b perpendicular
    to it, one counter-clockwise lap every `period` seconds, at lap angle phi0 when t = 0.
    Lap angle 0 is the end of the a axis, (x0, y0) + a (cos theta0, sin theta0).
    p*(t + period) = p*(t).

    Reference: https://en.wikipedia.org/wiki/Ellipse#Parametric_representation
    """

    def __init__(self, params: Params):
        self.center = np.array([params.x0, params.y0])
        self.theta0 = params.theta0
        self.a = params.a
        self.b = params.b
        self.period = params.period
        self.phi0 = params.phi0

    def rotation(self) -> np.ndarray:
        c, s = math.cos(self.theta0), math.sin(self.theta0)
        return np.array([[c, -s], [s, c]])

    def position(self, t: float) -> np.ndarray:
        arg = self.phi0 + (2 * math.pi / self.period) * t
        return self.rotation() @ np.array([self.a * math.cos(arg), self.b * math.sin(arg)]) + self.center

    def velocity(self, t: float) -> np.ndarray:
        h = 2 * math.pi / self.period
        arg = self.phi0 + h * t
        return self.rotation() @ np.array([-self.a * h * math.sin(arg), self.b * h * math.cos(arg)])

    def acceleration(self, t: float) -> np.ndarray:
        h = 2 * math.pi / self.period
        arg = self.phi0 + h * t
        return self.rotation() @ np.array([-self.a * h * h * math.cos(arg),
                                           -self.b * h * h * math.sin(arg)])


class Rectangle:
    """Rectangle centred on (x0, y0) with side 2a along theta0 and side 2b perpendicular
    to it, one counter-clockwise lap every `period` seconds at constant speed
    4 (a + b) / period. p*(0) is the middle of the side at (x0, y0) + a (cos theta0,
    sin theta0), heading theta0 + pi/2. p*(t + period) = p*(t).

    Each side has kappa* = 0. At a corner q* turns by
    pi/2 at once, so dq*/dt is not defined there and the car cannot follow the corner
    exactly: expect ||p* - p|| to rise after each corner.
    """

    def __init__(self, params: Params):
        self.center = np.array([params.x0, params.y0])
        self.theta0 = params.theta0
        self.a = params.a
        self.b = params.b
        self.period = params.period

    def rotation(self) -> np.ndarray:
        c, s = math.cos(self.theta0), math.sin(self.theta0)
        return np.array([[c, -s], [s, c]])

    def side(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        """Position and velocity at time t in the rectangle frame."""
        a, b = self.a, self.b
        length = 4 * a + 4 * b
        v = length / self.period
        s = (v * t + b) % length
        sides = [(np.array([a, -b]), np.array([0.0, 1.0]), 2 * b),
                 (np.array([a, b]), np.array([-1.0, 0.0]), 2 * a),
                 (np.array([-a, b]), np.array([0.0, -1.0]), 2 * b),
                 (np.array([-a, -b]), np.array([1.0, 0.0]), 2 * a)]
        for corner, direction, side_length in sides:
            if s <= side_length:
                return corner + s * direction, v * direction
            s -= side_length
        corner, direction, _ = sides[-1]
        return corner + 2 * a * direction, v * direction

    def position(self, t: float) -> np.ndarray:
        return self.rotation() @ self.side(t)[0] + self.center

    def velocity(self, t: float) -> np.ndarray:
        return self.rotation() @ self.side(t)[1]

    def acceleration(self, t: float) -> np.ndarray:
        return np.zeros(2)


class Spline:
    """Closed uniform cubic B-spline through the control points P_0, ..., P_{N-1} of
    `control_points` (x0, y0, x1, y1, ...), one lap every `period` seconds, N >= 4.

    The curve has N segments, each lasting period / N seconds. At time t it is in segment
    i = floor(t / (period / N)) mod N, at local parameter u in [0, 1) through that segment,
    and p*(t) is a weighted sum of the four control points P_i, P_{i+1}, P_{i+2}, P_{i+3},
    with indices taken mod N so the curve closes. The weights are the four uniform cubic
    B-spline basis functions of u. p*(t + period) = p*(t).

    The curve passes near the control points but not through them. It stays inside the
    convex hull of the control points, so control points inside the safety polytope give a
    path inside it. Position, velocity and acceleration are continuous across segments, so
    w* and kappa* have no jumps.

    Reference: https://en.wikipedia.org/wiki/B-spline (cubic B-spline, matrix form).
    """

    def __init__(self, params: Params):
        points = np.asarray(params.control_points, dtype=float)
        if points.size % 2 != 0 or points.size < 8:
            raise ValueError(
                "control_points needs x, y pairs for at least 4 points, "
                f"got {points.size} numbers"
            )
        self.points = points.reshape(-1, 2)
        self.period = params.period

    def segment(self, t: float) -> tuple[int, int, float]:
        n = len(self.points)
        s = t / (self.period / n)
        return n, int(math.floor(s)) % n, s - math.floor(s)

    def blend(self, i: int, n: int, weights) -> np.ndarray:
        return sum(w * self.points[(i + k) % n] for k, w in enumerate(weights))

    def position(self, t: float) -> np.ndarray:
        n, i, u = self.segment(t)
        u2, u3 = u * u, u * u * u
        return self.blend(i, n, [(-u3 + 3 * u2 - 3 * u + 1) / 6, (3 * u3 - 6 * u2 + 4) / 6,
                                 (-3 * u3 + 3 * u2 + 3 * u + 1) / 6, u3 / 6])

    def velocity(self, t: float) -> np.ndarray:
        n, i, u = self.segment(t)
        u2 = u * u
        c = self.blend(i, n, [(-u2 + 2 * u - 1) / 2, (3 * u2 - 4 * u) / 2,
                              (-3 * u2 + 2 * u + 1) / 2, u2 / 2])
        return (n / self.period) * c

    def acceleration(self, t: float) -> np.ndarray:
        n, i, u = self.segment(t)
        d = self.blend(i, n, [1 - u, 3 * u - 2, 1 - 3 * u, u])
        return (n * n / (self.period * self.period)) * d


def reference(shape, t: float) -> Reference:
    """Return p*, q*, v*, w* and kappa* at time t, from shape.position(t),
    shape.velocity(t) and shape.acceleration(t)."""

    speed = np.linalg.norm(shape.velocity(t))
    S = np.array([[0, -1], [1, 0]])
    heading = shape.velocity(t) / speed
    angular = shape.acceleration(t) @ (S @ heading) / speed

    ret: Reference = Reference(
        shape.position(t),
        heading,
        speed,
        angular,
        angular / speed,
    )

    return ret


def make_shape(params: Params):
    if params.shape == "circle":
        return Circle(params)
    if params.shape == "gerono":
        return Gerono(params)
    if params.shape == "ellipse":
        return Ellipse(params)
    if params.shape == "rectangle":
        return Rectangle(params)
    if params.shape == "spline":
        return Spline(params)
    raise ValueError(f"unknown shape {params.shape!r}")


def sample_path(shape, t_end: float, n: int = 600):
    """[[x, y], ...] of p*(t) at n + 1 times evenly spaced over [0, t_end]."""
    return [
        [float(c) for c in shape.position(t)] for t in np.linspace(0.0, t_end, n + 1)
    ]


def check_limits(shape, params: Params, t_end: float, n: int = 2000):
    """Raise ValueError if |kappa*| > kappa_max or |w*| > w_max anywhere on [0, t_end]."""
    refs = [(t, reference(shape, t)) for t in np.linspace(0.0, t_end, n + 1)]
    t_k, worst_k = max(refs, key=lambda r: abs(r[1].kappa))
    if abs(worst_k.kappa) > params.kappa_max:
        raise ValueError(
            f"|kappa*| = {abs(worst_k.kappa):.3f} 1/m at t = {t_k:.2f} s "
            f"is above kappa_max = {params.kappa_max} 1/m"
        )
    t_w, worst_w = max(refs, key=lambda r: abs(r[1].w))
    if abs(worst_w.w) > params.w_max:
        raise ValueError(
            f"|w*| = {abs(worst_w.w):.3f} rad/s at t = {t_w:.2f} s "
            f"is above w_max = {params.w_max} rad/s"
        )
