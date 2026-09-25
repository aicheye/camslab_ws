"""Reference trajectories p*(t).

Each shape gives the reference position and its first two time derivatives:

    position(t)       p*(t)            [m]
    velocity(t)       dp*/dt           [m/s]
    acceleration(t)   d^2 p*/dt^2      [m/s^2]

Differentiate position(t) by hand for velocity(t) and acceleration(t).
tests/test_trajectory.py compares them with finite differences of position(t).

reference(shape, t) turns the three into what control() uses: p*, q*, v*, w* and the
curvature kappa*. check_limits() samples reference() and rejects a trajectory whose
|kappa*| or |w*| is above the limits in Params, before any simulation runs.
"""

import math
from dataclasses import dataclass

import numpy as np

from .types import Params


@dataclass
class Reference:
    p: np.ndarray  # p*(t) [m]
    q: np.ndarray  # q*(t), unit tangent of the path, ||q*|| = 1
    v: float  # v*(t) = ||dp*/dt|| [m/s]
    w: float  # w*(t), from dq*/dt = w* S q* [rad/s]
    kappa: float  # kappa*(t), curvature, w* = kappa* v* [1/m]


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
        a = math.cos(self.theta0)
        b = math.sin(self.theta0)
        c = math.cos(self.kappa * self.v * t)
        d = math.sin(self.kappa * self.v * t)
        return np.array([a * c - b * d, b * c + a * d])

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
    half-length a, one lap every `period` seconds. p*(0) = (x0, y0), and p*(t + period)
    = p*(t).

    Reference: https://en.wikipedia.org/wiki/Lemniscate_of_Gerono (parametric form).
    """

    def __init__(self, params: Params):
        self.center = np.array([params.x0, params.y0])
        self.theta0 = params.theta0
        self.a = params.a
        self.period = params.period

    def position(self, t: float) -> np.ndarray:
        raise NotImplementedError(
            "Gerono.position() in trajectory.py is not implemented"
        )

    def velocity(self, t: float) -> np.ndarray:
        raise NotImplementedError(
            "Gerono.velocity() in trajectory.py is not implemented"
        )

    def acceleration(self, t: float) -> np.ndarray:
        raise NotImplementedError(
            "Gerono.acceleration() in trajectory.py is not implemented"
        )


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
