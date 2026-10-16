from dataclasses import asdict, dataclass

import numpy as np


@dataclass
class State:
    """Robot state in the global frame.

    p = [x, y] is the position.
    q = [cos(theta), sin(theta)] is the heading unit vector, ||q|| = 1.
    """

    p: np.ndarray
    q: np.ndarray

    @classmethod
    def from_pose(cls, x, y, theta):
        return cls(p=np.array([x, y], dtype=float),
                   q=np.array([np.cos(theta), np.sin(theta)]))


SHAPES = ("circle", "gerono", "ellipse", "rectangle", "spline")


@dataclass
class Params:
    shape: str = "circle"
    x0: float = 2.0
    y0: float = 1.0
    theta0: float = 0.0
    circle_speed: float = 0.3
    kappa: float = 1.0
    a: float = 2.0
    b: float = 1.0
    period: float = 40.0
    phi0: float = 0.0
    control_points: tuple = (1.0, 0.8, 2.5, 1.2, 4.0, 0.8, 4.4, 2.5, 2.5, 3.2, 0.6, 2.5)

    k_par: float = 1.0
    k_perp: float = 4.0
    k_q: float = 2.0

    kappa_max: float = 2.9
    w_max: float = 2.0
    v_max: float = 0.6

    def to_dict(self):
        return asdict(self)
