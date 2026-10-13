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


# Values Params.shape can take.
SHAPES = ("circle", "gerono", "ellipse", "rectangle", "spline")


@dataclass
class Params:
    # Trajectory. (x0, y0, theta0) is the pose of the trajectory frame: the circle starts
    # at (x0, y0) heading theta0, the Gerono lemniscate and the ellipse are centred on
    # (x0, y0) with axis a along theta0 and start at lap angle phi0, the rectangle is
    # centred on (x0, y0) with sides 2a along theta0 and 2b. The spline ignores
    # (x0, y0, theta0) and goes around its control points.
    shape: str = "circle"
    x0: float = 2.0        # [m]
    y0: float = 1.0        # [m]
    theta0: float = 0.0    # [rad]
    circle_speed: float = 0.3  # circle: constant speed [m/s]
    kappa: float = 1.0     # circle: curvature 1/r [1/m], 0 is a straight line, > 0 turns left
    a: float = 2.0         # gerono, ellipse: half the long axis; rectangle: half the side along theta0 [m]
    b: float = 1.0         # ellipse: half the length of the short axis [m]
    period: float = 40.0   # gerono, ellipse, rectangle, spline: time for one lap [s]
    phi0: float = 0.0      # gerono, ellipse: lap angle at t = 0 [rad]
    # spline: control points (x0, y0, x1, y1, ...) [m], at least 4 points
    control_points: tuple = (1.0, 0.8, 2.5, 1.2, 4.0, 0.8, 4.4, 2.5, 2.5, 3.2, 0.6, 2.5)

    # Control law gains, all > 0.
    k_par: float = 1.0     # k_parallel [1/s]
    k_perp: float = 4.0    # k_perp [1/m^2]
    k_q: float = 2.0       # k_q [1/s]

    # Limits. A reference with |kappa*| > kappa_max or |w*| > w_max runs with a warning
    # (check_limits in trajectory.py), and every command is clamped to |v| <= v_max,
    # |w| <= min(w_max, kappa_max |v|) (clamp in simulate.py).
    kappa_max: float = 2.9  # [1/m]; the Gazebo R2 turns at most tan(0.6) / 0.235
    w_max: float = 2.0      # [rad/s]
    v_max: float = 0.6      # [m/s]

    def to_dict(self):
        return asdict(self)
