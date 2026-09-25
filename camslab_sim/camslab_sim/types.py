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
SHAPES = ("circle", "gerono")


@dataclass
class Params:
    # Trajectory. (x0, y0, theta0) is the pose of the trajectory frame: the circle starts
    # at (x0, y0) heading theta0, the Gerono lemniscate is centred on (x0, y0) with its
    # long axis along theta0.
    shape: str = "circle"
    x0: float = 2.0        # [m]
    y0: float = 1.0        # [m]
    theta0: float = 0.0    # [rad]
    circle_speed: float = 0.3  # circle: constant speed [m/s]
    kappa: float = 1.0     # circle: curvature 1/r [1/m], 0 is a straight line, > 0 turns left
    a: float = 2.0         # gerono: half the length of the long axis [m]
    period: float = 40.0   # gerono: time for one lap [s]

    # Control law gains, all > 0.
    k_par: float = 1.0     # k_parallel [1/s]
    k_perp: float = 4.0    # k_perp [1/m^2]
    k_q: float = 2.0       # k_q [1/s]

    # Limits the reference must stay inside (check_limits in trajectory.py).
    kappa_max: float = 2.9  # |kappa*| [1/m]; the Gazebo R2 turns at most tan(0.6) / 0.235
    w_max: float = 2.0      # |w*| [rad/s]

    def to_dict(self):
        return asdict(self)
