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


@dataclass
class Params:
    v_star: float = 0.5    # cruise speed used when ||p* - p|| > D [m/s]
    d_switch: float = 0.5  # D, the switching distance [m]
    k_v: float = 1.0       # Kv [1/s]
    k_w: float = 2.0       # Kw [1/s]
    epsilon: float = 0.05  # goal tolerance [m]

    def to_dict(self):
        return asdict(self)
