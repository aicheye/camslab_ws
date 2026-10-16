"""Interface every adaptive estimator implements."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import ClassVar

import numpy as np

from ..trajectory import Reference


@dataclass
class Measurement:
    t: float
    p: np.ndarray
    q: np.ndarray
    v: float
    w: float
    xi: float


class Estimator(ABC):
    """An estimator is an ODE dX/dt = dynamics(X, m) in the layout of dynamics() on slide 46,
    stepped with forward Euler once per control tick. X holds the filter states followed
    by theta_hat, ordered as fields."""

    name: ClassVar[str]
    fields: ClassVar[tuple[str, ...]]
    shapes: ClassVar[tuple[str, ...]]

    def __init__(self, path: dict, theta0):
        if path.get("shape") not in self.shapes:
            raise ValueError(f"estimator {self.name!r} needs shape in {self.shapes}, "
                             f"got {path.get('shape')!r}")
        self.path = dict(path)
        self.theta0 = np.array(theta0, dtype=float)
        self.X = None

    @abstractmethod
    def initial_state(self, m: Measurement) -> np.ndarray:
        """X at the first measurement."""

    @abstractmethod
    def dynamics(self, X: np.ndarray, m: Measurement) -> np.ndarray:
        """dX/dt for the state X and the measurement m."""

    def update(self, m: Measurement, dt: float) -> None:
        if self.X is None:
            self.X = self.initial_state(m)
        self.X = self.X + dt * self.dynamics(self.X, m)

    def estimate(self) -> np.ndarray:
        """theta_hat now, ordered as fields."""
        if self.X is None:
            return self.theta0.copy()
        return self.X[-len(self.fields):].copy()

    def truth(self) -> np.ndarray | None:
        """theta* from the true path, ordered as fields, for comparison only. None when
        it is unknown."""
        return None

    def reference(self, m: Measurement, dt: float) -> Reference | None:
        """Closed loop: the reference control() tracks, built from theta_hat (the mapping
        mechanism). None when the estimator has no closed-loop mode."""
        return None
