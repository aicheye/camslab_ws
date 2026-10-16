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
    name: ClassVar[str]
    fields: ClassVar[tuple[str, ...]]
    shapes: ClassVar[tuple[str, ...]]

    def __init__(self, path: dict):
        if path.get("shape") not in self.shapes:
            raise ValueError(f"estimator {self.name!r} needs shape in {self.shapes}, "
                             f"got {path.get('shape')!r}")
        self.path = dict(path)

    @abstractmethod
    def update(self, m: Measurement, dt: float) -> None:
        """Advance the filters and the adaptive law by dt with the measurement m."""

    @abstractmethod
    def estimate(self) -> np.ndarray:
        """theta_hat now, ordered as fields."""

    def truth(self) -> np.ndarray | None:
        """theta* from the true path, ordered as fields, for comparison only. None when
        it is unknown."""
        return None

    def reference(self, m: Measurement, dt: float) -> Reference | None:
        """Closed loop: the reference control() tracks, built from theta_hat (the mapping
        mechanism). None when the estimator has no closed-loop mode."""
        return None
