"""Estimators by name."""

from .base import Estimator
from .circle import CircleEstimator

ESTIMATORS: dict[str, type[Estimator]] = {
    CircleEstimator.name: CircleEstimator,
}

MODES = ("open", "closed")


def make_estimator(name: str, path: dict) -> Estimator:
    if name not in ESTIMATORS:
        raise ValueError(f"unknown estimator {name!r}, one of {sorted(ESTIMATORS)}")
    return ESTIMATORS[name](path)
