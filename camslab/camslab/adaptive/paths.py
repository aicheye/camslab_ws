"""True path geometry and the simulated distance sensor."""

import math

import numpy as np


def circle_geometry(path: dict) -> tuple[np.ndarray, float]:
    """Centre and radius of the `circle` shape: it starts at (x0, y0) heading theta0 with
    curvature kappa, as in camslab.trajectory.Circle."""
    kappa = path["kappa"]
    if abs(kappa) < 1e-9:
        raise ValueError("a straight line (kappa = 0) has no centre")
    c = np.array([path["x0"] - math.sin(path["theta0"]) / kappa,
                  path["y0"] + math.cos(path["theta0"]) / kappa])
    return c, 1.0 / abs(kappa)


def distance_to_path(path: dict, p: np.ndarray) -> float:
    """xi for the robot at p. Only the circle is implemented."""
    if path.get("shape") != "circle":
        raise ValueError(f"distance_to_path() supports the circle only, got {path.get('shape')!r}")
    c, r = circle_geometry(path)
    return float(np.linalg.norm(np.asarray(p) - c) - r)
