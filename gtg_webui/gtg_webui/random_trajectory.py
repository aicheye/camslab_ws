"""Random trajectory parameters inside the working area, for the web UI's Random button.

Candidates are placed with two size bounds that follow from the shape definitions:

    circle   every point is within 2 / |kappa| (the diameter) of the start (x0, y0)
    gerono   every point is within a (the half-length) of the centre (x0, y0)

The circle's |kappa| is also kept at or below kappa_max and w_max / circle_speed. The
Gerono's curvature and turn rate depend on the trajectory functions, so the caller checks
each Gerono candidate (check_limits() offline, controller_node's parameter check in ROS)
and asks for another one when it is rejected.
"""

import math
import random

# The 0 to 6 m square shown by the web UI, Gazebo's camera, and the RViz grid [m].
AREA = (0.0, 0.0, 6.0, 6.0)  # x_min, y_min, x_max, y_max
MARGIN = 0.3                 # distance kept from the edges, about one robot length [m]
MIN_RADIUS = 0.5             # smallest circle radius picked [m]
MIN_A = 1.0                  # smallest Gerono half-length picked [m]
PERIOD_RANGE = (25.0, 60.0)  # Gerono lap time [s]


def _inner(area, margin):
    x_min, y_min, x_max, y_max = area
    return x_min + margin, y_min + margin, x_max - margin, y_max - margin


def random_params(shape, params, area=AREA, margin=MARGIN, rng=random):
    """Return a dict of trajectory parameters for `shape` ("circle" or "gerono").

    params gives kappa_max, w_max and circle_speed. Raises ValueError when the limits
    leave no circle that fits in the area.
    """
    x_min, y_min, x_max, y_max = _inner(area, margin)
    size = min(x_max - x_min, y_max - y_min)
    theta0 = rng.uniform(-math.pi, math.pi)

    if shape == "circle":
        # r = 1 / |kappa|. The circle spans 2r from its start in any direction, so the
        # start needs 2r of room on every side: 4r <= size.
        r_min = max(MIN_RADIUS, 1.0 / params["kappa_max"],
                    params["circle_speed"] / params["w_max"])
        r_max = size / 4.0
        if r_min > r_max:
            raise ValueError(f"no circle fits: radius must be at least {r_min:.2f} m "
                             f"and at most {r_max:.2f} m")
        r = rng.uniform(r_min, r_max)
        return {"x0": rng.uniform(x_min + 2 * r, x_max - 2 * r),
                "y0": rng.uniform(y_min + 2 * r, y_max - 2 * r),
                "theta0": theta0,
                "kappa": rng.choice((-1.0, 1.0)) / r}

    if shape == "gerono":
        a = rng.uniform(MIN_A, size / 2.0)
        return {"x0": rng.uniform(x_min + a, x_max - a),
                "y0": rng.uniform(y_min + a, y_max - a),
                "theta0": theta0,
                "a": a,
                "period": rng.uniform(*PERIOD_RANGE)}

    raise ValueError(f"unknown shape {shape!r}")


def path_inside(path, area=AREA):
    """True if every [x, y] of path is inside area."""
    x_min, y_min, x_max, y_max = area
    return all(x_min <= x <= x_max and y_min <= y <= y_max for x, y in path)
