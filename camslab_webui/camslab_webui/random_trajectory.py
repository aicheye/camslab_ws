"""Random trajectory parameters inside the working area, for the web UI's Random button.

Candidates are placed with two size bounds that follow from the shape definitions:

    circle     every point is within 2 / |kappa| (the diameter) of the start (x0, y0)
    gerono     every point is within a (the half-length) of the centre (x0, y0)
    ellipse    every point is within a (the long half-axis, b <= a) of the centre (x0, y0)
    rectangle  every point is within sqrt(a^2 + b^2) (the half-diagonal) of the centre
    spline     every point is inside the convex hull of the control points, which are
               within R of their centre

The circle's |kappa| is also kept at or below kappa_max and w_max / circle_speed. The
ellipse's b is kept at or above sqrt(a / kappa_max), because its largest curvature is
a / b^2 at the ends of the long axis. The spline's control points go once around a
centre at evenly spread angles, so the control polygon does not cross itself, and some of
them are pulled in towards the centre, so the path has concave stretches. The rectangle
has kappa* = 0 on its sides. The turn rate of the Gerono, the ellipse and the spline
depends on the trajectory functions, so the caller checks each candidate (check_limits() offline,
controller's limit warning in ROS) and asks for another one when it breaks them.
"""

import math
import random

# The 0 to 6 m square shown by the web UI, Gazebo's camera, and the RViz grid [m].
AREA = (0.0, 0.0, 6.0, 6.0)  # x_min, y_min, x_max, y_max
MARGIN = 0.3                 # distance kept from the edges, about one robot length [m]
MIN_RADIUS = 0.5             # smallest circle radius picked [m]
MIN_A = 1.0                  # smallest Gerono or ellipse long half-axis picked [m]
MIN_B = 0.5                  # smallest ellipse short half-axis picked [m]
PERIOD_RANGE = (25.0, 60.0)  # Gerono, ellipse, rectangle or spline lap time [s]
MIN_HALF_SIDE = 0.5          # smallest rectangle half side picked [m]
SPLINE_POINTS = (6, 9)       # fewest and most spline control points picked
MIN_SPLINE_R = 2.0           # smallest spline control point radius R picked [m]
SPLINE_INNER_P = 0.35        # chance that a spline control point is an inner point
SPLINE_INNER_R = (0.1, 0.35) # inner point radius, as a fraction of R
SPLINE_OUTER_R = (0.85, 1.0) # other point radius, as a fraction of R


def _inner(area, margin):
    x_min, y_min, x_max, y_max = area
    return x_min + margin, y_min + margin, x_max - margin, y_max - margin


def random_params(shape, params, area=AREA, margin=MARGIN, rng=random):
    """Return a dict of trajectory parameters for `shape` ("circle", "gerono", "ellipse",
    "rectangle" or "spline"). The spline's control_points is a list [x0, y0, x1, y1, ...].

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
                "period": rng.uniform(*PERIOD_RANGE),
                "phi0": rng.uniform(-math.pi, math.pi)}

    if shape == "ellipse":
        a = rng.uniform(MIN_A, size / 2.0)
        b_min = max(MIN_B, math.sqrt(a / params["kappa_max"]))
        return {"x0": rng.uniform(x_min + a, x_max - a),
                "y0": rng.uniform(y_min + a, y_max - a),
                "theta0": theta0,
                "a": a,
                "b": rng.uniform(min(b_min, a), a),
                "period": rng.uniform(*PERIOD_RANGE),
                "phi0": rng.uniform(-math.pi, math.pi)}

    if shape == "rectangle":
        # The corners are sqrt(a^2 + b^2) from the centre in some direction. With both half
        # sides at most size / (2 sqrt(2)), that is at most size / 2.
        a = rng.uniform(MIN_HALF_SIDE, size / (2.0 * math.sqrt(2.0)))
        b = rng.uniform(MIN_HALF_SIDE, size / (2.0 * math.sqrt(2.0)))
        reach = math.hypot(a, b)
        return {"x0": rng.uniform(x_min + reach, x_max - reach),
                "y0": rng.uniform(y_min + reach, y_max - reach),
                "theta0": theta0,
                "a": a,
                "b": b,
                "period": rng.uniform(*PERIOD_RANGE)}

    if shape == "spline":
        # Point k is at angle theta0 + 2 pi k / n, moved by up to a quarter of the gap, so
        # the points go once counter-clockwise around the centre. Each point is an inner
        # point with probability SPLINE_INNER_P, at least one and at most n - 2 of them,
        # and the path bends inwards near those. The path stays in the convex hull of the
        # points, inside the circle of radius R.
        n = rng.randint(*SPLINE_POINTS)
        big_r = rng.uniform(MIN_SPLINE_R, size / 2.0)
        cx = rng.uniform(x_min + big_r, x_max - big_r)
        cy = rng.uniform(y_min + big_r, y_max - big_r)
        gap = 2.0 * math.pi / n
        inner = [rng.random() < SPLINE_INNER_P for _ in range(n)]
        if not any(inner):
            inner[rng.randrange(n)] = True
        while sum(inner) > n - 2:
            inner[rng.choice([k for k in range(n) if inner[k]])] = False
        points = []
        for k in range(n):
            angle = theta0 + k * gap + rng.uniform(-gap / 4.0, gap / 4.0)
            r = big_r * rng.uniform(*(SPLINE_INNER_R if inner[k] else SPLINE_OUTER_R))
            points += [round(cx + r * math.cos(angle), 2), round(cy + r * math.sin(angle), 2)]
        return {"control_points": points, "period": rng.uniform(*PERIOD_RANGE)}

    raise ValueError(f"unknown shape {shape!r}")


def path_inside(path, area=AREA):
    """True if every [x, y] of path is inside area."""
    x_min, y_min, x_max, y_max = area
    return all(x_min <= x <= x_max and y_min <= y <= y_max for x, y in path)
