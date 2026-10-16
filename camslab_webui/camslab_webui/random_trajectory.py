"""Random trajectory parameters inside the working area, for the web UI's Random button."""

import math
import random

AREA = (0.0, 0.0, 6.0, 6.0)
MARGIN = 0.3
MIN_RADIUS = 0.5
MIN_A = 1.0
MIN_B = 0.5
PERIOD_RANGE = (25.0, 60.0)
MIN_HALF_SIDE = 0.5
SPLINE_POINTS = (6, 9)
MIN_SPLINE_R = 2.0
SPLINE_INNER_P = 0.35
SPLINE_INNER_R = (0.1, 0.35)
SPLINE_OUTER_R = (0.85, 1.0)


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
