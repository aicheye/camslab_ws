"""Safety polytope from the web UI: convex hull of the corners and its parameter file."""

import datetime
import math
import os


def convex_hull(points):
    """Counter-clockwise convex hull of [[x, y], ...] (Andrew's monotone chain). Points
    inside the hull or on an edge between two corners are left out."""
    pts = sorted({(float(x), float(y)) for x, y in points})
    if len(pts) < 3:
        return [list(p) for p in pts]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1] + upper[:-1]]


def write_file(path, corners, margin):
    """Write the controller parameter file, or delete it when corners is empty."""
    if not corners:
        if os.path.exists(path):
            os.remove(path)
        return
    flat = ", ".join(f"{c:.4f}" for corner in corners for c in corner)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    with open(path, "w") as f:
        f.write(f"""\
# Safety polytope, set in the web UI on {stamp}. Edit it there, not here.
# Corners [m] in the map (motion capture) frame, counter-clockwise. Set it again after
# every motion capture calibration, since the coordinates move with it.
#
# polytope         corners as [x1, y1, x2, y2, ...]. The controller refuses a reference path
#                  that leaves it, and disables itself if the robot leaves it.
# polytope_margin  distance the reference path keeps from every edge [m]
/**/controller:
  ros__parameters:
    polytope: [{flat}]
    polytope_margin: {float(margin)}
""")


def signed_distance(corners, p):
    """Distance of p to the nearest edge of the counter-clockwise convex polygon
    `corners`, positive inside."""
    d = math.inf
    for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1]):
        ex, ey = x1 - x0, y1 - y0
        d = min(d, (ex * (p[1] - y0) - ey * (p[0] - x0)) / math.hypot(ex, ey))
    return d


def hold_point(corners):
    """Controller parameters of a path that stays at the centroid of the safety polytope
    (it moves 1 mm/s), for a car that is driven by hand: the controller accepts a safety
    polytope only around its path, and this one is inside every area with a centre."""
    pts = [(float(x), float(y)) for x, y in corners]
    area = cx = cy = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        w = x0 * y1 - x1 * y0
        area, cx, cy = area + w, cx + (x0 + x1) * w, cy + (y0 + y1) * w
    return {"shape": "circle", "x0": round(cx / (3.0 * area), 3), "y0": round(cy / (3.0 * area), 3),
            "theta0": 0.0, "circle_speed": 0.001, "kappa": 0.0}


def fit_bowtie(corners, margin, kappa_max, corner_radius=0.0, a_max=2.0, aspect=0.5, speed=0.14,
               slack=0.05):
    """Controller parameters of the largest bowtie inside the safety polytope `corners`
    (counter-clockwise, convex): {shape, x0, y0, theta0, a, b, period}.

    The centre is the centroid of the polygon and the long side runs along its longest
    edge. The corners (+-a, +-b) with b = aspect * a keep margin + slack from every edge;
    the path itself is inside them because its corners are arcs of corner_radius, or of
    1 / kappa_max when that is 0. a is at most a_max and at least what those arcs need. The lap time gives about
    `speed` m/s. A ValueError says by how much the area is too small.

    Of the two directions along the longest edge, the one is taken in which a stand
    that faces the car arriving at a corner faces farthest from the +y axis of the
    frame: the estimator's marker orientation is singular for a rotation by 180 degrees
    from the frame, which an upright marker facing +y has."""
    pts = [(float(x), float(y)) for x, y in corners]
    area = cx = cy = 0.0
    longest, theta = 0.0, 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        w = x0 * y1 - x1 * y0
        area, cx, cy = area + w, cx + (x0 + x1) * w, cy + (y0 + y1) * w
        if math.hypot(x1 - x0, y1 - y0) > longest:
            longest, theta = math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0)
    cx, cy = cx / (3.0 * area), cy / (3.0 * area)
    diagonal = math.atan(aspect)

    def from_plus_y(theta0):
        facings = (theta0 + math.pi + diagonal, theta0 - diagonal, theta0 + math.pi / 2)
        return min(abs(math.remainder(f - math.pi / 2, 2 * math.pi)) for f in facings)

    theta0 = max((theta, theta + math.pi), key=from_plus_y)
    theta0 = math.remainder(theta0, 2 * math.pi)
    c, s = math.cos(theta0), math.sin(theta0)

    def clearance(a):
        b = aspect * a
        return min(signed_distance(pts, (cx + c * u - s * v, cy + s * u + c * v))
                   for u in (-a, a) for v in (-b, b))

    need = margin + slack
    radius = corner_radius or 1.0 / kappa_max
    a_min = math.tan(math.pi / 4 + diagonal / 2) * radius / aspect
    if clearance(a_min) < need:
        raise ValueError(
            f"the safe area is too small for a bowtie: the smallest one, {2 * a_min:.1f} m by "
            f"{2 * aspect * a_min:.1f} m, comes within {clearance(a_min):.2f} m of its edge "
            f"and needs {need:.2f} m. An area of {2 * (a_min + need):.1f} m by "
            f"{2 * (aspect * a_min + need):.1f} m is enough")
    low, high = a_min, a_max
    if clearance(high) < need:
        for _ in range(40):
            mid = 0.5 * (low + high)
            low, high = (mid, high) if clearance(mid) >= need else (low, mid)
        high = low
    a = math.floor(high * 100.0) / 100.0 if high > a_min + 0.01 else high
    b = aspect * a
    # length of the lap: two diagonals and two short sides, each corner cut by an arc
    turn = math.pi / 2 + diagonal
    lap = 4.0 * math.hypot(a, b) + 4.0 * b - 4.0 * radius * (2.0 * math.tan(turn / 2) - turn)
    return {"shape": "bowtie", "x0": round(cx, 3), "y0": round(cy, 3), "theta0": round(theta0, 4),
            "a": float(a), "b": float(b), "period": round(lap / speed, 1)}


def read_file(path):
    """(corners, margin) of a file written by write_file, or ([], 0.2) without one."""
    import yaml

    if not path or not os.path.isfile(path):
        return [], 0.2
    with open(path) as f:
        values = (yaml.safe_load(f) or {}).get("/**/controller", {}).get("ros__parameters", {})
    xy = [float(v) for v in values.get("polytope") or []]
    return [xy[i:i + 2] for i in range(0, len(xy) - 1, 2)], float(values.get("polytope_margin", 0.2))
