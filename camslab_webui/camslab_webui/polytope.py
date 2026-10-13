"""Safety polytope from the web UI: convex hull of the clicked corners, and the controller
parameter file that keeps it across launches.

The file sets `polytope` ([x1, y1, x2, y2, ...], counter-clockwise, map frame) and
`polytope_margin` on every controller. robot.launch.py loads it after params.yaml when it
exists. Motion capture coordinates move with every calibration, so the corners are data in
this file and are set again in the web UI after a calibration.
"""

import datetime
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
