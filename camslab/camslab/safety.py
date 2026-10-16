"""Safety polytope: a convex counter-clockwise polygon the robot must stay inside."""

import math

import numpy as np


def cross(u: np.ndarray, v: np.ndarray) -> float:
    return float(u[0] * v[1] - u[1] * v[0])


class Polytope:
    """Vertices [x1, y1, x2, y2, ...], counter-clockwise around a convex polygon.

    signed_distance(p) = min_i cross(v_{i+1} - v_i, p - v_i) / ||v_{i+1} - v_i||, positive
    inside.
    """

    def __init__(self, xy):
        xy = [float(v) for v in xy]
        if len(xy) % 2 != 0:
            raise ValueError(f"polytope has {len(xy)} numbers, expected x, y pairs")
        self.vertices = np.array(xy).reshape(-1, 2)
        n = len(self.vertices)
        if n < 3:
            raise ValueError(f"polytope has {n} vertices, expected at least 3")
        for i in range(n):
            a, b, c = self.vertices[i], self.vertices[(i + 1) % n], self.vertices[(i + 2) % n]
            if cross(b - a, c - b) <= 0.0:
                raise ValueError(
                    f"polytope turns right or goes straight at vertex {(i + 1) % n + 1}; the "
                    "vertices must go counter-clockwise around a convex polygon (the web UI "
                    "writes them in this order)")

    def signed_distance(self, p) -> float:
        n = len(self.vertices)
        d = math.inf
        for i in range(n):
            a = self.vertices[i]
            edge = self.vertices[(i + 1) % n] - a
            d = min(d, cross(edge, np.asarray(p) - a) / float(np.linalg.norm(edge)))
        return d

    def contains(self, p) -> bool:
        return self.signed_distance(p) >= 0.0


def check_inside(shape, polytope: Polytope, margin: float, t_end: float,
                 samples: int = 2000) -> str:
    """Why p*(t) on [0, t_end] comes closer than margin to the polytope edge, or ""."""
    d_worst, t_worst, p_worst = math.inf, 0.0, np.zeros(2)
    for t in np.linspace(0.0, t_end, samples + 1):
        p = shape.position(t)
        d = polytope.signed_distance(p)
        if d < d_worst:
            d_worst, t_worst, p_worst = d, float(t), p
    if d_worst >= margin:
        return ""
    where = f"p*(t) = ({p_worst[0]:.2f}, {p_worst[1]:.2f}) m at t = {t_worst:.2f} s"
    if d_worst < 0.0:
        return f"{where} is {-d_worst:.3f} m outside the safety polytope"
    return (f"{where} is {d_worst:.3f} m from the safety polytope edge, less than "
            f"polytope_margin = {margin:.3f} m")
