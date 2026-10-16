"""Repeated-pole filter, slides 22-27 of the estimation deck."""

import numpy as np


def filter_matrices(lam: float, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """F, G, H of the repeated-pole filter 1/(s + lam)^n, from slide 25.

    F is (n, n), G is (n,) and H is (n + 1, n). With xi' = F xi + G y, the filter output
    w = H xi + [0; G] y holds s^k / (s + lam)^n applied to y for k = 0, 1, ..., n.
    """
    F = -lam * np.eye(n) + np.eye(n, k=1)
    G = np.zeros(n)
    G[-1] = 1.0
    e1 = np.zeros(n)
    e1[0] = 1.0
    H = np.array([e1 @ np.linalg.matrix_power(F, k) for k in range(n + 1)])
    return F, G, H
