"""Adaptive path estimator: theta* = [r, c_x, c_y] of the circle that fits the path near the
robot, from p and xi. See docs/path-estimation-hints.md for the derivation."""

import math

import numpy as np
from scipy.integrate import solve_ivp

from .trajectory import make_shape, reference
from .types import Params

FIELDS = ("r", "c_x", "c_y")
COLUMNS = tuple(f + "_hat" for f in FIELDS) + tuple(f + "_true" for f in FIELDS)
# The web UI plot of the sample columns, see camslab_webui.server.adaptive_config().
PLOTS = [{"caption": "Estimates, dashed: true values", "fields": list(COLUMNS), "log": False}]


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


class Path:
    """One lap of the true path p*(t), sampled at n times, for the sensor and the true
    circle. The circle's lap lasts 2 pi / (|kappa| circle_speed), every other shape's lasts
    period."""

    def __init__(self, params: Params, n: int = 2000):
        if params.shape == "circle":
            if abs(params.kappa) < 1e-9:
                raise ValueError("a straight line (kappa = 0) has no lap")
            lap = 2 * math.pi / (abs(params.kappa) * params.circle_speed)
        else:
            lap = params.period
        self.shape = make_shape(params)
        self.t = np.linspace(0.0, lap, n + 1)
        self.points = np.array([self.shape.position(t) for t in self.t[:-1]])

    def closest(self, p: np.ndarray) -> tuple[float, float]:
        """(d, t): the distance from p to the polygon and the time of the closest point."""
        a = self.points
        ab = np.roll(a, -1, axis=0) - a
        s = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.einsum("ij,ij->i", ab, ab), 0.0, 1.0)
        d = np.linalg.norm(a + s[:, None] * ab - p, axis=1)
        i = int(np.argmin(d))
        return float(d[i]), float(self.t[i] + s[i] * (self.t[i + 1] - self.t[i]))

    def xi(self, p: np.ndarray) -> float:
        """The sensor: distance from p to the path, positive outside and negative inside
        (even-odd rule)."""
        d, _ = self.closest(p)
        a = self.points
        b = np.roll(a, -1, axis=0)
        crosses = (a[:, 1] > p[1]) != (b[:, 1] > p[1])
        x_cross = (a[crosses, 0] + (p[1] - a[crosses, 1]) * (b[crosses, 0] - a[crosses, 0])
                   / (b[crosses, 1] - a[crosses, 1]))
        inside = np.count_nonzero(x_cross > p[0]) % 2 == 1
        return -d if inside else d

    def truth(self, p: np.ndarray) -> np.ndarray:
        """theta* for comparison only: the osculating circle at the point of the path
        closest to p, r = 1 / |kappa*| and c = p* + n / kappa*, with n the left normal of
        q*. NaN where the path is straight."""
        _, t = self.closest(p)
        ref = reference(self.shape, t)
        if abs(ref.kappa) < 1e-6:
            return np.full(3, np.nan)
        n = np.array([-ref.q[1], ref.q[0]])
        return np.array([1.0 / abs(ref.kappa), *(ref.p + n / ref.kappa)])


class Estimator:
    """The estimator part of ode() in camslab_standalone/path_circle/circle.py, without the
    plant. It only observes: the controller tracks the true trajectory.

    X = [xi_f (5 filter states), r_hat, c_x_hat, c_y_hat], one filter state per signal in
    y = [xi^2 / 2, ||p||^2 / 2, xi, p_x, p_y]. The filter states start at 0. Each control
    tick integrates dynamics() over dt with solve_ivp, holding p and xi constant.
    """

    def __init__(self, params: Params, lam: float = 1.0, gamma: float = 10.0,
                 theta0=(0.5, 0.0, 0.0)):
        self.params = params
        self.gamma = gamma * np.eye(3)
        self.F, self.G, self.H = filter_matrices(lam, 1)
        self.X = np.concatenate((np.zeros(5), theta0))

    def dynamics(self, X: np.ndarray, p: np.ndarray, xi: float) -> np.ndarray:
        F, G, H, gamma = self.F, self.G, self.H, self.gamma

        xi_f = X[0:5]
        theta_hat = X[5:8]

        px, py = p

        y = np.array([xi**2 / 2, px**2 / 2 + py**2 / 2, xi, px, py])
        W = np.array([H @ [xi_f[i]] + np.concatenate(([0], G)) * y[i] for i in range(5)])
        s_y = W[:, 1]

        z = s_y[0] - s_y[1]
        phi = -s_y[2:5]

        z_hat = theta_hat @ phi

        dxi_f = np.array([F @ [xi_f[i]] + G * y[i] for i in range(5)]).ravel()

        dtheta_hat = -gamma @ phi * (z_hat - z)

        return np.concatenate((dxi_f, dtheta_hat))

    def update(self, p: np.ndarray, xi: float, dt: float) -> None:
        if dt <= 0:
            return
        sol = solve_ivp(lambda t, X: self.dynamics(X, p, xi), (0.0, dt), self.X,
                        rtol=1e-8, atol=1e-10)
        self.X = sol.y[:, -1]

    def estimate(self) -> np.ndarray:
        """theta_hat now, ordered as FIELDS."""
        return self.X[5:8].copy()
