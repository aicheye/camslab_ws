"""Circle estimator: theta* = [r, c_x, c_y] of a circular path from p and xi."""

import math

import numpy as np

from ..trajectory import Reference
from .base import Estimator, Measurement
from .filters import filter_matrices
from .paths import circle_geometry

S = np.array([[0.0, -1.0], [1.0, 0.0]])


class CircleEstimator(Estimator):
    name = "circle"
    fields = ("r", "c_x", "c_y")
    shapes = ("circle",)

    def __init__(self, path: dict, lam: float = 1.0, gamma: float = 10.0,
                 theta0: tuple[float, float, float] = (0.5, 0.0, 0.0)):
        super().__init__(path)
        self.lam = lam
        self.gamma = gamma * np.eye(3)
        self.F, self.G, self.H = filter_matrices(lam, 1)
        self.theta_hat = np.array(theta0, dtype=float)
        self.xi_f = None
        self.phase = None

    def signals(self, m: Measurement) -> np.ndarray:
        return np.array([m.xi**2 / 2, m.p @ m.p / 2, m.xi, m.p[0], m.p[1]])

    def update(self, m: Measurement, dt: float) -> None:
        y = self.signals(m)
        if self.xi_f is None:
            self.xi_f = (y / self.lam)[:, None]

        W = self.xi_f @ self.H.T + np.outer(y, np.r_[0.0, self.G])
        s_y = W[:, 1]
        z = s_y[0] - s_y[1]
        phi = -s_y[2:5]
        z_hat = self.theta_hat @ phi

        dxi_f = self.xi_f @ self.F.T + np.outer(y, self.G)
        dtheta_hat = -self.gamma @ phi * (z_hat - z)

        self.xi_f = self.xi_f + dt * dxi_f
        self.theta_hat = self.theta_hat + dt * dtheta_hat

    def estimate(self) -> np.ndarray:
        return self.theta_hat.copy()

    def truth(self) -> np.ndarray:
        c, r = circle_geometry(self.path)
        return np.array([r, *c])

    def reference(self, m: Measurement, dt: float) -> Reference:
        """Certainty equivalence: drive around the estimated circle at circle_speed, in the
        direction of the path (the sign of kappa). r_hat is held at or above 1 / kappa_max,
        the tightest circle the car can turn."""
        r_hat = max(self.theta_hat[0], 1.0 / self.path["kappa_max"])
        c_hat = self.theta_hat[1:3]
        direction = 1.0 if self.path["kappa"] >= 0 else -1.0
        v = self.path["circle_speed"]
        if self.phase is None:
            d = m.p - c_hat
            self.phase = math.atan2(d[1], d[0])
        else:
            self.phase += dt * direction * v / r_hat
        u = np.array([math.cos(self.phase), math.sin(self.phase)])
        return Reference(p=c_hat + r_hat * u, q=direction * (S @ u), v=v,
                              w=direction * v / r_hat, kappa=direction / r_hat)
