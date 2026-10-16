"""Circle estimator: theta* = [r, c_x, c_y] of a circular path from p and xi."""

import math

import numpy as np

from ..trajectory import Reference
from .base import Estimator, Measurement
from .filters import filter_matrices
from .paths import circle_geometry


class CircleEstimator(Estimator):
    """The estimator part of ode() in camslab_offline/path_circle/circle.py, without the
    plant. See docs/path-estimation-hints.md for the derivation.

    X = [xi_f (5 filter states), r_hat, c_x_hat, c_y_hat], one filter state per signal in
    y = [xi^2 / 2, ||p||^2 / 2, xi, p_x, p_y].
    """

    name = "circle"
    fields = ("r", "c_x", "c_y")
    shapes = ("circle",)

    def __init__(self, path: dict, lam: float = 1.0, gamma: float = 10.0,
                 theta0=(0.5, 0.0, 0.0)):
        super().__init__(path, theta0)
        self.lam = lam
        self.gamma = gamma * np.eye(3)
        self.F, self.G, self.H = filter_matrices(lam, 1)
        self.phase = None

    def initial_state(self, m: Measurement) -> np.ndarray:
        """Filter states at the steady state of the first measurement, so the first
        s/Lambda y is 0. From 0, the first phi is as large as p, and forward Euler at
        dt = 0.02 s with gamma = 10 reached 1e6 for a robot starting at (4.2, 3.55)."""
        px, py = m.p
        y = np.array([m.xi**2 / 2, px**2 / 2 + py**2 / 2, m.xi, px, py])
        return np.concatenate((y / self.lam, self.theta0))

    def dynamics(self, X: np.ndarray, m: Measurement) -> np.ndarray:
        F, G, H, gamma = self.F, self.G, self.H, self.gamma

        xi_f = X[0:5]
        theta_hat = X[5:8]

        px, py = m.p
        xi = m.xi

        y = np.array([xi**2 / 2, px**2 / 2 + py**2 / 2, xi, px, py])
        W = np.array([H @ [xi_f[i]] + np.concatenate(([0], G)) * y[i] for i in range(5)])
        s_y = W[:, 1]

        z = s_y[0] - s_y[1]
        phi = -s_y[2:5]

        z_hat = theta_hat @ phi

        dxi_f = np.array([F @ [xi_f[i]] + G * y[i] for i in range(5)]).ravel()

        dtheta_hat = -gamma @ phi * (z_hat - z)

        return np.concatenate((dxi_f, dtheta_hat))

    def truth(self) -> np.ndarray:
        c, r = circle_geometry(self.path)
        return np.array([r, *c])

    def reference(self, m: Measurement, dt: float) -> Reference:
        """Certainty equivalence: drive around the estimated circle at circle_speed, in the
        direction of the path (the sign of kappa). r_hat is held at or above 1 / kappa_max,
        the tightest circle the car can turn."""
        r_hat, c_x_hat, c_y_hat = self.estimate()
        r_hat = max(r_hat, 1.0 / self.path["kappa_max"])
        direction = 1.0 if self.path["kappa"] >= 0 else -1.0
        v = self.path["circle_speed"]

        if self.phase is None:
            self.phase = math.atan2(m.p[1] - c_y_hat, m.p[0] - c_x_hat)
        else:
            self.phase += dt * direction * v / r_hat

        u = np.array([math.cos(self.phase), math.sin(self.phase)])
        tangent = np.array([-u[1], u[0]])
        return Reference(p=np.array([c_x_hat, c_y_hat]) + r_hat * u, q=direction * tangent,
                         v=v, w=direction * v / r_hat, kappa=direction / r_hat)
