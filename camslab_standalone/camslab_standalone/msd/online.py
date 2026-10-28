"""Online estimator in one ODE, the form on slides 45-47 of the estimation deck."""

import math

import numpy as np
from scipy.integrate import solve_ivp

from .plant import MsdParams, excitation


def filter_matrices(lam: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """F, G, H of the filter 1/(s + lam)^2, from slide 47."""
    F = np.array([[-lam, 1.0], [0.0, -lam]])
    G = np.array([0.0, 1.0])
    H = np.array([[1.0, 0.0], [-lam, 1.0], [lam**2, -2 * lam]])
    return F, G, H


def ode(
    t: float,
    X: np.ndarray,
    params: MsdParams,
    F: np.ndarray,
    G: np.ndarray,
    H: np.ndarray,
    gamma: np.ndarray,
) -> np.ndarray:
    """dX/dt, the Python version of dynamics() on slide 46.

    Use excitation(t) for the input u. Return an array with shape (9,).
    """
    x1 = X[0]
    x2 = X[1]
    xi_x = np.array(X[2:4])
    xi_f = np.array(X[4:6])
    theta_hat = np.array(X[6:])

    f = 2 * math.sin(0.7 * t) + 1.5 * math.sin(1.9 * t) + math.sin(3.1 * t)

    wx = H @ xi_x + np.concatenate(([0], G)) * x1
    wf = H @ xi_f + np.concatenate(([0], G)) * f

    phi = np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]]) @ wx
    z = [1, 0, 0] @ wf

    z_hat = theta_hat.T @ phi

    dx1 = x2
    dx2 = (-params.k * x1 - params.b * x2 + f) / params.m

    dxi_x = F @ xi_x + G * x1
    dxi_f = F @ xi_f + G * f

    dtheta_hat = -gamma @ phi * (z_hat - z)

    return np.concatenate(([dx1, dx2], dxi_x, dxi_f, dtheta_hat))


def run(
    params: MsdParams,
    t_max: float = 60.0,
    dt: float = 1e-3,
    lam: float = 2.0,
    gamma: np.ndarray | None = None,
    theta0: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate ode() from rest and return (t, theta_hat).

    t has shape (N,) with spacing dt; theta_hat has shape (N, 3), ordered [m, b, k].
    """
    F, G, H = filter_matrices(lam)
    gamma = 15.0 * np.eye(3) if gamma is None else gamma
    theta0 = np.zeros(3) if theta0 is None else theta0

    t = np.arange(0.0, t_max + dt / 2, dt)
    X0 = np.r_[np.zeros(6), theta0]
    sol = solve_ivp(
        ode,
        (t[0], t[-1]),
        X0,
        t_eval=t,
        args=(params, F, G, H, gamma),
        rtol=1e-8,
        atol=1e-10,
    )
    return sol.t, sol.y[6:9].T
