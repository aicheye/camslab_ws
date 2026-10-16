"""Open-loop identification of a circular path, as on slides 44-47 of the estimation deck."""

import math
from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

N_FILTER_STATES = 5


@dataclass(frozen=True)
class Circle:
    c: tuple[float, float] = (2.0, 2.0)
    r: float = 1.0

    def theta(self) -> np.ndarray:
        """True parameter vector, ordered [r, c_x, c_y]."""
        return np.array([self.r, *self.c])


def excitation(t: float) -> tuple[float, float]:
    """Input (v [m/s], w [rad/s]): constant speed, turning at the rate of a 1.3 m loop plus
    two sinusoids, so the robot circles the path at a varying distance."""
    v = 0.3
    w = 0.3 / 1.3 + 0.5 * (math.sin(0.7 * t) + math.sin(1.9 * t))
    return v, w


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


def ode(
    t: float,
    X: np.ndarray,
    circle: Circle,
    F: np.ndarray,
    G: np.ndarray,
    H: np.ndarray,
    gamma: np.ndarray,
) -> np.ndarray:
    """dX/dt, in the same layout as dynamics() on slide 46.

    The plant is the unicycle p' = v [cos psi, sin psi], psi' = w, driven by
    excitation(t). Return an array with the same shape as X.
    """
    px = X[0]
    py = X[1]
    psi = X[2]
    xi_f = X[3:8]
    theta_hat = X[8:11]

    v, w = excitation(t)

    p = np.array([px, py])
    xi = np.linalg.norm(p - np.array(circle.c)) - circle.r

    y = np.array([xi**2 / 2, p @ p / 2, xi, px, py])
    W = np.array([H @ [xi_f[i]] + np.concatenate(([0], G)) * y[i] for i in range(5)])
    s_y = W[:, 1]

    z = s_y[0] - s_y[1]
    phi = -s_y[2:5]

    z_hat = theta_hat @ phi

    dpx = v * math.cos(psi)
    dpy = v * math.sin(psi)
    dpsi = w

    dxi_f = np.array([F @ [xi_f[i]] + G * y[i] for i in range(5)]).ravel()

    dtheta_hat = -gamma @ phi * (z_hat - z)

    return np.concatenate(([dpx, dpy, dpsi], dxi_f, dtheta_hat))


def run(
    circle: Circle,
    t_max: float = 120.0,
    dt: float = 0.01,
    lam: float = 1.0,
    gamma: np.ndarray | None = None,
    theta0: np.ndarray | None = None,
    pose0: tuple[float, float, float] = (3.3, 2.0, math.pi / 2),
) -> tuple[np.ndarray, np.ndarray]:
    """Integrate ode() and return (t, theta_hat).

    pose0 = (p_x, p_y, psi) is the initial robot pose, filter states start at 0, and
    theta_hat starts at theta0. theta_hat has shape (N, 3), ordered [r, c_x, c_y].
    """
    F, G, H = filter_matrices(lam, 1)
    gamma = 10.0 * np.eye(3) if gamma is None else gamma
    theta0 = np.array([0.5, 0.0, 0.0]) if theta0 is None else theta0

    t = np.arange(0.0, t_max + dt / 2, dt)
    X0 = np.r_[pose0, np.zeros(N_FILTER_STATES), theta0]
    sol = solve_ivp(
        ode,
        (t[0], t[-1]),
        X0,
        t_eval=t,
        args=(circle, F, G, H, gamma),
        rtol=1e-8,
        atol=1e-10,
    )
    return sol.t, sol.y[-3:].T


def main():
    import argparse

    ap = argparse.ArgumentParser(prog="camslab_offline.path_circle", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-max", type=float, default=120.0)
    ap.add_argument("--lam", type=float, default=1.0, help="filter pole")
    ap.add_argument("--gamma", type=float, default=10.0, help="gradient gain, gamma * I")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    circle = Circle()
    t, theta = run(circle, args.t_max, lam=args.lam, gamma=args.gamma * np.eye(3))

    names = ["r", "c_x", "c_y"]
    print("true      " + "  ".join(f"{s}={v:7.4f}" for s, v in zip(names, circle.theta())))
    print("estimate  " + "  ".join(f"{s}={v:7.4f}" for s, v in zip(names, theta[-1])))

    if args.no_plot:
        return
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    for i, ax in enumerate(axes):
        ax.plot(t, theta[:, i], label="estimate")
        ax.axhline(circle.theta()[i], color="k", ls="--", lw=1, label="true")
        ax.set_ylabel(names[i])
    axes[0].legend()
    axes[-1].set_xlabel("t [s]")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
