"""Mass-spring-damper, the true system that generates the data.

    m x'' + b x' + k x = u

Only x is measured. x' and x'' are not available to the estimator.
"""

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class MsdParams:
    m: float = 1.5  # kg
    b: float = 0.8  # N s/m
    k: float = 4.0  # N/m

    def theta(self) -> np.ndarray:
        """True parameter vector, ordered [m, b, k]."""
        return np.array([self.m, self.b, self.k])


def excitation(t: np.ndarray) -> np.ndarray:
    """Input force u(t) in N: a sum of sinusoids at three frequencies."""
    return 1.0 * np.sin(0.7 * t) + 0.6 * np.sin(2.1 * t) + 0.4 * np.cos(4.3 * t)


def simulate(params: MsdParams, t_max: float = 60.0, dt: float = 1e-3,
             x0: float = 0.0, v0: float = 0.0, noise_std: float = 0.0,
             seed: int | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return samples (t, x, u) every dt seconds up to t_max.

    noise_std adds white Gaussian noise to the measured x.
    """
    t = np.arange(0.0, t_max + dt / 2, dt)

    def rhs(ti, s):
        u = excitation(np.asarray(ti))
        return [s[1], (u - params.b * s[1] - params.k * s[0]) / params.m]

    sol = solve_ivp(rhs, (0.0, t[-1]), [x0, v0], t_eval=t, rtol=1e-10, atol=1e-12,
                    max_step=0.01)
    x = sol.y[0]
    if noise_std > 0.0:
        x = x + np.random.default_rng(seed).normal(0.0, noise_std, x.shape)
    return t, x, excitation(t)
