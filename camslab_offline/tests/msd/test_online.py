"""Checks of the one-ODE estimator in online.py."""

import numpy as np
import pytest

from camslab_offline.msd.online import run
from camslab_offline.msd.plant import MsdParams


def test_online_converges():
    params = MsdParams()
    t, theta = run(params, t_max=100.0, dt=1e-2, gamma=200.0 * np.eye(3))
    assert theta.shape == (t.size, 3)
    assert theta[-1] == pytest.approx(params.theta(), rel=1e-2)


def test_online_starts_at_theta0():
    theta0 = np.array([0.5, 2.5, 1.0])
    _, theta = run(MsdParams(), t_max=1.0, theta0=theta0)
    assert theta[0] == pytest.approx(theta0)
