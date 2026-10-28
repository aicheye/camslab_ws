"""Checks of the circular-path estimator."""

import numpy as np
import pytest

from camslab_standalone.path_circle.circle import Circle, filter_matrices, run


def test_filter_matrices_match_slide_47():
    lam = 2.0
    F, G, H = filter_matrices(lam, 2)
    assert F == pytest.approx(np.array([[-lam, 1.0], [0.0, -lam]]))
    assert G == pytest.approx(np.array([0.0, 1.0]))
    assert H == pytest.approx(np.array([[1.0, 0.0], [-lam, 1.0], [lam**2, -2 * lam]]))


def test_estimate_converges():
    circle = Circle()
    _, theta = run(circle, t_max=120.0, dt=0.1)
    assert theta[-1] == pytest.approx(circle.theta(), abs=0.05)


def test_starts_at_theta0():
    theta0 = np.array([1.5, 1.0, 3.0])
    _, theta = run(Circle(), t_max=1.0, theta0=theta0)
    assert theta[0] == pytest.approx(theta0)
