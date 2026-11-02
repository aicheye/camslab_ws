"""The measurement noise of the paper's Monte Carlo study (camslab.fiducial.Noise holds
the standard deviations), applied to what the fiducial estimator receives: the robot pose
and the marker observations. The robot itself moves exactly."""

import math

import numpy as np
from camslab.fiducial import Noise, pose


def exp_rotation(w: np.ndarray) -> np.ndarray:
    """Exp(w), the rotation matrix of the rotation vector w (Rodrigues' formula). Stands
    alone from the Cayley helpers so the noise model holds while those are stubbed."""
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3)
    K = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]]) / theta
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def perturb(T: np.ndarray, position_std: float, rotation_std: float, rng,
            yaw_only: bool = False) -> tuple:
    """(T with noise, |dp|, |w|): t <- t + dp, dp ~ N(0, position_std^2 I), and
    R <- R Exp(w), w ~ N(0, rotation_std^2 I), the perturbation composed on the right
    so it is expressed in the body frame. yaw_only draws w about the body z axis only."""
    dp = rng.normal(0.0, position_std, 3)
    if yaw_only:
        w = np.array([0.0, 0.0, rng.normal(0.0, rotation_std)])
    else:
        w = rng.normal(0.0, rotation_std, 3)
    T_noisy = pose(T[:3, :3] @ exp_rotation(w), T[:3, 3] + dp)
    return T_noisy, float(np.linalg.norm(dp)), float(np.linalg.norm(w))


def noisy_heights(heights: dict, noise: Noise, rng) -> dict:
    """The known marker heights with one Gaussian offset each, drawn once per run."""
    return {j: h + rng.normal(0.0, noise.height) for j, h in heights.items()}


# The noise drawn at one sample, the columns of the web UI's noise plot: the robot
# position and yaw perturbation, and the largest marker position and rotation
# perturbation among the visible markers.
NOISE_FIELDS = ("noise_pR", "noise_yaw", "noise_z", "noise_b")


def noisy_measurements(T_R: np.ndarray, observations: dict, noise: Noise, rng):
    """(T_R, observations, drawn) as the estimator receives them: yaw and position noise
    on the robot pose, position and rotation noise on every marker observation. drawn
    holds NOISE_FIELDS, the magnitudes of the perturbations of this sample."""
    T_R, dp, dpsi = perturb(T_R, noise.robot_position, noise.robot_yaw, rng, yaw_only=True)
    noisy, dz, db = {}, 0.0, 0.0
    for j, T in observations.items():
        noisy[j], dz_j, db_j = perturb(T, noise.marker_position, noise.marker_rotation, rng)
        dz, db = max(dz, dz_j), max(db, db_j)
    return T_R, noisy, [dp, dpsi, dz, db]
