"""The simulation study of Section VIII of the paper: a planar robot alternating straight
segments with left and right turns, the camera extrinsic and three fiducials of the
paper, and the estimator of camslab.fiducial. The paper's covariance radii (Section VI)
are not computed."""

import math
from dataclasses import dataclass

import numpy as np
from camslab.fiducial import (Estimator, Scene, errors, paper_markers, pose, robot_pose,
                              rotation_cayley, cayley_rotation)
from camslab.types import State

from ..common.unicycle import step

ERROR_NAMES = ("||x~||", "||t~_X|| [m]", "max_j ||m~_j||", "max_j ||t~_j|| [m]")


def excitation(t: float) -> tuple[float, float]:
    """Input (v [m/s], w [rad/s]): constant speed, a 2 s left turn after 4 s straight,
    then 4 s straight and a 2 s right turn, repeating every 12 s."""
    phase = t % 12.0
    if 4.0 <= phase < 6.0:
        return 0.5, 0.6
    if 10.0 <= phase < 12.0:
        return 0.5, -0.6
    return 0.5, 0.0


@dataclass(frozen=True)
class Noise:
    """Standard deviations of the paper's Monte Carlo study."""

    robot_position: float = 0.2e-3      # [m] per axis
    robot_yaw: float = math.radians(0.01)
    marker_position: float = 1e-3      # [m] per axis, in the camera frame
    marker_rotation: float = math.radians(0.1)
    height: float = 0.2e-3             # [m], one offset per run and marker


def perturb(T: np.ndarray, position_std: float, rotation_std: float, rng,
            yaw_only: bool = False) -> np.ndarray:
    """T with Gaussian position noise and a small random rotation on the left."""
    t = T[:3, 3] + rng.normal(0.0, position_std, 3)
    if yaw_only:
        axis_angle = np.array([0.0, 0.0, rng.normal(0.0, rotation_std)])
    else:
        axis_angle = rng.normal(0.0, rotation_std, 3)
    R = cayley_rotation(np.tan(np.linalg.norm(axis_angle) / 2.0)
                        * axis_angle / max(np.linalg.norm(axis_angle), 1e-300)) @ T[:3, :3]
    return pose(R, t)


def run(t_max: float = 35.0, dt: float = 0.05, noise: Noise | None = None,
        seed: int | None = None, **estimator_kwargs) -> tuple[np.ndarray, np.ndarray]:
    """Integrate the study and return (t, e) with e the four errors of ERROR_NAMES per
    sample. noise adds the paper's measurement noise to what the estimator receives; the
    robot itself moves exactly."""
    rng = np.random.default_rng(seed)
    scene = Scene(markers=paper_markers())
    heights = scene.known_heights()
    if noise is not None:
        heights = {j: h + rng.normal(0.0, noise.height) for j, h in heights.items()}
    estimator = Estimator(scene.n, heights, **estimator_kwargs)
    truth = scene.truth()
    state = State.from_pose(0.0, 0.0, 0.0)
    t = np.arange(0.0, t_max + dt / 2, dt)
    e = np.zeros((t.size, 4))
    for k, tk in enumerate(t):
        T_R = robot_pose(state)
        observations = scene.observe(T_R)
        if noise is not None:
            T_R = perturb(T_R, noise.robot_position, noise.robot_yaw, rng, yaw_only=True)
            observations = {j: perturb(T, noise.marker_position, noise.marker_rotation, rng)
                            for j, T in observations.items()}
        estimator.update(T_R, observations, dt)
        e[k] = errors(estimator.estimate(), truth, scene.n)
        state = step(state, *excitation(tk), dt)
    return t, e


def monte_carlo(runs: int, seed: int = 0, **kwargs) -> tuple[np.ndarray, np.ndarray]:
    """(t, rmse) over `runs` noisy runs, rmse of shape (N, 4)."""
    squares = None
    for i in range(runs):
        t, e = run(noise=Noise(), seed=seed + i, **kwargs)
        squares = e**2 if squares is None else squares + e**2
    return t, np.sqrt(squares / runs)
