"""The simulation study of Section VIII of the paper: a planar robot alternating straight
segments with left and right turns, the camera extrinsic and three fiducials of the
paper, and the estimator of camslab.fiducial. The paper's covariance radii (Section VI)
are not computed."""

import numpy as np
from camslab.fiducial import Estimator, Scene, errors, paper_markers, robot_pose
from camslab.types import State

from ..common.unicycle import step
from .noise import noisy_heights, noisy_measurements
from camslab.fiducial import Noise

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


def run(t_max: float = 35.0, dt: float = 0.05, noise: Noise | None = None,
        seed: int | None = None, **estimator_kwargs):
    """Integrate the study and return (t, e, r): per sample the four errors of
    ERROR_NAMES and the four covariance radii of eq. (31), NaN until the laws switch
    on. noise adds the paper's measurement noise to what the estimator receives (the
    robot itself moves exactly) and is what the covariance prediction assumes; with
    None the run is exact and the prediction uses the paper's values."""
    rng = np.random.default_rng(seed)
    scene = Scene(markers=paper_markers())
    heights = scene.known_heights()
    if noise is not None:
        heights = noisy_heights(heights, noise, rng)
    estimator = Estimator(scene.n, heights, noise=noise, **estimator_kwargs)
    truth = scene.truth()
    state = State.from_pose(0.0, 0.0, 0.0)
    t = np.arange(0.0, t_max + dt / 2, dt)
    e = np.zeros((t.size, 4))
    r = np.zeros((t.size, 4))
    for k, tk in enumerate(t):
        T_R = robot_pose(state)
        observations = scene.observe(T_R)
        if noise is not None:
            T_R, observations, _ = noisy_measurements(T_R, observations, noise, rng)
        estimator.update(T_R, observations, dt)
        e[k] = errors(estimator.estimate(), truth, scene.n)
        r[k] = estimator.radii()
        state = step(state, *excitation(tk), dt)
    return t, e, r


def monte_carlo(runs: int, seed: int = 0, **kwargs):
    """(t, rmse, radius) over `runs` noisy runs: the root-mean-square error and the mean
    covariance radius of eq. (31), the solid and dashed curves of Fig. 1, each (N, 4)."""
    squares = radii = None
    for i in range(runs):
        t, e, r = run(noise=Noise(), seed=seed + i, **kwargs)
        squares = e**2 if squares is None else squares + e**2
        radii = r if radii is None else radii + r
    return t, np.sqrt(squares / runs), radii / runs
