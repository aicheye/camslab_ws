"""Offline check of a track: replay it with the paper's measurement noise, record every
robot pose and marker observation, then fit the camera extrinsic and the marker poses to
all of them at once by nonlinear least squares. The batch optimum tells whether the data
of the track determine the parameters at all, independently of the adaptive laws; the
online estimate is compared with it and with the truth.

    uv run python -m camslab_standalone.fiducial.offline [--out PNG]

reads the trajectory of the simulator at http://127.0.0.1:8000, or uses the default
bowtie when none is running, prints the comparison and plots the online error against
the batch optimum.
"""

import asyncio
import dataclasses
import json

import numpy as np
from scipy.optimize import least_squares

from camslab.fiducial import (ERROR_FIELDS, Noise, Scene, cayley_rotation, covariance_radius,
                              errors, fields)
from camslab.types import Params, State

from ..tracking.simulate import FiducialObserver, simulate


class RecordingObserver(FiducialObserver):
    """FiducialObserver that keeps what the estimator received: (T_R, {j: C_T_Mj})."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data = []
        update = self.estimator.update

        def recording_update(T_R, observations, dt):
            self.data.append((T_R, observations))
            update(T_R, observations, dt)

        self.estimator.update = recording_update


def batch_fit(data: list, heights: dict, n: int, noise: Noise, start: np.ndarray) -> dict:
    """Least squares over X = [x, t_X, m_1..m_n, t_1..t_n] (the estimator's layout) of
    eq. (1) for every recorded observation: the predicted C_T_Mj = (W_T_R X)^-1 Y_j
    against the measured one, rotations by the chordal distance ||R_pred - R_obs||_F,
    plus the known heights. Residuals are weighted by the noise standard deviations, so
    (J^T J)^-1 at the optimum is the covariance of the fit. Returns {"estimate",
    "cov", "sigma_min": smallest singular value of J, "cost"}."""
    R_R, p_R, who, R_obs, t_obs = [], [], [], [], []
    for T_R, observations in data:
        for j, T in observations.items():
            R_R.append(T_R[:3, :3])
            p_R.append(T_R[:3, 3])
            who.append(j)
            R_obs.append(T[:3, :3])
            t_obs.append(T[:3, 3])
    R_R, p_R, R_obs, t_obs = (np.array(a) for a in (R_R, p_R, R_obs, t_obs))
    who = np.array(who)
    known = sorted(heights)
    # chordal distance is sqrt(2) theta for a small angle theta
    w_rot = 1.0 / (np.sqrt(2.0) * noise.marker_rotation)
    w_pos = 1.0 / noise.marker_position
    w_h = 1.0 / noise.height

    def residuals(X):
        x, t_X = X[0:3], X[3:6]
        m = X[6:6 + 3 * n].reshape(n, 3)
        t = X[6 + 3 * n:].reshape(n, 3)
        R_Y = np.array([cayley_rotation(g) for g in m])
        R_C = R_R @ cayley_rotation(x)                               # W_R_C per sample
        R_pred = np.transpose(R_C, (0, 2, 1)) @ R_Y[who]
        d = t[who] - p_R - np.einsum("nij,j->ni", R_R, t_X)
        t_pred = np.einsum("nji,nj->ni", R_C, d)
        return np.concatenate([
            w_rot * (R_pred - R_obs).reshape(-1),
            w_pos * (t_pred - t_obs).reshape(-1),
            w_h * np.array([t[j, 2] - heights[j] for j in known]),
        ])

    fit = least_squares(residuals, start, method="lm", xtol=1e-12, ftol=1e-12)
    J = fit.jac
    s = np.linalg.svd(J, compute_uv=False)
    cov = np.linalg.pinv(J.T @ J)
    return {"estimate": fit.x, "cov": cov, "sigma_min": float(s[-1]), "cost": float(fit.cost)}


def verify(params: Params, initial: State, scene: Scene | None = None, dt: float = 0.02,
           t_max: float = 35.0, seed: int = 0, out: str | None = None) -> None:
    """Replay the track with the paper's noise, print the online, the batch and the true
    parameters' errors, the predicted radii, and the observability of the batch fit.
    Then show the online error against the batch optimum, or write it to `out`."""
    scene = Scene() if scene is None else scene
    noise = Noise()
    observer = RecordingObserver(scene, noise=noise, rng=np.random.default_rng(seed))
    rows = np.array(simulate(initial, params, dt, t_max, observer), dtype=float)
    n, truth = scene.n, observer.truth
    online = observer.estimator.estimate()
    online_radii = observer.estimator.radii()
    from_online = batch_fit(observer.data, observer.estimator.heights, n, noise, online)
    from_zero = batch_fit(observer.data, observer.estimator.heights, n, noise, np.zeros_like(online))

    def radii(cov):
        P_t = [cov[6 + 3 * n + 3 * j:9 + 3 * n + 3 * j, 6 + 3 * n + 3 * j:9 + 3 * n + 3 * j] for j in range(n)]
        P_m = [cov[6 + 3 * j:9 + 3 * j, 6 + 3 * j:9 + 3 * j] for j in range(n)]
        return [covariance_radius(cov[0:3, 0:3]), covariance_radius(cov[3:6, 3:6]),
                max(map(covariance_radius, P_m)), max(map(covariance_radius, P_t))]

    print(f"{params.shape} a={params.a} b={params.b}, {len(observer.data)} samples, "
          f"{sum(len(o) for _, o in observer.data)} observations, "
          f"online laws ready={observer.estimator.ready()}")
    print(f"{'':20s} " + " ".join(f"{name:>10s}" for name in ERROR_FIELDS))
    print(f"{'online error':20s} " + " ".join(f"{v:10.2e}" for v in errors(online, truth, n)))
    print(f"{'online radius':20s} " + " ".join(f"{v:10.2e}" for v in online_radii))
    print(f"{'batch from online':20s} " + " ".join(f"{v:10.2e}" for v in errors(from_online["estimate"], truth, n)))
    print(f"{'batch from zero':20s} " + " ".join(f"{v:10.2e}" for v in errors(from_zero["estimate"], truth, n)))
    print(f"{'batch radius':20s} " + " ".join(f"{v:10.2e}" for v in radii(from_online["cov"])))
    s_min = from_online["sigma_min"]
    print(f"batch Jacobian: smallest singular value {s_min:.3e}"
          + ("  (a direction of the parameters is not determined by this track)" if s_min < 1e-3 else ""))
    print("online x_hat ", np.array2string(online[0:3], precision=4), " t_X_hat", np.array2string(online[3:6], precision=4))
    print("batch  x     ", np.array2string(from_online["estimate"][0:3], precision=4), " t_X    ", np.array2string(from_online["estimate"][3:6], precision=4))
    print("true   x     ", np.array2string(truth[0:3], precision=4), " t_X    ", np.array2string(truth[3:6], precision=4))

    from .figures import TITLES, error_figure
    col = lambda name: 10 + observer.fields.index(name)
    e = rows[:, [col(n) for n in ERROR_FIELDS]]
    r = rows[:, [col(n) for n in ("r_x", "r_tX", "r_m", "r_t")]]
    fig = error_figure(rows[:, 0], e, r, TITLES, "Online error", "99.73 % radius",
                       f"Online estimator against the batch optimum, {params.shape} "
                       f"a = {params.a:.2f} m, b = {params.b:.2f} m",
                       batch=errors(from_online["estimate"], truth, n))
    if out:
        fig.savefig(out, dpi=200)
        print(f"figure written to {out}")
    else:
        import matplotlib.pyplot as plt
        plt.show()


async def current_track(url: str):
    """(Params, State) of the trajectory the running simulator shows, or None."""
    import aiohttp
    try:
        async with aiohttp.ClientSession() as s, s.ws_connect(url, timeout=2) as ws:
            while True:
                msg = json.loads((await asyncio.wait_for(ws.receive(), 2)).data)
                if msg["type"] == "config":
                    names = {f.name for f in dataclasses.fields(Params)}
                    p = {k: v for k, v in msg["params"].items() if k in names}
                    if "control_points" in p:
                        p["control_points"] = tuple(p["control_points"])
                    return Params(**p), State.from_pose(**msg["initial"])
    except Exception:
        return None


def main():
    import argparse
    ap = argparse.ArgumentParser(prog="camslab_standalone.fiducial.offline", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", metavar="PNG", help="write the figure instead of showing it")
    args = ap.parse_args()
    found = asyncio.run(current_track("http://127.0.0.1:8000/ws"))
    if found is None:
        print("no simulator on port 8000, using the default bowtie")
        found = (Params(shape="bowtie", x0=3.0, y0=3.0, a=2.0, b=1.2, period=40.0),
                 State.from_pose(3.0, 3.0, 0.5))
    verify(*found, out=args.out)


if __name__ == "__main__":
    main()
