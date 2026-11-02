"""The fiducial estimator inside the closed-loop tracking simulator."""

import numpy as np
import pytest
from camslab.fiducial import ERROR_FIELDS, RADIUS_FIELDS, Noise, Scene, columns

from camslab_standalone.fiducial.noise import NOISE_FIELDS
from camslab_standalone.tracking.simulate import FiducialObserver, make_observer, simulate
from camslab.types import Params, State


def errors_of(row, observer):
    """The ERROR_FIELDS of a sample row."""
    return [row[10 + observer.fields.index(name)] for name in ERROR_FIELDS]


def test_rows_carry_estimates_truths_and_errors():
    observer = make_observer("fiducial", Params())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(), 0.02, 2.0, observer)
    n = observer.scene.n
    assert observer.fields == list(columns(n)) + list(NOISE_FIELDS)
    assert [p["fields"] for p in observer.plots] == [
        ["e_x", "e_tX", "r_x", "r_tX"], ["e_m", "e_t", "r_m", "r_t"], list(NOISE_FIELDS)]
    assert rows[0][10 + observer.fields.index("lam_x")] == 0.0
    assert rows[-1][10 + observer.fields.index("n_x")] >= 0
    assert all(len(row) == 10 + len(observer.fields) for row in rows)
    # the radii are None until the laws switch on and the noise columns without noise;
    # everything else is finite
    assert np.all(np.isfinite(np.array(rows, dtype=float)[:, :10 + len(columns(n)) - 4]))


def test_gerono_identifies_the_camera_and_the_markers():
    """The lemniscate has straight stretches at its crossing, so both pair types occur."""
    params = Params(shape="gerono")
    observer = FiducialObserver(Scene())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), params, 0.02, 60.0, observer)
    e_x, e_tX, e_m, e_t = errors_of(rows[-1], observer)
    assert observer.estimator.ready() == (True, True)
    assert e_x < 1e-3 and e_tX < 1e-3 and e_m < 1e-2 and e_t < 1e-2


def test_bowtie_identifies_the_camera_and_the_markers():
    """Straight edges give pure-translation pairs and the corners rotation pairs."""
    params = Params(shape="bowtie", x0=3.0, y0=3.0, a=2.0, b=1.2, period=40.0)
    observer = FiducialObserver(Scene())
    rows = simulate(State.from_pose(3.0, 3.0, 0.5), params, 0.02, 60.0, observer)
    e_x, e_tX, e_m, e_t = errors_of(rows[-1], observer)
    assert observer.estimator.ready() == (True, True)
    assert e_x < 1e-3 and e_tX < 1e-3 and e_m < 1e-2 and e_t < 1e-2


def test_circle_identifies_the_translation_but_not_the_orientation():
    observer = FiducialObserver(Scene())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(shape="circle"), 0.02, 30.0,
                    observer)
    assert observer.estimator.ready() == (False, True)
    assert errors_of(rows[-1], observer)[0] == pytest.approx(np.linalg.norm(Scene().camera_cayley))


def test_noise_perturbs_what_the_estimator_receives():
    """With the paper's noise the run stays finite and the errors no longer reach zero;
    the controller still tracks the true state, so the tracking columns are unchanged."""
    params = Params(shape="bowtie", x0=3.0, y0=3.0, a=2.0, b=1.2, period=40.0)
    initial = State.from_pose(3.0, 3.0, 0.5)
    clean = simulate(initial, params, 0.02, 40.0, FiducialObserver(Scene()))
    noisy_observer = FiducialObserver(Scene(), noise=Noise(), rng=np.random.default_rng(1))
    noisy = simulate(initial, params, 0.02, 40.0, noisy_observer)
    assert np.all(np.isfinite(np.array(noisy[-1], dtype=float)))
    assert np.array(noisy)[:, :10] == pytest.approx(np.array(clean)[:, :10])
    e_x, e_tX, e_m, e_t = errors_of(noisy[-1], noisy_observer)
    assert 0.0 < e_x < 0.1 and 0.0 < e_tX < 0.1 and 0.0 < e_m < 0.1 and 0.0 < e_t < 0.1
    radii = [noisy[-1][10 + noisy_observer.fields.index(name)] for name in RADIUS_FIELDS]
    assert all(0.0 < r < 1.0 for r in radii)
    drawn = [noisy[-1][10 + noisy_observer.fields.index(name)] for name in NOISE_FIELDS]
    assert all(0.0 < d < 0.05 for d in drawn)
