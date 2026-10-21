"""The fiducial estimator inside the closed-loop tracking simulator."""

import numpy as np
import pytest
from camslab.fiducial import PLOT_FIELDS, Scene, columns

from camslab_offline.tracking.simulate import FiducialObserver, make_observer, simulate
from camslab.types import Params, State


def test_rows_carry_estimates_truths_and_errors():
    observer = make_observer("fiducial", Params())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(), 0.02, 2.0, observer)
    n = observer.scene.n
    assert observer.fields == list(columns(n))
    assert observer.plot == list(PLOT_FIELDS)
    assert all(len(row) == 10 + len(columns(n)) for row in rows)
    assert np.all(np.isfinite(np.array(rows, dtype=float)))


def test_gerono_identifies_the_camera_and_the_markers():
    """The lemniscate has straight stretches at its crossing, so both pair types occur."""
    params = Params(shape="gerono")
    observer = FiducialObserver(Scene())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), params, 0.02, 60.0, observer)
    e_x, e_tX, e_m, e_t = rows[-1][-4:]
    assert observer.estimator.ready() == (True, True)
    assert e_x < 1e-3 and e_tX < 1e-3 and e_m < 1e-2 and e_t < 1e-2


def test_circle_identifies_the_translation_but_not_the_orientation():
    observer = FiducialObserver(Scene())
    rows = simulate(State.from_pose(2.5, 0.5, 0.0), Params(shape="circle"), 0.02, 30.0,
                    observer)
    assert observer.estimator.ready() == (False, True)
    assert rows[-1][-4] == pytest.approx(np.linalg.norm(Scene().camera_cayley))
