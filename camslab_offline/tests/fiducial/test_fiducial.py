"""camslab.fiducial, step by step: the Cayley algebra, the regressions, the information
sums, the translation and marker outputs, the histories, and the whole estimator on the
paper's study. See docs/fiducial-hints.md for the order."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camslab.fiducial import (E3, Estimator, History, Scene, cayley_rotation, compose,
                              errors, height_output, information, inverse,
                              marker_orientation_observation,
                              marker_position_observation, paper_markers, pose,
                              robot_pose, rotate, rotation_cayley, rotation_regression,
                              rpy_rotation, skew, translation_information,
                              translation_regression, translation_rhs)
from camslab.types import State
from camslab_offline.common.unicycle import step
from camslab_offline.fiducial.study import excitation, run

RNG = np.random.default_rng(1)
X = np.array([0.12, -0.08, 0.18])
T_X = np.array([0.18, -0.06, 0.32])


def random_cayley(scale=0.5):
    return RNG.normal(size=3) * scale


# --- step 1: Cayley coordinates, eq. (2) to (4) ---------------------------------------


def test_skew_is_the_cross_product():
    v, w = RNG.normal(size=3), RNG.normal(size=3)
    assert skew(v) @ w == pytest.approx(np.cross(v, w))


def test_cayley_rotation_is_a_rotation_by_2_atan_norm_about_g():
    g = random_cayley()
    R = cayley_rotation(g)
    assert R @ R.T == pytest.approx(np.eye(3))
    assert np.linalg.det(R) == pytest.approx(1.0)
    angle = 2.0 * math.atan(np.linalg.norm(g))
    expected = Rotation.from_rotvec(angle * g / np.linalg.norm(g)).as_matrix()
    assert R == pytest.approx(expected)


def test_rotation_cayley_inverts_cayley_rotation():
    g = random_cayley()
    assert rotation_cayley(cayley_rotation(g)) == pytest.approx(g)


def test_compose_is_the_cayley_vector_of_the_product():
    g1, g2 = random_cayley(), random_cayley()
    assert cayley_rotation(compose(g1, g2)) == pytest.approx(
        cayley_rotation(g1) @ cayley_rotation(g2))


def test_rotate_matches_the_matrix_for_one_and_many_vectors():
    g = random_cayley()
    R = cayley_rotation(g)
    v = RNG.normal(size=3)
    V = RNG.normal(size=(6, 3))
    assert rotate(g, v) == pytest.approx(R @ v)
    assert rotate(g, V) == pytest.approx(V @ R.T)


# --- step 2: orientation regressions, eq. (7) and (8) ----------------------------------


def relative_pair(yaw: float, displacement):
    """(A, B) of eq. (6) for a robot that yaws and moves by `displacement` in its own
    frame, with the true extrinsic and a marker."""
    X_true = pose(cayley_rotation(X), T_X)
    Y = paper_markers()[0].pose()
    T_Ri = robot_pose(State.from_pose(0.3, -0.4, 0.2))
    T_Rk = T_Ri @ pose(rpy_rotation(0.0, 0.0, yaw), displacement)
    T_Mi = inverse(T_Ri @ X_true) @ Y
    T_Mk = inverse(T_Rk @ X_true) @ Y
    return inverse(T_Ri) @ T_Rk, T_Mi @ inverse(T_Mk)


def test_rotation_regression_is_exact_for_a_yaw_pair():
    A, B = relative_pair(0.7, [0.4, 0.1, 0.0])
    Phi, y = rotation_regression(rotation_cayley(A[:3, :3]), rotation_cayley(B[:3, :3]))
    assert Phi.shape == (3, 3) and y.shape == (3,)
    assert Phi @ X == pytest.approx(y)


def test_rotation_regression_has_a_null_direction():
    """One yaw pair cannot identify all of x: Phi^R is singular."""
    A, B = relative_pair(0.7, [0.0, 0.0, 0.0])
    Phi, _ = rotation_regression(rotation_cayley(A[:3, :3]), rotation_cayley(B[:3, :3]))
    assert np.linalg.matrix_rank(Phi) == 2


def test_translation_regression_is_exact_for_a_straight_pair():
    A, B = relative_pair(0.0, [0.5, 0.0, 0.0])
    Phi, y = translation_regression(B[:3, 3], A[:3, 3])
    assert Phi @ X == pytest.approx(y)


def test_yaw_and_straight_pairs_together_identify_x():
    pairs = [relative_pair(0.7, [0.4, 0.1, 0.0]), relative_pair(-0.5, [0.2, 0.0, 0.0])]
    regressions = [rotation_regression(rotation_cayley(A[:3, :3]),
                                       rotation_cayley(B[:3, :3])) for A, B in pairs]
    A, B = relative_pair(0.0, [0.5, 0.0, 0.0])
    regressions.append(translation_regression(B[:3, 3], A[:3, 3]))
    G, h = information(np.array([r[0] for r in regressions]),
                       np.array([r[1] for r in regressions]))
    assert np.linalg.eigvalsh(G)[0] > 1e-3
    assert np.linalg.solve(G, h) == pytest.approx(X)


# --- step 3: information sums, eq. (11) ------------------------------------------------


def test_information_is_the_sum_of_normal_equations():
    Phi = RNG.normal(size=(4, 3, 3))
    y = RNG.normal(size=(4, 3))
    G, h = information(Phi, y)
    assert G == pytest.approx(sum(P.T @ P for P in Phi))
    assert h == pytest.approx(sum(P.T @ v for P, v in zip(Phi, y)))


# --- step 4: translation outputs, eq. (18) to (20) --------------------------------------


def height_data(n=5):
    """Known-height observations of marker 0 from n random robot poses."""
    X_true = pose(cayley_rotation(X), T_X)
    marker = paper_markers()[0]
    h, p_z, z = [], [], []
    for _ in range(n):
        T_R = robot_pose(State.from_pose(*RNG.uniform(-2, 2, 2), RNG.uniform(-3, 3)))
        T_M = inverse(T_R @ X_true) @ marker.pose()
        h.append(marker.position[2])
        p_z.append(T_R[2, 3])
        z.append(T_M[:3, 3])
    return np.array(h), np.array(p_z), np.array(z)


def test_height_output_is_the_camera_height_with_the_true_orientation():
    h, p_z, z = height_data()
    assert height_output(X, h, p_z, z) == pytest.approx(np.full(h.size, T_X[2]))


def test_translation_information_and_rhs_give_the_true_translation():
    pairs = [relative_pair(0.7, [0.4, 0.1, 0.0]), relative_pair(-0.5, [0.2, 0.0, 0.0])]
    Phi_t = np.array([A[:3, :3] - np.eye(3) for A, _ in pairs])
    t_A = np.array([A[:3, 3] for A, _ in pairs])
    t_B = np.array([B[:3, 3] for _, B in pairs])
    h, p_z, z = height_data()
    G_t = translation_information(Phi_t, h.size)
    assert G_t == pytest.approx(sum(P.T @ P for P in Phi_t) + h.size * np.outer(E3, E3))
    assert np.linalg.eigvalsh(G_t)[0] > 1e-3
    h_t = translation_rhs(X, Phi_t, t_A, t_B, h, p_z, z)
    assert np.linalg.solve(G_t, h_t) == pytest.approx(T_X)


# --- step 5: marker observations, eq. (23) and (25) ------------------------------------


def test_marker_observations_are_exact_with_the_true_extrinsic():
    scene = Scene(markers=paper_markers())
    T_R = robot_pose(State.from_pose(0.5, 0.2, 2.9))
    truth = scene.truth()
    for j, T_M in scene.observe(T_R).items():
        m_obs = marker_orientation_observation(T_R[:3, :3], X, T_M[:3, :3])
        t_obs = marker_position_observation(T_R[:3, 3], T_R[:3, :3], X, T_X, T_M[:3, 3])
        assert m_obs == pytest.approx(truth[6 + 3 * j:9 + 3 * j])
        assert t_obs == pytest.approx(truth[6 + 3 * scene.n + 3 * j:9 + 3 * scene.n + 3 * j])


# --- the provided parts: sensor and histories ------------------------------------------


def test_scene_observation_reconstructs_the_marker_pose():
    scene = Scene(markers=paper_markers())
    T_R = robot_pose(State.from_pose(0.3, -0.2, 0.7))
    for j, T in scene.observe(T_R).items():
        assert T_R @ scene.camera() @ T == pytest.approx(scene.marker_poses()[j])


def test_scene_field_of_view_hides_markers_behind_the_camera():
    scene = Scene(markers=paper_markers(), fov=math.radians(90.0))
    ahead = robot_pose(State.from_pose(0.0, 1.0, 0.0))
    behind = robot_pose(State.from_pose(0.0, 1.0, math.pi))
    assert 0 in scene.observe(ahead)
    assert 0 not in scene.observe(behind)


def test_history_keeps_its_excitation_when_full():
    history = History(10)
    strong = [np.diag([1.0, 1.0, 1.0])] * 10
    for info in strong:
        history.push(("strong",), info)
    history.push(("weak",), np.diag([0.0, 0.0, 1e-3]))
    assert len(history) == 10
    assert np.linalg.eigvalsh(history.G())[0] == pytest.approx(9.0)


# --- step 6: the whole estimator --------------------------------------------------------


def test_study_converges_without_noise():
    t, e = run(t_max=35.0, dt=0.05)
    assert e.shape == (t.size, 4)
    assert e[0] == pytest.approx([np.linalg.norm(X), np.linalg.norm(T_X), e[0, 2], e[0, 3]])
    assert np.all(e[-1] < 1e-6)


def test_estimate_is_ordered_as_truth():
    scene = Scene(markers=paper_markers())
    assert errors(scene.truth(), scene.truth(), scene.n) == pytest.approx(np.zeros(4))
    assert Estimator(scene.n, scene.known_heights()).estimate().shape == scene.truth().shape


def test_a_circle_leaves_the_orientation_unidentified():
    """Yaw-only motion excites two of the three Cayley components (Section IX)."""
    scene = Scene(markers=paper_markers())
    estimator = Estimator(scene.n, scene.known_heights())
    state, dt = State.from_pose(0.0, 0.0, 0.0), 0.05
    for _ in range(int(30.0 / dt)):
        T_R = robot_pose(state)
        estimator.update(T_R, scene.observe(T_R), dt)
        state = step(state, 0.5, 0.4, dt)
    assert estimator.ready() == (False, True)
    assert estimator.estimate()[0:3] == pytest.approx(np.zeros(3))


def test_excitation_alternates_straight_and_turns():
    assert excitation(1.0) == (0.5, 0.0)
    assert excitation(5.0)[1] > 0 > excitation(11.0)[1]
