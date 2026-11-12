"""Simultaneous camera-extrinsic calibration and fiducial surveying under planar motion,
after Choopojcharoen and Fidan, "Simultaneous Cascaded Adaptive Camera Calibration and
Robot-Assisted Fiducial Surveying under Planar Motion" (ECC 2027). Equation numbers below
refer to that paper. The functions marked "Step n" are the exercise: see
docs/fiducial-hints.md for the order and the tests that check each one.

Frames: {W} world (the map), {R} robot, {C} camera, {M_j} marker j. A pose A_T_B is a 4x4
homogeneous matrix mapping coordinates of {B} to {A}. The unknowns are the camera extrinsic
X = R_T_C, with Cayley vector x and position t_X, and the marker world poses Y_j = W_T_Mj,
with Cayley vectors m_j and positions t_j. The robot world pose W_T_R is known, and the
camera returns C_T_Mj for every visible marker.
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.integrate import solve_ivp

from .types import State

E3 = np.array([0.0, 0.0, 1.0])
I3 = np.eye(3)

# Camera-parameter names, then per marker j the three Cayley components m{j}_k and the
# three position components t{j}_k. Errors e_* are norms of estimate minus truth, the
# marker ones the maximum over the markers, as in Fig. 1 of the paper.
CAMERA_FIELDS = ("x_1", "x_2", "x_3", "tX_1", "tX_2", "tX_3")
ERROR_FIELDS = ("e_x", "e_tX", "e_m", "e_t")
# lambda_min(G_x), lambda_min(G_t) and the sizes of the three history buffers H_x, H_t, H_h
# of Section VII: how the information accumulates and when the camera laws switch on.
EXCITATION_FIELDS = ("lam_x", "lam_t", "n_x", "n_t", "n_h")
# The 99.73 % covariance radii of eq. (31), the dashed curves of Fig. 1, one per error
# of ERROR_FIELDS; the marker ones are the maximum over the markers.
RADIUS_FIELDS = ("r_x", "r_tX", "r_m", "r_t")
# chi^2 quantile with 3 degrees of freedom at 0.9973 (scipy.stats.chi2.ppf(0.9973, 3))
CHI2_3_9973 = 14.15625


def fields(n_markers: int) -> tuple[str, ...]:
    """Names of the 6 + 6 n_markers estimated parameters, in the order of the state."""
    names = list(CAMERA_FIELDS)
    for j in range(1, n_markers + 1):
        names += [f"m{j}_{k}" for k in (1, 2, 3)]
    for j in range(1, n_markers + 1):
        names += [f"t{j}_{k}" for k in (1, 2, 3)]
    return tuple(names)


def columns(n_markers: int) -> tuple[str, ...]:
    """Sample-row columns: every estimate, every true value, the four errors, the
    excitation of EXCITATION_FIELDS, then the four covariance radii of RADIUS_FIELDS."""
    f = fields(n_markers)
    return (tuple(n + "_hat" for n in f) + tuple(n + "_true" for n in f) + ERROR_FIELDS
            + EXCITATION_FIELDS + RADIUS_FIELDS)


def plots(estimator) -> list:
    """The web UI's plots of the sample columns, Fig. 1 of the paper in two panels: the
    camera errors and the marker errors on a log axis, each with its covariance radius
    dashed. The camera panel's tooltip adds the excitation, lambda_min of G_x and G_t
    against the gates alpha_x and alpha_t, and the history sizes. Each plot is
    {"caption", "fields", "log", "dashed": {field: field whose colour it takes},
    "dashed_label": legend entry for the dashed fields, "tooltip": [fields shown on
    hover only]}."""
    return [
        {"caption": "Camera extrinsic error",
         "fields": ["e_x", "e_tX", "r_x", "r_tX"], "log": True,
         "dashed": {"r_x": "e_x", "r_tX": "e_tX"}, "dashed_label": "99.73 % radius",
         "tooltip": list(EXCITATION_FIELDS)},
        {"caption": "Fiducial pose error, worst marker",
         "fields": ["e_m", "e_t", "r_m", "r_t"], "log": True,
         "dashed": {"r_m": "e_m", "r_t": "e_t"}, "dashed_label": "99.73 % radius"},
    ]


@dataclass(frozen=True)
class Noise:
    """Standard deviations of the measurement noise, the values of the paper's Monte
    Carlo study (Section VIII). The simulators add this noise to what the estimator
    receives; the estimator uses it for the covariance prediction of Section VI."""

    robot_position: float = 0.2e-3      # [m] per axis
    robot_yaw: float = math.radians(0.01)
    marker_position: float = 1e-3      # [m] per axis, in the camera frame
    marker_rotation: float = math.radians(0.1)  # [rad], small rotation per axis
    height: float = 0.2e-3             # [m], one offset per run and marker


# --- Cayley rotation coordinates (Section II.B) -----------------------------------------


def skew(v: np.ndarray) -> np.ndarray:
    """[v]_x, the matrix with [v]_x w = v x w."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def cayley_rotation(g: np.ndarray) -> np.ndarray:
    """R(g), the rotation matrix of the Cayley vector g, eq. (2). Step 1."""
    return (I3 + skew(g)) @ np.linalg.inv(I3 - skew(g))


def rotation_cayley(R: np.ndarray) -> np.ndarray:
    """The Cayley vector g of a rotation matrix R, the inverse of eq. (2):
    [g]_x = (R - I)(R + I)^-1. Undefined at a rotation by pi."""
    S = (R - I3) @ np.linalg.inv(R + I3)
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def compose(g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
    """g1 (+) g2, the Cayley vector of R(g1) R(g2), eq. (3). Step 1."""
    return (g1 + g2 + np.cross(g1, g2)) / (1 - g1 @ g2)


def rotate(g: np.ndarray, v: np.ndarray) -> np.ndarray:
    """R(g, v) = R(g) v without building the matrix, eq. (4). v is one vector or an
    (n, 3) array of vectors, one per row. Step 1."""
    return ((1 - g @ g) * v + 2 * np.multiply.outer(v @ g, g) + 2 * np.cross(g, v)) / (
        1 + g @ g
    )


def rotation_angle(R: np.ndarray) -> float:
    """The angle of a rotation matrix, in [0, pi]."""
    return math.acos(min(1.0, max(-1.0, (np.trace(R) - 1.0) / 2.0)))


# --- Homogeneous poses ------------------------------------------------------------------


def pose(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    return pose(R.T, -R.T @ T[:3, 3])


def rpy_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = R_z(yaw) R_y(pitch) R_x(roll), the URDF and SDF convention."""
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def robot_pose(state: State, z: float = 0.0) -> np.ndarray:
    """W_T_R of a planar robot state: heading q about the world z axis at height z."""
    R = np.array(
        [[state.q[0], -state.q[1], 0.0], [state.q[1], state.q[0], 0.0], [0.0, 0.0, 1.0]]
    )
    return pose(R, [state.p[0], state.p[1], z])


# --- The scene: the true camera and markers, and the pose sensor -----------------------


@dataclass
class Marker:
    """A fiducial at `position` [m] with orientation `rpy` [rad] in the world frame.
    known_height says whether the estimator is told the marker's world-frame height."""

    position: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)
    known_height: bool = False

    def pose(self) -> np.ndarray:
        return pose(rpy_rotation(*self.rpy), self.position)


def paper_markers() -> list:
    """The three fiducials of the paper's simulation study, with orientations chosen here
    (the paper gives none) well inside the Cayley chart. Heights of the first two are
    supplied to the estimator."""
    return [
        Marker((2.0, 1.0, 0.0), (0.0, 0.0, 0.8), True),
        Marker((-1.5, 2.2, 0.0), (0.3, 0.0, -1.2), True),
        Marker((1.0, -2.0, 0.4), (0.0, 0.4, 2.0), False),
    ]


def lab_markers() -> list:
    """Three fiducials inside the 6 m x 6 m lab area, around the default paths."""
    return [
        Marker((4.5, 1.0, 0.0), (0.0, 0.0, 2.5), True),
        Marker((0.5, 3.0, 0.0), (0.3, 0.0, -0.5), True),
        Marker((3.0, 4.0, 0.4), (0.0, 0.4, -1.8), False),
    ]


def random_scene(rng=None, area=(0.0, 0.0, 6.0, 6.0), n_markers: int = 3) -> "Scene":
    """A random camera extrinsic and n_markers random fiducials inside `area` [m]. The
    camera is rotated by 0.2 to 0.6 rad about a random axis and sits 0.1 to 0.3 m ahead
    of the rear axle, 0.2 to 0.4 m up; the markers stand up to 0.5 m high with roll and
    pitch within 0.4 rad and any yaw within 2.5 rad, well inside the Cayley chart. The
    heights of the first two markers are known to the estimator."""
    rng = np.random.default_rng(rng)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    cayley = tuple(float(c) for c in math.tan(rng.uniform(0.2, 0.6) / 2.0) * axis)
    position = (rng.uniform(0.1, 0.3), rng.uniform(-0.12, 0.12), rng.uniform(0.2, 0.4))
    x_min, y_min, x_max, y_max = area
    markers = [
        Marker(
            (rng.uniform(x_min + 0.5, x_max - 0.5), rng.uniform(y_min + 0.5, y_max - 0.5),
             rng.uniform(0.0, 0.5)),
            (rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-2.5, 2.5)),
            j < 2,
        )
        for j in range(n_markers)
    ]
    return Scene(cayley, tuple(float(c) for c in position), markers)


@dataclass
class Scene:
    """The truth: the camera extrinsic X = R_T_C as a Cayley vector and a position, and
    the markers. observe() is the sensor. fov is the full view angle about the camera's
    x axis and `range` the detection distance; the defaults see everything, as in the
    paper's study."""

    camera_cayley: tuple = (0.12, -0.08, 0.18)
    camera_position: tuple = (0.18, -0.06, 0.32)
    markers: list = field(default_factory=lab_markers)
    fov: float = 2.0 * math.pi
    range: float = math.inf

    @property
    def n(self) -> int:
        return len(self.markers)

    def camera(self) -> np.ndarray:
        """X = R_T_C."""
        return pose(
            cayley_rotation(np.asarray(self.camera_cayley, float)), self.camera_position
        )

    def marker_poses(self) -> list:
        """Y_j = W_T_Mj for every marker."""
        return [m.pose() for m in self.markers]

    def known_heights(self) -> dict:
        """{j: h_j} for the markers whose world-frame height is known, j from 0."""
        return {
            j: float(m.position[2])
            for j, m in enumerate(self.markers)
            if m.known_height
        }

    def observe(self, T_R: np.ndarray) -> dict:
        """The sensor: {j: C_T_Mj} for every marker visible from robot pose W_T_R = T_R,
        eq. (1) solved for the observation. Visible means inside `range` and within
        fov / 2 of the camera's x axis."""
        C_T_W = inverse(T_R @ self.camera())
        seen = {}
        for j, Y in enumerate(self.marker_poses()):
            T = C_T_W @ Y
            c = T[:3, 3]
            d = float(np.linalg.norm(c))
            if d > self.range:
                continue
            if d > 0.0 and math.acos(min(1.0, max(-1.0, c[0] / d))) > self.fov / 2.0:
                continue
            seen[j] = T
        return seen

    def truth(self) -> np.ndarray:
        """[x, t_X, m_1 .. m_N, t_1 .. t_N], the true value of Estimator.estimate()."""
        Ys = self.marker_poses()
        return np.concatenate(
            [
                np.asarray(self.camera_cayley, float),
                np.asarray(self.camera_position, float),
                *[rotation_cayley(Y[:3, :3]) for Y in Ys],
                *[Y[:3, 3] for Y in Ys],
            ]
        )


def errors(estimate: np.ndarray, truth: np.ndarray, n_markers: int) -> np.ndarray:
    """ERROR_FIELDS: ||x~||, ||t~_X||, max_j ||m~_j||, max_j ||t~_j||."""
    e = estimate - truth
    m = e[6 : 6 + 3 * n_markers].reshape(n_markers, 3)
    t = e[6 + 3 * n_markers :].reshape(n_markers, 3)
    return np.array(
        [
            np.linalg.norm(e[0:3]),
            np.linalg.norm(e[3:6]),
            np.max(np.linalg.norm(m, axis=1)),
            np.max(np.linalg.norm(t, axis=1)),
        ]
    )


# --- Regressions (Sections III and IV) --------------------------------------------------


def rotation_regression(p: np.ndarray, q: np.ndarray) -> tuple:
    """(Phi^R, y^R) of eq. (7) from the Cayley vectors p of R_A and q of R_B of a pair
    with relative rotation, so that Phi^R x = y^R. Step 2."""
    return (skew(p + q), q - p)


def translation_regression(u: np.ndarray, d: np.ndarray) -> tuple:
    """(Phi^T, y^T) of eq. (8) from a pair with unchanged robot orientation: d is the
    robot displacement in the robot frame and u the relative marker translation in the
    camera frame, so that Phi^T x = y^T. Step 2."""
    return (skew(u + d), u - d)


def information(Phi: np.ndarray, y: np.ndarray) -> tuple:
    """(G, h) of eq. (11) from stacked regressors Phi (n, 3, 3) and outputs y (n, 3).
    Step 3."""
    G = np.einsum("nki,nkj->ij", Phi, Phi)
    h = np.einsum("nki,nk->i", Phi, y)

    return (G, h)


def height_output(
    x: np.ndarray, h: np.ndarray, p_z: np.ndarray, z: np.ndarray
) -> np.ndarray:
    """y^h of eq. (18) for stacked height observations: the known marker heights h (n,),
    the robot heights p_z (n,) and the marker positions in the camera frame z (n, 3).
    Returns (n,). Step 4."""
    return h - p_z - rotate(x, z) @ E3


def translation_information(Phi_t: np.ndarray, n_heights: int) -> np.ndarray:
    """G_t of eq. (19) from the stacked relative-pose regressors Phi_t (n, 3, 3) and the
    number of height observations. Step 4."""
    G = np.einsum("nki,nkj->ij", Phi_t, Phi_t)
    G += n_heights * np.multiply.outer(E3, E3)

    return G


def translation_rhs(
    x: np.ndarray,
    Phi_t: np.ndarray,
    t_A: np.ndarray,
    t_B: np.ndarray,
    h: np.ndarray,
    p_z: np.ndarray,
    z: np.ndarray,
) -> np.ndarray:
    """h_t(x) of eq. (20), re-evaluated from the stored raw data at the orientation x:
    Phi_t (n, 3, 3), t_A (n, 3), t_B (n, 3), and the height data as in height_output().
    Step 4."""
    y_t = rotate(x, t_B) - t_A
    h_t = np.einsum("nki,nk->i", Phi_t, y_t)
    h_t += E3 * np.sum(height_output(x, h, p_z, z))

    return h_t


def marker_orientation_observation(
    R_R: np.ndarray, x: np.ndarray, R_b: np.ndarray
) -> np.ndarray:
    """m_obs of eq. (23) with the robot rotation R(a) = R_R and the marker rotation in the
    camera frame R(b) = R_b given as matrices. Compose matrices and convert once, so the
    robot's heading may pass through pi. Step 5."""
    return rotation_cayley(R_R @ cayley_rotation(x) @ R_b)


def marker_position_observation(
    p_R: np.ndarray, R_R: np.ndarray, x: np.ndarray, t_X: np.ndarray, z: np.ndarray
) -> np.ndarray:
    """t_obs of eq. (25) from the robot position p_R and rotation R_R, the camera
    orientation x and position t_X, and the marker position z in the camera frame.
    Step 5."""
    return p_R + R_R @ (t_X + rotate(x, z))


# --- First-order covariance (Section VI) --------------------------------------------------


def cayley_rate(g: np.ndarray) -> np.ndarray:
    """C(g) with dg = C(g) w for a small rotation R(g) <- R(g) Exp(w) in the body frame:
    C(g) = (I + [g]_x + g g^T) / 2, the Cayley (Gibbs) vector kinematics."""
    return 0.5 * (I3 + skew(g) + np.outer(g, g))


def rotate_jacobian(g: np.ndarray, v: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """d(R(g) v) / dg by central differences, (3, 3) for one vector v or (n, 3, 3) for
    (n, 3) vectors, the Jacobian J_R(x, z) of eq. (30)."""
    v = np.asarray(v, float)
    J = np.zeros(v.shape + (3,))
    for k in range(3):
        d = np.zeros(3)
        d[k] = eps
        J[..., k] = (rotate(g + d, v) - rotate(g - d, v)) / (2.0 * eps)
    return J


def relative_rotation_covariances(p, q, R_B, t_Mk, u, d, noise: Noise):
    """The measurement covariances of a rotation pair (eq. (7)) and its relative-pose
    translation tuple (eq. (19)): Sigma of the Cayley vectors p (robot, from the two yaw
    readings) and q (marker, from the two marker rotations), and of the translations
    d = t_A (robot displacement) and u = t_B (marker displacement), from the paper's
    independent noise on the two observations of the pair."""
    C_p, C_q = cayley_rate(p), cayley_rate(q)
    Sig_p = 2.0 * noise.robot_yaw**2 * np.outer(C_p @ E3, C_p @ E3)
    Sig_q = 2.0 * noise.marker_rotation**2 * C_q @ C_q.T
    Sig_d, Sig_u = translation_pair_covariances(R_B, t_Mk, u, d, noise)
    return Sig_p, Sig_q, Sig_d, Sig_u


def translation_pair_covariances(R_B, t_Mk, u, d, noise: Noise):
    """Sigma of d = t_A, the robot displacement in the robot frame, and of
    u = t_B = t_Mi - R_B t_Mk, the marker displacement in the camera frame, for the two
    observations of a pair: position noise on both robot poses, the yaw noise of the
    first pose rotating d, position noise on both marker readings and the two marker
    rotation noises rotating R_B t_Mk."""
    e3_d = np.cross(E3, d)
    Sig_d = 2.0 * noise.robot_position**2 * I3 + noise.robot_yaw**2 * np.outer(e3_d, e3_d)
    v = skew(R_B @ t_Mk)
    Sig_u = 2.0 * noise.marker_position**2 * I3 + 2.0 * noise.marker_rotation**2 * v @ v.T
    return Sig_d, Sig_u


def covariance_radius(P: np.ndarray) -> float:
    """b_0.9973 of eq. (31) with a zero mean error: sqrt(chi2_{3, 0.9973} lambda_max(P))."""
    return float(math.sqrt(CHI2_3_9973 * max(np.linalg.eigvalsh(P)[-1], 0.0)))


# --- The estimator (Sections III to V and VII) ------------------------------------------


class History:
    """A bounded history of regression entries with their information. Each entry carries
    its own (G_i, h_i); push() adds them to the running sums and pop() subtracts them, so
    G() and h() cost nothing per tick. When full, push() first pops the entry whose
    removal lowers lambda_min(G) the least, so the history keeps its excitation
    (Section VII, step 2). The sums are recomputed from the entries every maxlen pops
    to clear the rounding drift of the subtractions."""

    def __init__(self, maxlen: int):
        self.maxlen = maxlen
        self.entries = []
        self.infos = []
        self.G_sum = np.zeros((3, 3))
        self.h_sum = np.zeros(3)
        self.pops = 0
        self._stacked = {}

    def __len__(self):
        return len(self.entries)

    def push(self, entry, G_i: np.ndarray, h_i: np.ndarray | None = None) -> None:
        h_i = np.zeros(3) if h_i is None else np.asarray(h_i, float)
        if len(self.entries) >= self.maxlen:
            self.pop(self.least_informative())
        self.entries.append(entry)
        self.infos.append((np.asarray(G_i, float), h_i))
        self.G_sum = self.G_sum + G_i
        self.h_sum = self.h_sum + h_i
        self._stacked.clear()

    def pop(self, i: int) -> None:
        del self.entries[i]
        G_i, h_i = self.infos.pop(i)
        self.G_sum = self.G_sum - G_i
        self.h_sum = self.h_sum - h_i
        self.pops += 1
        if self.pops % self.maxlen == 0:
            self.G_sum = sum((G for G, _ in self.infos), np.zeros((3, 3)))
            self.h_sum = sum((h for _, h in self.infos), np.zeros(3))
        self._stacked.clear()

    def least_informative(self) -> int:
        """The index whose removal lowers lambda_min(G) the least."""
        infos = np.array([G for G, _ in self.infos])
        lam = np.linalg.eigvalsh(self.G_sum[None] - infos)[:, 0]
        return int(np.argmax(lam))

    def G(self) -> np.ndarray:
        return self.G_sum.copy()

    def h(self) -> np.ndarray:
        return self.h_sum.copy()

    def stacked(self, k: int) -> np.ndarray:
        """Entry component k of every entry, stacked along a new first axis. Cached
        until the next push or pop."""
        if k not in self._stacked:
            self._stacked[k] = np.array([e[k] for e in self.entries])
        return self._stacked[k]


@dataclass
class Tick:
    """The inputs held constant while dynamics() is integrated over one tick: the
    information matrices and stored data of step 1 and 2 of the algorithm, and the
    current observations of step 5."""

    G_x: np.ndarray
    h_x: np.ndarray
    ready_x: bool
    G_t: np.ndarray
    ready_t: bool
    Phi_t: np.ndarray
    t_A: np.ndarray
    t_B: np.ndarray
    h: np.ndarray
    p_z: np.ndarray
    z_h: np.ndarray
    R_R: np.ndarray
    p_R: np.ndarray
    chi: np.ndarray
    R_b: np.ndarray
    z: np.ndarray


class Estimator:
    """The simultaneous cascaded estimator of Section VII.

    X = [x_hat (3), tX_hat (3), m_hat_1 .. m_hat_N (3 each), t_hat_1 .. t_hat_N (3 each)]
    starts at 0. update() runs steps 1 and 2 of the algorithm on the new observations,
    then integrates dynamics(), steps 3 to 5, over dt with solve_ivp while the stored data
    and the observations are held constant.

    Pairs for the regressions are formed per marker every d_step of robot travel, against
    the observations of that marker over the last l_buffer of travel: the partner with the
    largest displacement among those with a relative rotation of at most straight_tol and
    a displacement of at least d_min gives a pure-translation pair, eq. (8), and the
    partner with the largest relative rotation between psi_min and psi_max gives a rotation
    pair, eq. (7), plus a relative-pose translation tuple for eq. (19) and (20). A
    known-height observation, eq. (18), is stored at every attempt.

    Eq. (8) holds for an unchanged robot orientation, so a pure-translation pair with a
    rotation below straight_tol biases x_hat by about straight_tol |t_X| / d_min. Pairs
    with small rotations or short displacements are exact without noise but amplify
    measurement noise, which is why the largest partners are taken. On a circle or an
    ellipse the heading never repeats at a distance, so there are no pure-translation
    pairs and x stays unidentified (Section IX); the Gerono lemniscate and paths with
    straight stretches identify everything.
    """

    def __init__(
        self,
        n_markers: int,
        heights: dict,
        gamma_x: float = 2.0,
        gamma_t: float = 2.0,
        gamma_m: float = 1.0,
        gamma_j: float = 1.0,
        alpha_x: float = 0.05,
        alpha_t: float = 0.05,
        n_x: int = 120,
        n_t: int = 120,
        n_h: int = 80,
        psi_min: float = math.radians(30.0),
        psi_max: float = math.radians(150.0),
        straight_tol: float = 5e-4,
        d_min: float = 0.2,
        d_step: float = 0.1,
        l_buffer: float = 4.0,
        noise: Noise | None = None,
    ):
        self.n = n_markers
        self.heights = dict(heights)
        # The measurement noise assumed by the covariance prediction of Section VI.
        self.noise = Noise() if noise is None else noise
        self._marker_cov = {}
        self.gamma_x, self.gamma_t, self.gamma_m, self.gamma_j = (
            gamma_x,
            gamma_t,
            gamma_m,
            gamma_j,
        )
        self.alpha_x, self.alpha_t = alpha_x, alpha_t
        self.psi_min, self.psi_max = psi_min, psi_max
        self.straight_tol, self.d_min = straight_tol, d_min
        self.d_step, self.l_buffer = d_step, l_buffer
        # The bounded history buffers of Section VII: orientation regressions for G_x and
        # h_x, raw relative-pose tuples (Phi_t, t_A, t_B) for G_t, and known-height
        # observations, capped at the paper's 120, 120 and 80 entries.
        self.H_x = History(n_x)
        self.H_t = History(n_t)
        self.H_h = History(n_h)
        self.buffers = {}
        # {marker: [pure-translation pairs, rotation pairs]} formed so far, for a report
        # of how much each marker has contributed.
        self.pair_counts = {}
        self.travel = 0.0
        self.last_p = None
        self.X = np.zeros(6 + 6 * n_markers)
        self.tick = None

    # -- steps 1 and 2: histories -------------------------------------------------------

    def ingest(self, T_R: np.ndarray, observations: dict) -> None:
        """Form regressions from the new observations {j: C_T_Mj} at robot pose T_R and
        store them."""
        p_R = T_R[:3, 3]
        if self.last_p is not None:
            self.travel += float(np.linalg.norm(p_R - self.last_p))
        self.last_p = p_R
        for j, T_M in observations.items():
            buffer = self.buffers.setdefault(j, [])
            if buffer and self.travel - buffer[-1][0] < self.d_step:
                continue
            straight = rotation = None
            for _, T_Ri, T_Mi in buffer:
                A = inverse(T_Ri) @ T_R
                angle, dist = rotation_angle(A[:3, :3]), float(np.linalg.norm(A[:3, 3]))
                if (
                    angle <= self.straight_tol
                    and dist >= self.d_min
                    and (straight is None or dist > straight[0])
                ):
                    straight = (dist, T_Ri, T_Mi, A)
                # psi_max keeps the pair away from the Cayley chart's singularity at
                # pi, where tan(angle / 2) amplifies the measurement noise without bound
                if (self.psi_min <= angle <= self.psi_max
                        and (rotation is None or angle > rotation[0])):
                    rotation = (angle, T_Ri, T_Mi, A)
            # Each history entry also carries the covariances of its two measured
            # vectors for Section VI: (Phi, y, Sigma_a, Sigma_b) with the residual
            # r = (I + [x]_x) b - (I - [x]_x) a, where (a, b) is (p, q) for a rotation
            # pair and (d, u) for a pure-translation pair.
            if straight is not None:
                _, _, T_Mi, A = straight
                B = T_Mi @ inverse(T_M)
                d, u = A[:3, 3], B[:3, 3]
                Phi, y = translation_regression(u, d)
                Sig_d, Sig_u = translation_pair_covariances(B[:3, :3], T_M[:3, 3], u, d,
                                                            self.noise)
                self.H_x.push((Phi, y, Sig_d, Sig_u), *information(Phi[None], y[None]))
                self.pair_counts.setdefault(j, [0, 0])[0] += 1
            if rotation is not None:
                self.pair_counts.setdefault(j, [0, 0])[1] += 1
                _, _, T_Mi, A = rotation
                B = T_Mi @ inverse(T_M)
                p, q = rotation_cayley(A[:3, :3]), rotation_cayley(B[:3, :3])
                Phi, y = rotation_regression(p, q)
                Sig_p, Sig_q, Sig_d, Sig_u = relative_rotation_covariances(
                    p, q, B[:3, :3], T_M[:3, 3], B[:3, 3], A[:3, 3], self.noise)
                self.H_x.push((Phi, y, Sig_p, Sig_q), *information(Phi[None], y[None]))
                Phi_t = A[:3, :3] - I3
                self.H_t.push((Phi_t, A[:3, 3], B[:3, 3], Sig_d, Sig_u),
                              translation_information(Phi_t[None], 0))
            buffer.append((self.travel, T_R, T_M))
            while buffer and buffer[0][0] < self.travel - self.l_buffer:
                del buffer[0]
            if j in self.heights:
                self.H_h.push(
                    (self.heights[j], T_R[2, 3], T_M[:3, 3], j),
                    translation_information(np.zeros((0, 3, 3)), 1),
                )

    def make_tick(self, T_R: np.ndarray, observations: dict) -> Tick:
        """Everything dynamics() reads besides X."""
        G_x, h_x = self.H_x.G(), self.H_x.h()
        if len(self.H_t):
            Phi_t, t_A, t_B = (self.H_t.stacked(k) for k in range(3))
        else:
            Phi_t, t_A, t_B = np.zeros((0, 3, 3)), np.zeros((0, 3)), np.zeros((0, 3))
        if len(self.H_h):
            h, p_z, z_h = (self.H_h.stacked(k) for k in range(3))
        else:
            h, p_z, z_h = np.zeros(0), np.zeros(0), np.zeros((0, 3))
        G_t = self.H_t.G() + self.H_h.G()
        chi = np.zeros(self.n)
        R_b = np.tile(I3, (self.n, 1, 1))
        z = np.zeros((self.n, 3))
        for j, T_M in observations.items():
            chi[j], R_b[j], z[j] = 1.0, T_M[:3, :3], T_M[:3, 3]
        return Tick(
            G_x,
            h_x,
            bool(np.linalg.eigvalsh(G_x)[0] >= self.alpha_x),
            G_t,
            bool(np.linalg.eigvalsh(G_t)[0] >= self.alpha_t),
            Phi_t,
            t_A,
            t_B,
            h,
            p_z,
            z_h,
            T_R[:3, :3],
            T_R[:3, 3],
            chi,
            R_b,
            z,
        )

    # -- steps 3 to 5: the adaptive laws ------------------------------------------------

    def dynamics(self, X: np.ndarray, tick: Tick) -> np.ndarray:
        """dX/dt: eq. (14) for x_hat, eq. (21) for tX_hat, eq. (24) and (26) for the
        markers, in the layout of dynamics() on slide 46. Step 6.

        X = [x_hat (3), tX_hat (3), m_hat (3 n), t_hat (3 n)]. Run eq. (14) only while
        tick.ready_x and eq. (21) only while tick.ready_t, with zero derivative
        otherwise. tick.chi[j] is the visibility of marker j, tick.R_b[j] and tick.z[j]
        its current observation. The gains are self.gamma_x, self.gamma_t, self.gamma_m
        and self.gamma_j.
        """
        n = self.n
        x_hat = X[0:3]
        tX_hat = X[3:6]
        m_hat = X[6 : 6 + 3 * n].reshape(n, 3)
        t_hat = X[6 + 3 * n :].reshape(n, 3)

        # eq. (14): preconditioned gradient on the orientation history
        dx_hat = np.zeros(3)
        if tick.ready_x:
            dx_hat = self.gamma_x * np.linalg.solve(tick.G_x, tick.h_x - tick.G_x @ x_hat)

        # eq. (20), (21): h_t re-evaluated at the current x_hat, then the same law.
        # tick.z_h is the stored height data; tick.z is the current observations.
        dtX_hat = np.zeros(3)
        if tick.ready_t:
            h_t = translation_rhs(
                x_hat, tick.Phi_t, tick.t_A, tick.t_B, tick.h, tick.p_z, tick.z_h
            )
            dtX_hat = self.gamma_t * np.linalg.solve(tick.G_t, h_t - tick.G_t @ tX_hat)

        # eq. (23) to (26): every visible marker tracks its observation
        dm_hat = np.zeros((n, 3))
        dt_hat = np.zeros((n, 3))
        for j in range(n):
            if tick.chi[j] == 0.0:
                continue
            m_obs = marker_orientation_observation(tick.R_R, x_hat, tick.R_b[j])
            t_obs = marker_position_observation(
                tick.p_R, tick.R_R, x_hat, tX_hat, tick.z[j]
            )
            dm_hat[j] = self.gamma_m * tick.chi[j] * (m_obs - m_hat[j])
            dt_hat[j] = self.gamma_j * tick.chi[j] * (t_obs - t_hat[j])

        return np.concatenate((dx_hat, dtX_hat, dm_hat.ravel(), dt_hat.ravel()))

    def update(self, T_R: np.ndarray, observations: dict, dt: float) -> None:
        """One camera sample: robot pose W_T_R and {j: C_T_Mj} of the visible markers,
        then dt seconds of the adaptive laws."""
        self.ingest(T_R, observations)
        self.tick = self.make_tick(T_R, observations)
        if dt <= 0:
            return
        sol = solve_ivp(
            lambda t, X: self.dynamics(X, self.tick),
            (0.0, dt),
            self.X,
            rtol=1e-8,
            atol=1e-10,
        )
        self.X = sol.y[:, -1]

    def estimate(self) -> np.ndarray:
        """[x_hat, tX_hat, m_hat_1 .. m_hat_N, t_hat_1 .. t_hat_N], ordered as fields(N)."""
        return self.X.copy()

    def excitation(self) -> np.ndarray:
        """EXCITATION_FIELDS at the last update: lambda_min(G_x), lambda_min(G_t) and the
        number of stored rotation pairs, translation pairs and height observations."""
        if self.tick is None:
            lam_x = lam_t = 0.0
        else:
            lam_x = float(np.linalg.eigvalsh(self.tick.G_x)[0])
            lam_t = float(np.linalg.eigvalsh(self.tick.G_t)[0])
        return np.array([lam_x, lam_t, float(len(self.H_x)),
                         float(len(self.H_t)), float(len(self.H_h))])

    def ready(self) -> tuple:
        """(G_x >= alpha_x I, G_t >= alpha_t I) at the last update."""
        if self.tick is None:
            return False, False
        return self.tick.ready_x, self.tick.ready_t

    # -- Section VI: first-order covariance ---------------------------------------------

    def covariance(self) -> dict:
        """The first-order covariances of the estimates for self.noise, Section VI:
        "x" P_x (3, 3) of eq. (28), "tX" P_tX (3, 3) of eq. (29), "m" (n, 3, 3) and "t"
        (n, 3, 3) of the markers from eq. (30) and its orientation analogue, evaluated
        at the current estimate. An entry is None until the corresponding law has
        switched on (P_x needs G_x >= alpha_x I, the rest also G_t >= alpha_t I); a
        marker keeps the covariance of its latest observation.

        P_x = G_x^-1 (sum_l Phi_l^T S_l Phi_l) G_x^-1 with S_l from the stored
        measurement covariances. P_tX adds the propagated orientation uncertainty
        K P_x K^T, K = sum_i Phi_t,i^T dy_t,i/dx, and the run-wise height datum, which
        is common to all height entries of one marker. The marker covariances take
        the latest observation of eq. (23) and (25) through the Jacobians of eq. (30)
        with the joint covariance of [x, t_X], plus the robot and marker noise of that
        observation; the averaging of the marker laws (24) and (26) is not credited.
        """
        tick, noise = self.tick, self.noise
        n = self.n
        out = {"x": None, "tX": None,
               "m": np.full((n, 3, 3), np.nan), "t": np.full((n, 3, 3), np.nan)}
        if tick is None or not tick.ready_x or len(self.H_x) == 0:
            return out
        x, tX = self.X[0:3], self.X[3:6]
        A, B = I3 - skew(x), I3 + skew(x)
        Phi, Sig_a, Sig_b = (self.H_x.stacked(k) for k in (0, 2, 3))
        S = B @ Sig_b @ B.T + A @ Sig_a @ A.T
        G_x_inv = np.linalg.inv(tick.G_x)
        P_x = G_x_inv @ np.einsum("nki,nkl,nlj->ij", Phi, S, Phi) @ G_x_inv
        out["x"] = P_x
        if not tick.ready_t:
            return out

        # eq. (29): direct noise of the stored translation relations, then K P_x K^T
        R = cayley_rotation(x)
        M = np.zeros((3, 3))
        K = np.zeros((3, 3))
        if len(self.H_t):
            Phi_t, t_B, Sig_A, Sig_B = (self.H_t.stacked(k) for k in (0, 2, 3, 4))
            S_t = R @ Sig_B @ R.T + Sig_A
            M += np.einsum("nki,nkl,nlj->ij", Phi_t, S_t, Phi_t)
            K += np.einsum("nki,nkj->ij", Phi_t, rotate_jacobian(x, t_B))
        if len(self.H_h):
            z, who = self.H_h.stacked(2), self.H_h.stacked(3)
            per_marker = np.bincount(who.astype(int))
            direct = len(self.H_h) * (noise.robot_position**2 + noise.marker_position**2)
            datum = noise.height**2 * float(np.sum(per_marker.astype(float) ** 2))
            M += (direct + datum) * np.outer(E3, E3)
            K -= np.outer(E3, E3 @ rotate_jacobian(x, z).sum(axis=0))
        G_t_inv = np.linalg.inv(tick.G_t)
        P_tX = G_t_inv @ (M + K @ P_x @ K.T) @ G_t_inv
        out["tX"] = P_tX
        # joint covariance of [x, t_X]: t~_X = G_t^-1 (Phi_t^T eps_t + K x~)
        cross = P_x @ K.T @ G_t_inv
        P_c = np.block([[P_x, cross], [cross.T, P_tX]])

        # eq. (30) and its orientation analogue for every visible marker
        R_R, p_R = tick.R_R, tick.p_R
        for j in range(n):
            if tick.chi[j] == 0.0:
                continue
            z, R_b = tick.z[j], tick.R_b[j]
            J_c = np.hstack((R_R @ rotate_jacobian(x, z), R_R))
            w = R_R @ (tX + rotate(x, z))
            e3_w = np.cross(E3, w)
            P_t = (J_c @ P_c @ J_c.T
                   + (noise.marker_position**2 + noise.robot_position**2) * I3
                   + noise.robot_yaw**2 * np.outer(e3_w, e3_w))
            m = marker_orientation_observation(R_R, x, R_b)
            J_m = np.zeros((3, 3))
            for k in range(3):
                d = np.zeros(3)
                d[k] = 1e-6
                J_m[:, k] = (marker_orientation_observation(R_R, x + d, R_b)
                             - marker_orientation_observation(R_R, x - d, R_b)) / 2e-6
            C = cayley_rate(m)
            u = (R_R @ R @ R_b).T @ E3
            P_m = (J_m @ P_x @ J_m.T
                   + C @ (noise.marker_rotation**2 * I3
                          + noise.robot_yaw**2 * np.outer(u, u)) @ C.T)
            self._marker_cov[j] = (P_m, P_t)
        for j, (P_m, P_t) in self._marker_cov.items():
            out["m"][j], out["t"][j] = P_m, P_t
        return out

    def radii(self) -> np.ndarray:
        """RADIUS_FIELDS: the radius of eq. (31) for the four errors of ERROR_FIELDS,
        the marker ones the maximum over the markers. NaN until defined."""
        P = self.covariance()
        r = np.full(4, np.nan)
        if P["x"] is not None:
            r[0] = covariance_radius(P["x"])
        if P["tX"] is not None:
            r[1] = covariance_radius(P["tX"])
        known = [j for j in range(self.n) if np.all(np.isfinite(P["t"][j]))]
        if known:
            r[2] = max(covariance_radius(P["m"][j]) for j in known)
            r[3] = max(covariance_radius(P["t"][j]) for j in known)
        return r
