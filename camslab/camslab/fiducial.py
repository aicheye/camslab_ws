"""Simultaneous camera-extrinsic calibration and fiducial surveying under planar motion,
after Choopojcharoen and Fidan, "Simultaneous Cascaded Adaptive Camera Calibration and
Robot-Assisted Fiducial Surveying under Planar Motion" (ECC 2027). Equation numbers below
refer to that paper. See docs/fiducial-hints.md.

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


def fields(n_markers: int) -> tuple[str, ...]:
    """Names of the 6 + 6 n_markers estimated parameters, in the order of the state."""
    names = list(CAMERA_FIELDS)
    for j in range(1, n_markers + 1):
        names += [f"m{j}_{k}" for k in (1, 2, 3)]
    for j in range(1, n_markers + 1):
        names += [f"t{j}_{k}" for k in (1, 2, 3)]
    return tuple(names)


def columns(n_markers: int) -> tuple[str, ...]:
    """Sample-row columns: every estimate, every true value, then the four errors."""
    f = fields(n_markers)
    return tuple(n + "_hat" for n in f) + tuple(n + "_true" for n in f) + ERROR_FIELDS


PLOT_FIELDS = tuple(n + "_hat" for n in CAMERA_FIELDS) + tuple(n + "_true" for n in CAMERA_FIELDS)


# --- Cayley rotation coordinates (Section II.B) -----------------------------------------


def skew(v: np.ndarray) -> np.ndarray:
    """[v]_x, the matrix with [v]_x w = v x w."""
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def cayley_rotation(g: np.ndarray) -> np.ndarray:
    """R(g) = (I + [g]_x)(I - [g]_x)^-1, eq. (2). g = tan(angle / 2) axis."""
    S = skew(g)
    return (I3 + S) @ np.linalg.inv(I3 - S)


def rotation_cayley(R: np.ndarray) -> np.ndarray:
    """The Cayley vector g of a rotation matrix R, the inverse of eq. (2):
    [g]_x = (R - I)(R + I)^-1. Undefined at a rotation by pi."""
    S = (R - I3) @ np.linalg.inv(R + I3)
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def compose(g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
    """g1 (+) g2, the Cayley vector of R(g1) R(g2), eq. (3)."""
    return (g1 + g2 + np.cross(g1, g2)) / (1.0 - g1 @ g2)


def rotate(g: np.ndarray, v: np.ndarray) -> np.ndarray:
    """R(g, v) = R(g) v without building the matrix, eq. (4). v may be one vector or an
    (n, 3) array of vectors."""
    v = np.asarray(v, float)
    gg = g @ g
    return ((1.0 - gg) * v + 2.0 * np.multiply.outer(v @ g, g) + 2.0 * np.cross(g, v)) / (1.0 + gg)


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
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx


def robot_pose(state: State, z: float = 0.0) -> np.ndarray:
    """W_T_R of a planar robot state: heading q about the world z axis at height z."""
    R = np.array([[state.q[0], -state.q[1], 0.0], [state.q[1], state.q[0], 0.0],
                  [0.0, 0.0, 1.0]])
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
    return [Marker((2.0, 1.0, 0.0), (0.0, 0.0, 0.8), True),
            Marker((-1.5, 2.2, 0.0), (0.3, 0.0, -1.2), True),
            Marker((1.0, -2.0, 0.4), (0.0, 0.4, 2.0), False)]


def lab_markers() -> list:
    """Three fiducials inside the 6 m x 6 m lab area, around the default paths."""
    return [Marker((4.5, 1.0, 0.0), (0.0, 0.0, 2.5), True),
            Marker((0.5, 3.0, 0.0), (0.3, 0.0, -0.5), True),
            Marker((3.0, 4.0, 0.4), (0.0, 0.4, -1.8), False)]


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
        return pose(cayley_rotation(np.asarray(self.camera_cayley, float)),
                    self.camera_position)

    def marker_poses(self) -> list:
        """Y_j = W_T_Mj for every marker."""
        return [m.pose() for m in self.markers]

    def known_heights(self) -> dict:
        """{j: h_j} for the markers whose world-frame height is known, j from 0."""
        return {j: float(m.position[2]) for j, m in enumerate(self.markers) if m.known_height}

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
        return np.concatenate([np.asarray(self.camera_cayley, float),
                               np.asarray(self.camera_position, float),
                               *[rotation_cayley(Y[:3, :3]) for Y in Ys],
                               *[Y[:3, 3] for Y in Ys]])


def errors(estimate: np.ndarray, truth: np.ndarray, n_markers: int) -> np.ndarray:
    """ERROR_FIELDS: ||x~||, ||t~_X||, max_j ||m~_j||, max_j ||t~_j||."""
    e = estimate - truth
    m = e[6:6 + 3 * n_markers].reshape(n_markers, 3)
    t = e[6 + 3 * n_markers:].reshape(n_markers, 3)
    return np.array([np.linalg.norm(e[0:3]), np.linalg.norm(e[3:6]),
                     np.max(np.linalg.norm(m, axis=1)), np.max(np.linalg.norm(t, axis=1))])


# --- Regressions (Sections III and IV) --------------------------------------------------


def rotation_regression(p: np.ndarray, q: np.ndarray) -> tuple:
    """(Phi^R, y^R) of eq. (7) from the Cayley vectors p of R_A and q of R_B of a pair
    with relative rotation: [p + q]_x x = q - p."""
    return skew(p + q), q - p


def translation_regression(u: np.ndarray, d: np.ndarray) -> tuple:
    """(Phi^T, y^T) of eq. (8) from a pair with unchanged robot orientation: d is the
    robot displacement in the robot frame and u the relative marker translation in the
    camera frame, d = R(x) u, so [u + d]_x x = u - d."""
    return skew(u + d), u - d


def information(Phi: np.ndarray, y: np.ndarray) -> tuple:
    """(G, h) of eq. (11) from stacked regressors Phi (n, 3, 3) and outputs y (n, 3)."""
    G = np.einsum("nki,nkj->ij", Phi, Phi)
    h = np.einsum("nki,nk->i", Phi, y)
    return G, h


def height_output(x: np.ndarray, h: np.ndarray, p_z: np.ndarray, z: np.ndarray) -> np.ndarray:
    """y^h of eq. (18) for stacked height observations: the known marker heights h (n,),
    the robot heights p_z (n,) and the marker positions in the camera frame z (n, 3).
    y^h = h - p_z - e_3^T R(x) z = e_3^T t_X."""
    return h - p_z - rotate(x, z)[:, 2]


def translation_information(Phi_t: np.ndarray, n_heights: int) -> np.ndarray:
    """G_t of eq. (19) from the stacked relative-pose regressors Phi_t (n, 3, 3) and the
    number of height observations."""
    return np.einsum("nki,nkj->ij", Phi_t, Phi_t) + n_heights * np.outer(E3, E3)


def translation_rhs(x: np.ndarray, Phi_t: np.ndarray, t_A: np.ndarray, t_B: np.ndarray,
                    h: np.ndarray, p_z: np.ndarray, z: np.ndarray) -> np.ndarray:
    """h_t(x) of eq. (20), re-evaluated from the stored raw data at the orientation x."""
    y_t = rotate(x, t_B) - t_A
    return np.einsum("nki,nk->i", Phi_t, y_t) + E3 * np.sum(height_output(x, h, p_z, z))


def marker_orientation_observation(R_R: np.ndarray, x: np.ndarray, R_b: np.ndarray) -> np.ndarray:
    """m_obs of eq. (23): the Cayley vector of R(a) R(x) R(b), with the robot rotation
    R(a) = R_R and the marker rotation in the camera frame R(b) = R_b given as matrices,
    so the robot's heading may pass through pi."""
    return rotation_cayley(R_R @ cayley_rotation(x) @ R_b)


def marker_position_observation(p_R: np.ndarray, R_R: np.ndarray, x: np.ndarray,
                                t_X: np.ndarray, z: np.ndarray) -> np.ndarray:
    """t_obs of eq. (25): p_R + R(a) (t_X + R(x) z)."""
    return p_R + R_R @ (t_X + rotate(x, z))


# --- The estimator (Sections III to V and VII) ------------------------------------------


class History:
    """A bounded history of regression entries with their 3x3 information matrices. When
    full, the entry whose removal lowers lambda_min(G) the least is replaced, so the
    history keeps its excitation (Section VII, step 2)."""

    def __init__(self, maxlen: int):
        self.maxlen = maxlen
        self.entries = []
        self.infos = []

    def __len__(self):
        return len(self.entries)

    def push(self, entry, info: np.ndarray) -> None:
        if len(self.entries) >= self.maxlen:
            infos = np.array(self.infos)
            G = infos.sum(axis=0)
            lam = np.linalg.eigvalsh(G[None] - infos)[:, 0]
            i = int(np.argmax(lam))
            del self.entries[i], self.infos[i]
        self.entries.append(entry)
        self.infos.append(info)

    def G(self) -> np.ndarray:
        return np.sum(self.infos, axis=0) if self.infos else np.zeros((3, 3))

    def stacked(self, k: int) -> np.ndarray:
        """Entry component k of every entry, stacked along a new first axis."""
        return np.array([e[k] for e in self.entries])


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
    partner with the largest relative rotation, if at least psi_min, gives a rotation
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

    def __init__(self, n_markers: int, heights: dict, gamma_x: float = 2.0,
                 gamma_t: float = 2.0, gamma_m: float = 1.0, gamma_j: float = 1.0,
                 alpha_x: float = 0.05, alpha_t: float = 0.05, n_orientation: int = 120,
                 n_translation: int = 120, n_height: int = 80,
                 psi_min: float = math.radians(30.0), straight_tol: float = 5e-4,
                 d_min: float = 0.2, d_step: float = 0.1, l_buffer: float = 4.0):
        self.n = n_markers
        self.heights = dict(heights)
        self.gamma_x, self.gamma_t, self.gamma_m, self.gamma_j = gamma_x, gamma_t, gamma_m, gamma_j
        self.alpha_x, self.alpha_t = alpha_x, alpha_t
        self.psi_min, self.straight_tol, self.d_min = psi_min, straight_tol, d_min
        self.d_step, self.l_buffer = d_step, l_buffer
        self.orientation = History(n_orientation)
        self.translation = History(n_translation)
        self.height = History(n_height)
        self.buffers = {}
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
                if angle <= self.straight_tol and dist >= self.d_min and (
                        straight is None or dist > straight[0]):
                    straight = (dist, T_Ri, T_Mi, A)
                if angle >= self.psi_min and (rotation is None or angle > rotation[0]):
                    rotation = (angle, T_Ri, T_Mi, A)
            if straight is not None:
                _, _, T_Mi, A = straight
                B = T_Mi @ inverse(T_M)
                Phi, y = translation_regression(B[:3, 3], A[:3, 3])
                self.orientation.push((Phi, y), Phi.T @ Phi)
            if rotation is not None:
                _, _, T_Mi, A = rotation
                B = T_Mi @ inverse(T_M)
                Phi, y = rotation_regression(rotation_cayley(A[:3, :3]),
                                             rotation_cayley(B[:3, :3]))
                self.orientation.push((Phi, y), Phi.T @ Phi)
                Phi_t = A[:3, :3] - I3
                self.translation.push((Phi_t, A[:3, 3], B[:3, 3]), Phi_t.T @ Phi_t)
            buffer.append((self.travel, T_R, T_M))
            while buffer and buffer[0][0] < self.travel - self.l_buffer:
                del buffer[0]
            if j in self.heights:
                self.height.push((self.heights[j], T_R[2, 3], T_M[:3, 3]), np.outer(E3, E3))

    def make_tick(self, T_R: np.ndarray, observations: dict) -> Tick:
        """Everything dynamics() reads besides X."""
        if len(self.orientation):
            G_x, h_x = information(self.orientation.stacked(0), self.orientation.stacked(1))
        else:
            G_x, h_x = np.zeros((3, 3)), np.zeros(3)
        if len(self.translation):
            Phi_t, t_A, t_B = (self.translation.stacked(k) for k in range(3))
        else:
            Phi_t, t_A, t_B = np.zeros((0, 3, 3)), np.zeros((0, 3)), np.zeros((0, 3))
        if len(self.height):
            h, p_z, z_h = (self.height.stacked(k) for k in range(3))
        else:
            h, p_z, z_h = np.zeros(0), np.zeros(0), np.zeros((0, 3))
        G_t = translation_information(Phi_t, len(self.height))
        chi = np.zeros(self.n)
        R_b = np.tile(I3, (self.n, 1, 1))
        z = np.zeros((self.n, 3))
        for j, T_M in observations.items():
            chi[j], R_b[j], z[j] = 1.0, T_M[:3, :3], T_M[:3, 3]
        return Tick(G_x, h_x, bool(np.linalg.eigvalsh(G_x)[0] >= self.alpha_x),
                    G_t, bool(np.linalg.eigvalsh(G_t)[0] >= self.alpha_t),
                    Phi_t, t_A, t_B, h, p_z, z_h, T_R[:3, :3], T_R[:3, 3], chi, R_b, z)

    # -- steps 3 to 5: the adaptive laws ------------------------------------------------

    def dynamics(self, X: np.ndarray, tick: Tick) -> np.ndarray:
        """dX/dt: eq. (14) for x_hat, eq. (21) for tX_hat, eq. (24) and (26) for the
        markers, in the layout of dynamics() on slide 46."""
        n = self.n
        x_hat = X[0:3]
        tX_hat = X[3:6]
        m_hat = X[6:6 + 3 * n].reshape(n, 3)
        t_hat = X[6 + 3 * n:].reshape(n, 3)

        dx_hat = np.zeros(3)
        if tick.ready_x:
            dx_hat = self.gamma_x * np.linalg.solve(tick.G_x, tick.h_x - tick.G_x @ x_hat)

        dtX_hat = np.zeros(3)
        if tick.ready_t:
            h_t = translation_rhs(x_hat, tick.Phi_t, tick.t_A, tick.t_B, tick.h, tick.p_z,
                                  tick.z_h)
            dtX_hat = self.gamma_t * np.linalg.solve(tick.G_t, h_t - tick.G_t @ tX_hat)

        dm_hat = np.zeros((n, 3))
        dt_hat = np.zeros((n, 3))
        for j in np.flatnonzero(tick.chi):
            m_obs = marker_orientation_observation(tick.R_R, x_hat, tick.R_b[j])
            t_obs = marker_position_observation(tick.p_R, tick.R_R, x_hat, tX_hat, tick.z[j])
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
        sol = solve_ivp(lambda t, X: self.dynamics(X, self.tick), (0.0, dt), self.X,
                        rtol=1e-8, atol=1e-10)
        self.X = sol.y[:, -1]

    def estimate(self) -> np.ndarray:
        """[x_hat, tX_hat, m_hat_1 .. m_hat_N, t_hat_1 .. t_hat_N], ordered as fields(N)."""
        return self.X.copy()

    def ready(self) -> tuple:
        """(G_x >= alpha_x I, G_t >= alpha_t I) at the last update."""
        if self.tick is None:
            return False, False
        return self.tick.ready_x, self.tick.ready_t
