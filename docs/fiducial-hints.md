# Camera calibration and fiducial surveying

This implements Choopojcharoen and Fidan, *Simultaneous Cascaded Adaptive Camera
Calibration and Robot-Assisted Fiducial Surveying under Planar Motion* (ECC 2027), in the
simulators. Equation numbers below are the paper's.

A camera is mounted on the car with an unknown extrinsic transform `X = R_T_C`. Fiducial
markers (think ArUco tags) stand in the lab at unknown world poses `Y_j = W_T_Mj`. The car's
world pose `W_T_R` is known from the simulator or the motion capture. The camera returns
the relative pose `C_T_Mj` of every marker it sees. The estimator identifies `X` and every
`Y_j` while the car drives its trajectory, in one run and all at the same time.

No images are simulated. `Scene.observe()` plays the camera: it solves eq. (1) for `C_T_Mj`
from the true poses and reports the markers inside the field of view.

`camslab/camslab/fiducial.py` holds the sensor, the data handling and the estimator loop.
The mathematics of the paper is left as stubs that raise `NotImplementedError`. Each step
below names the stubs and the tests that check them. The tests run from `camslab_standalone/`:

    uv run pytest tests/fiducial -x          # stops at the first failing step
    uv run pytest tests/fiducial tests/tracking/test_fiducial_sim.py
    uv run python -m camslab_standalone.fiducial            # the paper's study, Fig. 1
    uv run python -m camslab_standalone.fiducial --runs 20  # with the paper's noise
    uv run python -m camslab_standalone.tracking --adaptive fiducial --shape bowtie

In Gazebo, set `adaptive: fiducial` in `camslab/config/sim.yaml` or pass
`adaptive:=fiducial`. RViz shows the true markers and camera in grey and the estimates in
green, with the four errors of the paper's Fig. 1 as text above the car. The web UI plots
those four errors, `||x~||`, `||t~_X||`, `max_j ||m~_j||` and `max_j ||t~_j||`, on a log
axis as in Fig. 1, and draws the marker estimates on the map. The camera transform
accuracy is the pair of camera curves; the plot goes to zero without noise. A second
plot shows the excitation: `lambda_min(G_x)` and `lambda_min(G_t)` of eqs. (11) and (19)
against the gates `alpha_x` and `alpha_t`. The camera laws switch on where a curve crosses
its dashed line, and the tooltip counts the stored rotation pairs, translation pairs and
height observations.

## Frames and notation

| Symbol | Code | Meaning |
|---|---|---|
| `A_T_B` | 4x4 array | pose of frame B in frame A, maps B coordinates to A coordinates |
| `W_T_R` | `T_R`, from `robot_pose(state)` | the car, x forward and z up, at the rear-axle centre |
| `X = R_T_C` | `Scene.camera()` | camera extrinsic: Cayley vector `x`, position `t_X` |
| `Y_j = W_T_Mj` | `Marker.pose()` | marker j: Cayley vector `m_j`, position `t_j` |
| `C_T_Mj` | `T_M`, from `Scene.observe()` | the measurement: rotation `R_b` with Cayley vector `b`, position `z` |
| `g` | 3-vector | Cayley/Gibbs vector, `g = tan(angle / 2) axis`, eq. (2) |

The estimate vector is `[x, t_X, m_1 .. m_N, t_1 .. t_N]`, the `6 + 6 N` states of the
paper, named by `fields(N)`. Everything starts at zero.

## Step 1: Cayley rotation coordinates

Stubs: `cayley_rotation(g)`, eq. (2); `compose(g1, g2)`, eq. (3); `rotate(g, v)`, eq. (4).
`skew(v)` is given. `rotate()` must accept one vector or an `(n, 3)` array of vectors,
one per row, because the stored data are evaluated in batches.

Tests: `test_cayley_rotation_*`, `test_rotation_cayley_*`, `test_compose_*`,
`test_rotate_*`. They compare with `scipy.spatial.transform.Rotation`, so check that the
rotation angle of `R(g)` is `2 atan(|g|)` about `g`.

## Step 2: orientation regressions

Stubs: `rotation_regression(p, q)`, eq. (7); `translation_regression(u, d)`, eq. (8).
Both return `(Phi, y)` with `Phi x = y` for the true `x`.

Equation (7) follows from the rotation part of eq. (5): `R(p) R(x) = R(x) R(q)`, with
`p = R(x) q`. Use eq. (3) on both sides. Equation (8) follows from the translation part of
eq. (5) with `R_A = I`: `d = R(x) u`. Write `R(x)` as in eq. (2) and multiply by
`I - [x]_x`.

Tests: `test_rotation_regression_*`, `test_translation_regression_*`,
`test_yaw_and_straight_pairs_together_identify_x`. The last one shows why both pair types
are needed: every yaw-only pair shares one unexcited direction.

## Step 3: information accumulation

Stub: `information(Phi, y)`, eq. (11), for stacked regressors `Phi` of shape `(n, 3, 3)`
and outputs `y` of shape `(n, 3)`. Return `(G_x, h_x)`. `np.einsum` does this in one line
each; a loop is fine too. The estimator calls it with `n = 1` when it stores a pair and
adds the result to the running sums of the history, so `G_x` and `h_x` are never
recomputed from the whole history; `translation_information()` of step 4 is used the
same way for `G_t`.

Test: `test_information_is_the_sum_of_normal_equations`.

## Step 4: camera translation

Stubs: `height_output(x, h, p_z, z)`, eq. (18), for stacked height observations;
`translation_information(Phi_t, n_heights)`, eq. (19); `translation_rhs(x, Phi_t, t_A,
t_B, h, p_z, z)`, eq. (20).

For eq. (18), take the z row of eq. (1) for a planar robot, where `e_3^T R_R = e_3^T`.
The result is the camera height `e_3^T t_X`, so one known marker height removes the axial
nullspace of the planar relative-pose regressions. Note that `translation_rhs()` is
evaluated again at every call with the current `x_hat`, which is why the raw tuples are
stored rather than the regression outputs.

Tests: `test_height_output_*`, `test_translation_information_and_rhs_*`.

## Step 5: marker observations

Stubs: `marker_orientation_observation(R_R, x, R_b)`, eq. (23);
`marker_position_observation(p_R, R_R, x, t_X, z)`, eq. (25).

Equation (23) composes three Cayley vectors. The robot orientation `a` and the marker
orientation in the camera frame `b` are given here as rotation matrices `R_R` and `R_b`,
so compose with matrices and convert the product once with `rotation_cayley()`. The
reason is Assumption 1: the Cayley chart is singular at a rotation by pi, and the car's
heading and a marker's orientation relative to the camera both pass through pi on a lap,
while the composed world orientation of a marker does not.

Test: `test_marker_observations_are_exact_with_the_true_extrinsic`.

## Step 6: the adaptive laws

Stub: `Estimator.dynamics(X, tick)`. Follow the layout of `dynamics()` on slide 46 of the
estimation deck, as in `camslab.adaptive`:

1. Extract `x_hat`, `tX_hat`, `m_hat` (N by 3) and `t_hat` (N by 3) from `X`.
2. Eq. (14) for `x_hat`, only while `tick.ready_x` (that is `G_x >= alpha_x I`),
   otherwise zero. `tick.G_x` and `tick.h_x` are the sums of step 3.
3. Eq. (20) and (21) for `tX_hat`, only while `tick.ready_t`. `tick.Phi_t`, `tick.t_A`,
   `tick.t_B`, `tick.h`, `tick.p_z` and `tick.z_h` are the stored raw data.
4. Eq. (23) to (26) for every marker. `tick.chi[j]` is the visibility, `tick.R_b[j]` and
   `tick.z[j]` the current observation, `tick.R_R` and `tick.p_R` the robot pose.
5. Concatenate the derivatives in the order of `X`.

`G^-1 (h - G theta)` is a linear solve; use `np.linalg.solve`. The gains are
`self.gamma_x`, `self.gamma_t`, `self.gamma_m` and `self.gamma_j`.

Tests: `test_study_converges_without_noise` (every error below 1e-6 after the paper's
35 s), `test_a_circle_leaves_the_orientation_unidentified`, and the closed-loop tests in
`tests/tracking/test_fiducial_sim.py`.

## What is given

- `Scene`: the truth and the sensor. `Scene.truth()` is only used for the error columns
  and the dashed lines.
- `History`: a bounded history with running sums of its entries' `(G_i, h_i)`: `push()`
  adds, `pop()` subtracts, and when full `push()` first pops the entry whose removal
  lowers `lambda_min(G)` the least, Section VII step 2.
- `Estimator.ingest()`: pair selection. Every `d_step` of travel, each visible marker is
  paired with its earlier observations over the last `l_buffer` of travel: the farthest
  partner with a relative rotation below `straight_tol` gives a pure-translation pair,
  and the partner with the largest rotation above `psi_min` gives a rotation pair and a
  translation tuple. Known heights are stored at the same time.
- `Estimator.make_tick()`: the information matrices, the readiness gates and the current
  observations, held constant over one tick.
- `Estimator.update()`: `ingest()`, `make_tick()`, then `solve_ivp` over `dynamics()`.

## Experiments

- The default web UI path is the circle. Run the estimator on it and read `e_x`: the
  orientation never becomes ready, because the car never repeats a heading at a distance,
  so there are no pure-translation pairs. The Gerono lemniscate has them around its
  crossing. A spline with four or more collinear control points in a row has exact
  straight stretches. The `bowtie` shape, a double triangle, is the closest to the
  paper's study: straight edges and corners that turn by more than 90 degrees to the
  right and to the left.
- Set `fov` in `Scene` or `config/fiducial.yaml` to 2.1 rad (120 degrees) and `range` to
  5 m. The markers come and go, and on the Gerono no marker is seen at equal headings
  on both sides of the crossing, so `x` stays unidentified; the rounded-rectangle spline
  still works.
- Run the study with `--runs 20` and compare the terminal errors with Table I of the
  paper. Lower `psi_min` to 5 degrees and `d_min` to 0.1 m: exact without noise, much
  worse with it, because the regressions divide small differences.
- Set `straight_tol` to 0.01 and watch `e_x` settle at about `straight_tol |t_X| / d_min`
  instead of zero. Equation (8) holds only for an unchanged heading.
- Remove the known heights (`known_height: false` on every marker) and check which
  component of `t_X` is lost.

## Reading

- The paper, Sections II to V and VII. Section VI, the covariance analysis, is not
  implemented.
- Ioannou and Fidan, *Adaptive Control Tutorial*, chapter 3, for the gradient law with a
  preconditioner, eq. (14).
