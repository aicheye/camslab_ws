# Developer reference

Use this page for package layout, simulation internals, and ROS interfaces. Start with
{doc}`install` and {doc}`running` for the normal user workflow.

## Package layout

| Package or directory | Purpose |
|---|---|
| `camslab/` | Python ROS 2 package: the shared library (`types`, `trajectory`, `controller`, `safety`, `adaptive`), the nodes `controller`, `pose_tf` and `estimator`, launch files, Gazebo model, and RViz config |
| `camslab_webui/` | shared web UI backend, frontend, and safety-polytope logic |
| `camslab_offline/` | offline experiments without ROS, one subpackage each, with shared code in `common/` |
| `docs/` | user and developer documentation |

`camslab_offline/` contains `COLCON_IGNORE` and is not built by colcon. It is a uv project
that imports `camslab` and `camslab_webui` from the repository, so the offline experiments
and the ROS nodes run the same control law, trajectories, and estimators.

| Experiment | Command, from `camslab_offline/` | Tests |
|---|---|---|
| Trajectory tracking with the web UI | `uv run python -m camslab_offline.tracking` | `tests/tracking` |
| Mass-spring-damper estimation | `uv run python -m camslab_offline.msd` | `tests/msd` |
| Circular path identification | `uv run python -m camslab_offline.path_circle` | `tests/path_circle` |
| Fiducial surveying study | `uv run python -m camslab_offline.fiducial` | `tests/fiducial` |

`common/unicycle.py` holds the unicycle plant and the command clamp.

## Offline simulator

Run the tracking simulator from `camslab_offline/`:

```bash
uv run python -m camslab_offline.tracking
uv run python -m camslab_offline.tracking --shape gerono
uv run python -m camslab_offline.tracking --csv run.csv
uv run pytest
```

Use `--initial X Y THETA_DEG` or `--seed N` for repeatable starts. The fixed-step loop
evaluates the reference, samples and clamps the command, holds it over `dt`, and calls
`step()`. The default `dt = 0.02 s` matches the ROS controller's 50 Hz rate.

Tests cover the plant, trajectory derivatives, reference generation, limit checks, and
tracking. Rectangle tracking is excluded because its instantaneous 90-degree corners
cannot be followed by the car.

## Gazebo nodes and topics

`gazebo_multi.launch.py` starts Gazebo, a clock bridge, one `robot.launch.py` group per
fleet entry, RViz, and the web UI. Each robot runs in namespace `<name>`:

| Node | Subscribes | Publishes |
|---|---|---|
| `controller` | `pose` | `cmd_vel`, `ref_pose`, `ref_path`, `safety_polytope` |
| `parameter_bridge` | Gazebo pose, joint states, `cmd_vel` | `pose`, `joint_states`, Gazebo `cmd_vel` |
| `robot_state_publisher` | `joint_states`, robot xacro | robot TF |
| `pose_tf` | `pose` | `map` to `base_footprint` TF |

The controller clamps `v` and `w`, commands zero for stale pose data, and exposes
`~/enable` as `std_srvs/srv/SetBool`. Gazebo nodes use `/clock` and `use_sim_time`.

The R2 model uses an Ackermann front axle with a 0.235 m wheelbase and a 0.6 rad steering
limit. The tracked point is `base_footprint`, the centre of the rear axle.

## Adaptive estimators

Both simulators can run one adaptive estimator next to the tracking controller. The
estimator only observes: the controller tracks the true trajectory, and the estimate does
not change the commands. Two estimators exist.

| Kind | Module | Estimates |
|---|---|---|
| `path` | `camslab/camslab/adaptive.py` | the circle `[r, c_x, c_y]` that fits the path near the robot |
| `fiducial` | `camslab/camslab/fiducial.py` | the camera extrinsic `(x, t_X)` and the world pose `(m_j, t_j)` of every fiducial marker |

Offline simulator:

```bash
uv run python -m camslab_offline.tracking --adaptive path --csv run.csv
uv run python -m camslab_offline.tracking --adaptive fiducial --shape gerono --csv run.csv
```

The web UI has an **Adaptive estimator** box with a selector for the kind, and a plot of
the estimated parameters with the true values dashed. The CSV export adds the estimator's
columns.

Gazebo:

```bash
ros2 launch camslab gazebo_multi.launch.py adaptive:=fiducial
```

or set `adaptive: path` or `adaptive: fiducial` in `config/sim.yaml`. Each robot then also
runs `estimator` (`camslab/camslab/estimator_node.py`), which subscribes to `pose` and
publishes `estimate` (`std_msgs/Float64MultiArray`, labelled with the column names). The
web UI shows the estimates of the first robot and resets the estimator on **Start**.

### Path estimator

`camslab.adaptive` fits a circle `theta = [r, c_x, c_y]` to the path from the robot
position `p` and the signed distance `xi` to the path, with the repeated-pole filter and
the gradient law from the estimation slides. It runs on every shape.

| Name | Purpose |
|---|---|
| `Estimator.dynamics(X, p, xi)` | `dX/dt` of the filters and the gradient law, in the layout of `dynamics()` on slide 46 |
| `Estimator.update(p, xi, dt)` | Integrates `dynamics()` over one control tick with `solve_ivp`, holding `p` and `xi` constant. The filter states start at 0 |
| `Estimator.estimate()` | Returns `theta_hat`, ordered as `FIELDS` |
| `Path.xi(p)` | The sensor: distance from `p` to one sampled lap of the true path, positive outside and negative inside |
| `Path.truth(p)` | The true value, for plots only: the osculating circle at the point of the path closest to `p`, `r = 1 / abs(kappa*)` and `c = p* + n / kappa*`. NaN on straight parts, where the circle does not exist |

On the circle shape the true value is the circle itself. On the other shapes it changes as
the closest point moves along the path. Each sample row ends with the `COLUMNS`
`r_hat, c_x_hat, c_y_hat, r_true, c_x_true, c_y_true`, and the estimate plot draws the
`_true` columns dashed in the colour of the matching estimate.

In this open-loop setup (slide 38) the circle shape gives the centre but not the radius,
because tracking keeps `xi` near zero, so `xi'` carries no information about `r`.

### Fiducial surveying estimator

`camslab.fiducial` implements Choopojcharoen and Fidan, *Simultaneous Cascaded Adaptive
Camera Calibration and Robot-Assisted Fiducial Surveying under Planar Motion* (ECC 2027).
A camera with unknown extrinsic `X = R_T_C` is mounted on the robot, and fiducial markers
with unknown world poses `Y_j = W_T_Mj` stand in the lab. The robot pose is known. The
camera is simulated as a pose sensor: `Scene.observe()` returns the relative pose `C_T_Mj`
of every marker inside the field of view, without any image. The estimator is written
step by step in {doc}`fiducial-hints` (`docs/fiducial-hints.md`).

| Name | Purpose |
|---|---|
| `Scene` | The truth: camera Cayley vector and position, the markers, `fov` and `range`. `observe(W_T_R)` is the sensor, `truth()` the true parameter vector |
| `Estimator(n_markers, heights)` | The cascaded estimator. `update(W_T_R, observations, dt)` stores the new regressions and integrates `dynamics()` over one tick with `solve_ivp` |
| `Estimator.estimate()` | `[x, t_X, m_1 .. m_N, t_1 .. t_N]`, ordered as `fields(N)` |
| `Estimator.ready()` | whether the orientation and translation histories are full rank, so their laws run |
| `columns(N)` | sample-row columns: every estimate, every true value, then `e_x`, `e_tX`, `e_m`, `e_t`, the error norms of Fig. 1 of the paper |

Only `fields` from `x_1` to `tX_3` are plotted; the map draws each marker's true position
as a grey square and its estimate as a dashed green square. In Gazebo the node also
publishes `fiducials` (`visualization_msgs/MarkerArray`): the true markers and camera in
grey, the estimates in green, shown in RViz.

The scene for Gazebo is in `camslab/config/fiducial.yaml`, with the estimator gains and
the pair-selection thresholds. The offline simulator uses the same default scene.

The camera orientation needs both relative-yaw pairs and pure-translation pairs of
observations. Trajectories with straight stretches identify everything: the Gerono
lemniscate, and a spline with runs of collinear control points. On a circle or an ellipse
the robot never drives straight, so `x` stays at its initial value and the downstream
estimates keep the resulting bias; that is the planar-motion limitation discussed in the
paper.

`python -m camslab_offline.fiducial` replays the paper's simulation study: 35 s of
alternating straight segments and turns at 20 Hz, with the paper's camera and markers.
`--runs N` adds the paper's measurement noise and reports the root-mean-square error over
N runs. The covariance radii of the paper are not computed.

## Fleet behavior

Add robots to `config/sim.yaml` and give each a unique lowercase name. The launch file
creates one namespace, Gazebo model, controller, and TF prefix per entry. Every robot
currently receives the same trajectory parameters and reference, so multiple robots can
collide. Implement separate references before using a fleet for coordination work.

The launch file sets `IGN_PARTITION=camslab_<ROS_DOMAIN_ID>` unless `IGN_PARTITION` is
already set. This keeps simulations in different ROS domains separate on one network.

## Safety polytope implementation

`camslab_webui/polytope.py` computes the counter-clockwise convex hull and writes the
parameter file. `camslab/camslab/safety.py` checks the sampled reference path, refuses enable while
the robot is outside, and disables the controller after a fresh pose outside the polygon.

The polygon applies to the rear-axle centre without a robot-size margin. Keep additional
clearance for the car body when drawing the polygon.

## Web UI protocol

The backend serves JSON over WebSocket at `/ws`. The implementation is in
`camslab_webui/camslab_webui/server.py`. The main message types are:

| Direction | Message |
|---|---|
| server → UI | `hello`, `config`, `samples`, `fleet`, `status` |
| UI → server | `set_initial`, `set_params`, `randomize_trajectory`, `set_polytope`, `start`, `stop`, `reset` |

The live fleet stream sends samples at 25 Hz. CSV fields are `t`, `x`, `y`, `qx`, `qy`,
`v`, `w`, `dist`, `xr`, and `yr`.

## Add a controller feature

1. Change the relevant type in `camslab/camslab/types.py`.
2. Add ROS parameters or subscriptions in `camslab/camslab/controller_node.py`.
3. Keep command saturation, stale-pose handling, and safety-polytope checks outside the
   experimental control law.
4. Add or update tests before changing the launch configuration.
5. Run `uv run pytest` in `camslab_offline/`. With `colcon build --symlink-install`, a
   Python change needs no rebuild.
