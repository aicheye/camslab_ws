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

## Adaptive estimator

`camslab/camslab/adaptive.py` holds the online path estimator that both simulators run next
to the tracking controller. It fits a circle `theta = [r, c_x, c_y]` to the path from the
robot position `p` and the signed distance `xi` to the path, with the repeated-pole filter
and the gradient law from the estimation slides. It runs on every shape.

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

The estimator only observes: the controller tracks the true trajectory, and the estimate
does not change the commands. In this open-loop setup (slide 38) the circle shape gives the
centre but not the radius, because tracking keeps `xi` near zero, so `xi'` carries no
information about `r`.

Offline simulator:

```bash
uv run python -m camslab_offline.tracking --adaptive --csv run.csv
```

The web UI has an **Adaptive estimator** box with a checkbox that turns it on, and
one plot per estimated parameter with the true value as a dashed line. The CSV export adds
one `<name>_hat` column per parameter.

Gazebo:

```bash
ros2 launch camslab gazebo_multi.launch.py adaptive:=true
```

or set `adaptive: true` in `config/sim.yaml`. Each robot then also runs
`estimator` (`camslab/camslab/estimator_node.py`), which subscribes to `pose` and
publishes `estimate` (`std_msgs/Float64MultiArray`, the six `COLUMNS`). The web UI shows the estimates of the first robot and resets the estimator on **Start**.

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
