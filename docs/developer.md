# Developer reference

Use this page for package layout, simulation internals, and ROS interfaces. Start with
{doc}`install` and {doc}`running` for the normal user workflow.

## Package layout

| Package or directory | Purpose |
|---|---|
| `camslab/` | C++ controller, trajectories, launch files, Gazebo model, and RViz config |
| `camslab_sim/` | standalone Python plant, trajectories, controller, and offline web UI |
| `camslab_webui/` | shared web UI backend, frontend, and safety-polytope logic |
| `camslab_estimation/` | standalone mass-spring-damper estimation exercise |
| `docs/` | user and developer documentation |

`camslab_sim/` and `camslab_estimation/` contain `COLCON_IGNORE` and are not built by
colcon.

## Offline simulator

Run the simulator from `camslab_sim/`:

```bash
uv run python -m camslab_sim
uv run python -m camslab_sim --shape gerono
uv run python -m camslab_sim --csv run.csv
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

## Fleet behavior

Add robots to `config/sim.yaml` and give each a unique lowercase name. The launch file
creates one namespace, Gazebo model, controller, and TF prefix per entry. Every robot
currently receives the same trajectory parameters and reference, so multiple robots can
collide. Implement separate references before using a fleet for coordination work.

The launch file sets `IGN_PARTITION=camslab_<ROS_DOMAIN_ID>` unless `IGN_PARTITION` is
already set. This keeps simulations in different ROS domains separate on one network.

## Safety polytope implementation

`camslab_webui/polytope.py` computes the counter-clockwise convex hull and writes the
parameter file. `src/polytope.cpp` checks the sampled reference path, refuses enable while
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

1. Change the relevant type in `camslab/include/camslab/types.hpp`.
2. Add ROS parameters or subscriptions in `camslab/src/controller_node.cpp`.
3. Keep command saturation, stale-pose handling, and safety-polytope checks outside the
   experimental control law.
4. Add or update tests before changing the launch configuration.
5. Rebuild with `colcon build --symlink-install --packages-select camslab`.
