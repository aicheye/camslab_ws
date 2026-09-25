# go-to-goal

Trajectory tracking for a Yahboom ROSMASTER R2 (Ackermann) driven through a
differential-drive `(v, w)` interface. The robot follows a reference p*(t), first a
circle and then a lemniscate of Gerono, with

    v = < v* q* + k_par (p* - p), q >
    w = w* + < k_perp v* (p* - p) + k_q q*, S q >

where `<a, b>` is the dot product, `S = [[0, -1], [1, 0]]`, and `k_par, k_perp, k_q > 0`.

| Sim | What | Code | Run |
|---|---|---|---|
| Offline | unicycle model, Python, one run at a time | `gtg_sim/` | `cd gtg_sim && uv run python -m gtg_sim` |
| Gazebo + RViz | R2s in Gazebo (one by default), C++ / ROS 2 | `gtg_ros/` | `ros2 launch gtg_ros gazebo_multi.launch.py` |

Both serve the same web UI (`gtg_webui/`) on http://localhost:8000.

## What is left to implement

| File | Function |
|---|---|
| `gtg_sim/gtg_sim/trajectory.py` | `Circle` and `Gerono`: `position()`, `velocity()`, `acceleration()`; `reference()` |
| `gtg_sim/gtg_sim/controller.py` | `control()` |
| `gtg_ros/src/trajectory.cpp` | the same trajectory functions in C++ |
| `gtg_ros/src/controller.cpp` | `control()` |

`velocity()` and `acceleration()` are the first and second time derivatives of
`position()`, worked out by hand. `reference()` turns the three into p*, q*, v*, w* and
the curvature kappa* = w* / v*. The circle is given by its curvature kappa rather than
its radius, so kappa = 0 is a straight line.

Until these are written the offline sim shows the first `... is not implemented` in the
UI header, and `controller_node` logs it while commanding zero. Notes on integrating the
plant: [docs/ode-hints.md](docs/ode-hints.md).

### Limits

Before a run, `check_limits()` (Python) and `checkLimits()` (C++) sample `reference()`
and reject a trajectory with `|kappa*| > kappa_max` or `|w*| > w_max`. The default
`kappa_max = 2.9 1/m` is the R2's turning limit `tan(0.6) / 0.235`. The Gerono default
`a = 2 m` peaks at `|kappa*| = 2.4 1/m`; `a = 1.5 m` would reach 3.2 1/m and is rejected.

## Offline sim

    cd gtg_sim
    uv run python -m gtg_sim                   # web UI, press Start
    uv run python -m gtg_sim --shape gerono    # every Params field is a flag
    uv run python -m gtg_sim --csv run.csv     # one run without the UI
    uv run pytest                              # passes once trajectory.py and controller.py are written

| Test file | Checks |
|---|---|
| `tests/test_plant.py` | `step()` |
| `tests/test_trajectory.py` | `velocity()` and `acceleration()` against finite differences of `position()` and `velocity()`, the circle's start pose, speed and curvature, the Gerono's centre, period and size, `reference()`, `check_limits()` |
| `tests/test_tracking.py` | `||p* - p|| < 0.05 m` over t in [30, 40] s from four initial poses, both shapes |

The robot starts at a random pose (x and y in 0 to 4 m, any heading). Pass
`--initial X Y THETA_DEG` or `--seed N` for a repeatable start. Parameters are in
`gtg_sim/types.py` `Params`; `(x0, y0, theta0)` is the circle's start pose and the
Gerono's centre and axis.

`simulate.py` is a fixed-step loop: evaluate `reference()` at t, sample `control()`, hold
`(v, w)` over `dt`, call `step()`. Start returns the whole run and the UI plays it back
with the reference path dashed and p* as a cross. The unicycle model has no turning
limit, unlike the Ackermann car in Gazebo.

## Gazebo + RViz

ROS 2 Humble with Gazebo Fortress, in the container (the repo root is a colcon source
directory):

    xhost +local:                                   # on the host, once per login, for RViz
    docker compose -f docker/compose.yaml run --rm ros
    colcon build --symlink-install && source install/setup.bash
    ros2 launch gtg_ros gazebo_multi.launch.py

| Argument | Default | |
|---|---|---|
| `fleet` | `config/fleet.yaml` | robot list |
| `rviz` | `true` | RViz, the main view |
| `webui` | `true` | web UI on `port` (8000) |
| `gui` | `false` | Gazebo window |

RViz shows each robot, its `reference_path` (orange line) and its current `reference`
p* (orange arrow). The controllers start enabled (`start_enabled: true`), and each
robot's trajectory clock starts at its first pose. Changing a trajectory parameter
(`ros2 param set /angostura/controller_node shape gerono`) restarts the clock and
republishes the path; a change that breaks `kappa_max` or `w_max` is rejected with the
reason. The web UI is optional: its Stop and Start disable and enable every controller,
and parameters set in it go to every controller.

### Fleet

`config/fleet.yaml` lists each robot's `name`, spawn pose, and colour. It has one robot,
angostura; the others are commented out because every robot tracks the same p*(t) and
they would collide. `gazebo_multi.launch.py` starts the world, RViz, and the web UI, and
includes `robot.launch.py` once per fleet entry with `name:=<name>` and the spawn pose.
`robot.launch.py` can also add one robot to a running world:
`ros2 launch gtg_ros robot.launch.py name:=ferret x:=1 y:=2`.

Per robot, in namespace `<name>` (Gazebo model `<name>`):

| Node | Subscribes | Publishes |
|---|---|---|
| `controller_node` (C++) | `pose` | `cmd_vel`, `reference` (PoseStamped), `reference_path` (Path, transient local) |
| `parameter_bridge` | Gazebo pose, joint states; `cmd_vel` | `pose`, `gazebo/joint_states`; Gazebo `cmd_vel` |
| `joint_state_publisher` | `gazebo/joint_states` | `joint_states`, stamped with wall time |
| `robot_state_publisher` | `joint_states`, `urdf/r2.urdf.xacro` | TF `<name>/base_footprint -> <name>/*_link` |
| `pose_tf_node` (C++) | `pose` | TF `map -> <name>/base_footprint` |

`controller_node` clamps each command to `|v| <= v_max` and
`|w| <= min(w_max, kappa_max |v|)`, and commands zero when the pose is older than
`pose_timeout`. It starts disabled unless `start_enabled`, and `~/enable` (SetBool)
switches it; enabling fails while the trajectory breaks the limits. Trajectory, gains
and limits are in `config/params.yaml` and can change at runtime. Frames follow
REP-105/120.

`webui_bridge_node` runs in the root namespace. It plots the first robot in the fleet
file or any other ("Plots for"), and draws every robot, its p*, and the first robot's
reference path. Reset stops the controllers and clears the plots.

### Gazebo model

| Piece | What it does |
|---|---|
| `models/r2/model.sdf` | R2 with an Ackermann front axle. Geometry and meshes from Yahboom's `yahboomcar_R2.urdf.xacro`: wheelbase 0.235 m, track 0.1685 m, wheel radius 0.0345 m, steering limit 0.6 rad. `urdf/r2.urdf.xacro` repeats this geometry for RViz; change both together. |
| `AckermannSteering` system | reads `cmd_vel` as `linear.x` = v [m/s], `angular.z` = w [rad/s] and sets the steering angle from v / w. The car cannot turn in place: `|w| <= |v| tan(0.6) / 0.235`. |
| `PosePublisher`, `JointStatePublisher` systems | model pose (100 Hz, frame at the ground below `base_link`) and wheel and steering joint states |
| `worlds/gui.config` | Gazebo window (`gui:=true`): camera over x, y in [0, 6] m and a 1 m grid |

The Gazebo car holds its last `cmd_vel`. `controller_node` publishes zero while disabled.
`IGN_PARTITION` in `docker/compose.yaml` keeps Gazebo sims in different `ROS_DOMAIN_ID`s
apart.

## Web UI

The map view is fixed with the origin at the bottom-left corner and 6 m along the
shorter side. Wheel zooms, drag pans, Fit frames the current run and the reference path, and the view is kept
in the browser's localStorage.

JSON over a WebSocket at `/ws`. Server code: `gtg_webui/gtg_webui/server.py`.

| Direction | Message |
|---|---|
| server -> UI | `{"type": "hello", "backend", "mode": "batch" \| "live", "can_set_pose", "robot_name", "fields"}` |
| server -> UI | `{"type": "config", "params": {...}, "path": [[x, y], ...], "initial": {"x", "y", "theta"}}` |
| server -> UI | `{"type": "samples", "reset": bool, "samples": [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}` |
| server -> UI | `{"type": "fleet", "reset": bool, "robots": {name: [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}}` (robots after the first) |
| server -> UI | `{"type": "status", "state": "idle" \| "running" \| "error", "message"}` |
| UI -> server | `set_initial {x, y, theta}`, `set_params {params}`, `start`, `stop`, `reset` |

`dist` is `||p* - p||` and `(xr, yr)` is p*; `xr` and `yr` are `null` until a live robot
publishes its first `reference`.

The offline sim sends one `samples` message per run (`mode: "batch"`) and accepts
`set_initial`. The Gazebo fleet streams one row per robot at 30 Hz (`mode: "live"`).
