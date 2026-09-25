# camslab_ws

Trajectory tracking for a Yahboom ROSMASTER R2 (Ackermann) driven through a
differential-drive `(v, w)` interface. The robot follows a reference p*(t), first a
circle and then a lemniscate of Gerono, with

    v = < v* q* + k_par (p* - p), q >
    w = w* + < k_perp v* (p* - p) + k_q q*, S q >

where `<a, b>` is the dot product, `S = [[0, -1], [1, 0]]`, and `k_par, k_perp, k_q > 0`.

| Sim | What | Code | Run |
|---|---|---|---|
| Offline | unicycle model, Python, one run at a time | `camslab_sim/` | `cd camslab_sim && uv run python -m camslab_sim` |
| Gazebo + RViz | R2s in Gazebo (one by default), C++ / ROS 2 | `camslab/` | `ros2 launch camslab gazebo_multi.launch.py` |

Both serve the same web UI (`camslab_webui/`) on http://localhost:8000.

## What is left to implement

The circle, `reference()`, `control()` and `step()` are written in Python and C++. Left:

| File | Function |
|---|---|
| `camslab_sim/camslab_sim/trajectory.py` | `Gerono`: `position()`, `velocity()`, `acceleration()` |
| `camslab/src/trajectory.cpp` | the same Gerono functions in C++ |

`velocity()` and `acceleration()` are the first and second time derivatives of
`position()`, worked out by hand. `reference()` turns the three into p*, q*, v*, w* and
the curvature kappa* = w* / v*. The circle is given by its curvature kappa rather than
its radius, so kappa = 0 is a straight line.

Until these are written the offline sim shows the first `... is not implemented` in the
UI header, and `controller` logs it while commanding zero. Notes on integrating the
plant: [docs/ode-hints.md](docs/ode-hints.md).

### Limits

Before a run, `check_limits()` (Python) and `checkLimits()` (C++) sample `reference()`
and reject a trajectory with `|kappa*| > kappa_max` or `|w*| > w_max`. The default
`kappa_max = 2.9 1/m` is the R2's turning limit `tan(0.6) / 0.235`. The Gerono default
`a = 2 m` peaks at `|kappa*| = 2.4 1/m`; `a = 1.5 m` would reach 3.2 1/m and is rejected.

## Offline sim

    cd camslab_sim
    uv run python -m camslab_sim                   # web UI, press Start
    uv run python -m camslab_sim --shape gerono    # every Params field is a flag
    uv run python -m camslab_sim --csv run.csv     # one run without the UI
    uv run pytest                              # passes once Gerono in trajectory.py is written

| Test file | Checks |
|---|---|
| `tests/test_plant.py` | `step()` |
| `tests/test_trajectory.py` | `velocity()` and `acceleration()` against finite differences of `position()` and `velocity()`, the circle's start pose, speed and curvature, the Gerono's centre, period and size, `reference()`, `check_limits()` |
| `tests/test_tracking.py` | `||p* - p|| < 0.05 m` over t in [30, 40] s from four initial poses, both shapes |

The robot starts at a random pose (x and y in 0 to 4 m, any heading). Pass
`--initial X Y THETA_DEG` or `--seed N` for a repeatable start. Parameters are in
`camslab_sim/types.py` `Params`; `(x0, y0, theta0)` is the circle's start pose and the
Gerono's centre and axis.

`simulate.py` is a fixed-step loop: evaluate `reference()` at t, sample `control()`, hold
`(v, w)` over `dt`, call `step()`. Start returns the whole run and the UI plays it back
with the reference path dashed and p* as a cross. The unicycle model has no turning
limit, unlike the Ackermann car in Gazebo.

## Gazebo + RViz

ROS 2 Humble with Gazebo Fortress (`ros-humble-ros-gz`), on Ubuntu 22.04 or in Docker.
The repo root is a colcon workspace: `build/`, `install/` and `log/` go next to the
packages and are ignored by git. `camslab_sim` has a `COLCON_IGNORE` and is not built.

On a host with ROS 2 Humble installed:

    source /opt/ros/humble/setup.bash
    cd camslab_ws
    rosdep install --from-paths . --ignore-src -y   # Gazebo bridge, RViz, xacro, aiohttp, ...
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch camslab gazebo_multi.launch.py

In Docker (`docker/Dockerfile` runs the same `rosdep install`):

    xhost +local:                                   # on the host, once per login, for RViz
    docker compose -f docker/compose.yaml run --rm ros
    colcon build --symlink-install && source install/setup.bash
    ros2 launch camslab gazebo_multi.launch.py

Rebuild the image (`docker compose -f docker/compose.yaml build`) after adding a
dependency to a `package.xml`.

| Argument | Default | |
|---|---|---|
| `fleet` | `config/fleet.yaml` | robot list |
| `rviz` | `true` | RViz, the main view |
| `webui` | `true` | web UI on `port` (8000) |
| `gui` | `false` | Gazebo window |

RViz shows each robot, its `ref_path` (orange line) and its current `ref_pose`
p* (orange arrow). The controllers start enabled (`start_enabled: true`), and each
robot's trajectory clock starts at its first pose. Changing a trajectory parameter
(`ros2 param set /angostura/controller shape gerono`) restarts the clock and
republishes the path; a change that breaks `kappa_max` or `w_max` is rejected with the
reason. The web UI is optional: its Stop and Start disable and enable every controller,
and parameters set in it go to every controller.

### Fleet

`config/fleet.yaml` lists each robot's `name`, spawn pose, and colour. It has one robot,
angostura; the others are commented out because every robot tracks the same p*(t) and
they would collide. `gazebo_multi.launch.py` starts the world, RViz, and the web UI, and
includes `robot.launch.py` once per fleet entry with `name:=<name>` and the spawn pose.
`robot.launch.py` can also add one robot to a running world:
`ros2 launch camslab robot.launch.py name:=ferret x:=1 y:=2`.

Per robot, in namespace `<name>` (Gazebo model `<name>`):

| Node | Subscribes | Publishes |
|---|---|---|
| `controller` (C++) | `pose` | `cmd_vel`, `ref_pose` (PoseStamped), `ref_path` (Path, transient local) |
| `parameter_bridge` | Gazebo pose, joint states; `cmd_vel` | `pose`, `joint_states`; Gazebo `cmd_vel` |
| `robot_state_publisher` | `joint_states`, `urdf/r2.urdf.xacro` | TF `<name>/base_footprint -> <name>/*_link`, at most 50 Hz |
| `pose_tf` (C++) | `pose` | TF `map -> <name>/base_footprint` |

`controller` clamps each command to `|v| <= v_max` and
`|w| <= min(w_max, kappa_max |v|)`, and commands zero when the pose is older than
`pose_timeout`. It starts disabled unless `start_enabled`, and `~/enable` (SetBool)
switches it; enabling fails while the trajectory breaks the limits. Trajectory, gains
and limits are in `config/params.yaml` and can change at runtime. Frames follow
REP-105/120.

Every node runs on Gazebo's simulation clock (`use_sim_time`, `/clock` bridged by
`gazebo_multi.launch.py`). The body TF from `pose` and the wheel TF from `joint_states`
carry Gazebo's stamps, so RViz draws the wheels on the body, and the trajectory clock
follows the simulation when Gazebo runs slower than real time.

`webui_bridge` runs in the root namespace. It plots the first robot in the fleet
file or any other ("Plots for"), and draws every robot, its p*, and the first robot's
reference path. Reset stops the controllers and clears the plots.

### Gazebo model

| Piece | What it does |
|---|---|
| `models/r2/model.sdf` | R2 with an Ackermann front axle. Geometry and meshes from Yahboom's `yahboomcar_R2.urdf.xacro`: wheelbase 0.235 m, track 0.1685 m, wheel radius 0.0345 m, steering limit 0.6 rad. `urdf/r2.urdf.xacro` repeats this geometry for RViz; change both together. |
| `AckermannSteering` system | reads `cmd_vel` as `linear.x` = v [m/s], `angular.z` = w [rad/s] and sets the steering angle from v / w. The car cannot turn in place: `|w| <= |v| tan(0.6) / 0.235`. |
| `PosePublisher`, `JointStatePublisher` systems | model pose (100 Hz) and wheel and steering joint states (every 1 ms physics step) |
| `worlds/gui.config` | Gazebo window (`gui:=true`): camera over x, y in [0, 6] m and a 1 m grid |

The Gazebo car holds its last `cmd_vel`. `controller` publishes zero while disabled.
The model frame, `base_footprint`, is on the ground at the middle of the rear axle,
0.116 m behind `base_link`. That point of an Ackermann car moves as a unicycle,
`dp/dt = v q`, which is the plant the control law assumes; tracking the body centre
instead leaves a 0.137 m steady error on the default circle. Fleet spawn poses place
this point.

The launch files set `IGN_PARTITION=camslab_<ROS_DOMAIN_ID>` unless it is already set, so
Gazebo sims in different `ROS_DOMAIN_ID`s on one network stay apart.

## Web UI

The map view is fixed with the origin at the bottom-left corner and 6 m along the
shorter side. Wheel zooms, drag pans, and Fit frames the current run and the reference
path. The view is kept in the browser's localStorage.

The Random button next to the shape picks a random start pose and size for the selected
shape (`camslab_webui/random_trajectory.py`) that stays 0.3 m inside the 0 to 6 m square and
within `kappa_max` and `w_max`. The offline sim checks each candidate with
`check_limits()` and the sampled path; in Gazebo, `controller` checks it, and the
bridge sends it with `set_parameters_atomically` so a rejected set changes nothing.

JSON over a WebSocket at `/ws`. Server code: `camslab_webui/camslab_webui/server.py`.

| Direction | Message |
|---|---|
| server -> UI | `{"type": "hello", "backend", "mode": "batch" \| "live", "can_set_pose", "robot_name", "fields"}` |
| server -> UI | `{"type": "config", "params": {...}, "path": [[x, y], ...], "initial": {"x", "y", "theta"}}` |
| server -> UI | `{"type": "samples", "reset": bool, "samples": [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}` |
| server -> UI | `{"type": "fleet", "reset": bool, "robots": {name: [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}}` (robots after the first) |
| server -> UI | `{"type": "status", "state": "idle" \| "running" \| "error", "message"}` |
| UI -> server | `set_initial {x, y, theta}`, `set_params {params}`, `randomize_trajectory`, `start`, `stop`, `reset` |

`dist` is `||p* - p||` and `(xr, yr)` is p*; `xr` and `yr` are `null` until a live robot
publishes its first `ref_pose`.

The offline sim sends one `samples` message per run (`mode: "batch"`) and accepts
`set_initial`. The Gazebo fleet streams one row per robot at 25 Hz (`mode: "live"`).
