# camslab_ws

Read the user documentation for lab members, written for people without ROS experience,
in
[`docs/`](docs/) and is built by Read the Docs (`.readthedocs.yaml`). Start with the
[documentation home page](docs/index.md). To build it locally:

    uv run --no-project --with-requirements docs/requirements.txt sphinx-build docs docs/_build/html

Trajectory tracking for a Yahboom ROSMASTER R2 (Ackermann) driven through a
differential-drive `(v, w)` interface. The robot follows a reference p*(t), a circle, a
lemniscate of Gerono, an ellipse, a rectangle or a closed cubic B-spline, with

    v = < v* q* + k_par (p* - p), q >
    w = w* + < k_perp v* (p* - p) + k_q q*, S q >

where `<a, b>` is the dot product, `S = [[0, -1], [1, 0]]`, and `k_par, k_perp, k_q > 0`.

| Sim | What | Code | Run |
|---|---|---|---|
| Offline | unicycle model, Python, one run at a time | `camslab_sim/` | `cd camslab_sim && uv run python -m camslab_sim` |
| Gazebo + RViz | R2s in Gazebo (one by default), C++ / ROS 2 | `camslab/` | `ros2 launch camslab gazebo_multi.launch.py` |
| RoboMaster S1 | the real S1 `angostura`, mocap pose, C++ / ROS 2 | `camslab/` | `ros2 launch camslab live.launch.py` |

All three serve the same web UI (`camslab_webui/`) on http://localhost:8000.

## What is left to implement

The circle, `reference()` and `control()` are written in Python and C++, and `step()` in
Python. Left:

| File | Function |
|---|---|
| `camslab_sim/camslab_sim/trajectory.py` | `Ellipse`, `Rectangle` and `Spline`: `position()`, `velocity()`, `acceleration()` |
| `camslab/src/trajectory.cpp` | `Rectangle`: the same functions in C++ |
| `camslab_estimation/camslab_estimation/online.py` | `ode()`: mass-spring-damper, filters and gradient law as one ODE, see `docs/estimation-hints.md` |

`velocity()` and `acceleration()` are the first and second time derivatives of
`position()`, worked out by hand. `reference()` turns the three into p*, q*, v*, w* and
the curvature kappa* = w* / v*. The circle is given by its curvature kappa rather than
its radius, so kappa = 0 is a straight line.

Until these are written the offline sim shows the first `... is not implemented` in the
UI header, and `controller` logs it while commanding zero. Notes on integrating the
plant: [docs/ode-hints.md](docs/ode-hints.md).

### Spline

`shape: spline` is a closed uniform cubic B-spline around `control_points`
(`[x0, y0, x1, y1, ...]`, at least 4 points), one lap every `period` seconds, so each of
the N segments lasts `period / N`. It needs no linear solve, position, velocity and
acceleration are continuous at the segment joins (so w* and kappa* have no jumps), and
the curve stays inside the convex hull of its control points: control points inside the
safety polytope give a path inside it. It passes near the control points, not through
them. The `Spline` docstring in `trajectory.py` describes the segment indexing.
The web UI edits the control points on the map, and Random picks 6 to 9 of them, some pulled in towards the middle so the path has dents.

### Rectangle

`shape: rectangle` is centred on `(x0, y0)` with side `2a` along `theta0` and side `2b`
across it, one counter-clockwise lap every `period` seconds at constant speed
`4 (a + b) / period`, starting at the middle of the side at
`(x0, y0) + a (cos theta0, sin theta0)`. Each side is straight, so kappa* = 0 and w* = 0
between corners and `check_limits()` passes. At a corner q*
turns by 90 degrees at once, which the car cannot follow, so `||p* - p||` rises after
each corner. `tests/test_tracking.py` leaves the rectangle out for that reason.

### Limits

`check_limits()` (Python) and `checkLimits()` (C++) sample `reference()` over
`[0, t_max]` and report where `|kappa*| > kappa_max` or `|w*| > w_max`. Such a trajectory
is still accepted, and the UI header shows the warning. Every command is clamped to
`|v| <= v_max, |w| <= min(w_max, kappa_max |v|)`, so the car lags p* where the trajectory
is past the limits. The default `kappa_max = 2.9 1/m` is the R2's turning limit
`tan(0.6) / 0.235`. The Gerono default `a = 2 m` peaks at `|kappa*| = 2.4 1/m`; `a = 1.5 m`
would reach 3.2 1/m, which gives a warning.
The ellipse peaks at `|kappa*| = a / b^2`, 2 1/m for the default `a = 2 m, b = 1 m`.
The default spline control points give `|kappa*| <= 1.8 1/m` at `period = 40 s`.

## Offline sim

    cd camslab_sim
    uv run python -m camslab_sim                   # web UI, press Start
    uv run python -m camslab_sim --shape gerono    # every Params field is a flag
    uv run python -m camslab_sim --csv run.csv     # one run without the UI
    uv run pytest                              # passes once Ellipse, Rectangle and Spline in trajectory.py are written

| Test file | Checks |
|---|---|
| `tests/test_plant.py` | `step()` |
| `tests/test_trajectory.py` | `velocity()` and `acceleration()` against finite differences of `position()` and `velocity()`, the circle's start pose, speed and curvature, the Gerono's and ellipse's centre, period and size, the ellipse's curvature at its axis ends, the rectangle's start, corners, speed and curvature, the spline's period, continuity at segment joins and convex hull, `reference()`, `check_limits()` |
| `tests/test_tracking.py` | `||p* - p|| < 0.05 m` over t in [30, 40] s (spline: [50, 60] s) from four initial poses, every shape except the rectangle |

The robot starts at a random pose (x and y in 0 to 4 m, any heading). Pass
`--initial X Y THETA_DEG` or `--seed N` for a repeatable start. Parameters are in
`camslab_sim/types.py` `Params`; `(x0, y0, theta0)` is the circle's start pose and the
Gerono's, ellipse's or rectangle's centre and axis, and `phi0` is the Gerono's and ellipse's
lap angle at t = 0. The ellipse has half-axis `a` along `theta0` and `b` across it, and
the rectangle has half sides `a` along `theta0` and `b` across it.

`simulate.py` is a fixed-step loop: evaluate `reference()` at t, sample `control()`, clamp
`(v, w)` the same way as the ROS `controller`, hold it over `dt`, call `step()`. The default
`dt = 0.02 s` is the Gazebo controller's 50 Hz, and the default `t_max = 60 s` matches the
controller's `t_max`. Start returns the whole run and the UI plays it back with the
reference path as an orange line and p* as a cross.

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

    docker compose -f docker/compose.yaml run --rm ros
    colcon build --symlink-install && source install/setup.bash
    ros2 launch camslab gazebo_multi.launch.py

Rebuild the image (`docker compose -f docker/compose.yaml build`) after adding a
dependency to a `package.xml`.

Three files in `config/` hold the settings:

| File | Used by | Holds |
|---|---|---|
| `params.yaml` | both | trajectory, gains, `w_max`, `v_max`, `pose_timeout`, `rate` |
| `sim.yaml` | `gazebo_multi.launch.py` | robot list, `gui`, `rviz`, `webui`, `port`, `start_enabled`, and `controller` values for the R2 (`kappa_max`) |
| `live.yaml` | `live.launch.py` | S1 and mocap connection, `rviz`, `webui`, `port`, `start_enabled`, and `controller` values for the S1 |

A launch argument overrides the top-level value of the same name
(`ros2 launch camslab gazebo_multi.launch.py gui:=true`), and `config:=<file>` picks
another settings file. `controller` values load after `params.yaml`, so they replace
it where both set a parameter. `polytope:=<file>` sets the safety polytope file (below):
loaded when it exists, written by the web UI, `config/polytope.yaml` by default, empty
for none.

RViz shows each robot, its `ref_path` (orange line) and its current `ref_pose`
p* (orange arrow). The controllers start enabled (`start_enabled: true`), and each
robot's trajectory clock starts at its first pose. Changing a trajectory parameter
(`ros2 param set /angostura/controller shape gerono`) restarts the clock and
republishes the path; a change that breaks `kappa_max` or `w_max` is accepted, logged as a
warning, and returned as the reason of the successful change. The web UI is optional: its Stop and Start disable and enable every controller,
and parameters set in it go to every controller.

### Fleet

`robots` in `config/sim.yaml` lists each robot's `name`, spawn pose, and colour. It has one robot,
angostura; the others are commented out because every robot tracks the same p*(t) and
they would collide. `gazebo_multi.launch.py` starts the world, RViz, and the web UI, and
includes `robot.launch.py` once per fleet entry with `name:=<name>` and the spawn pose.
Add one robot to a running world with `robot.launch.py`:
`ros2 launch camslab robot.launch.py name:=ferret x:=1 y:=2`.

Per robot, in namespace `<name>` (Gazebo model `<name>`):

| Node | Subscribes | Publishes |
|---|---|---|
| `controller` (C++) | `pose` | `cmd_vel`, `ref_pose` (PoseStamped), `ref_path` (Path, transient local), `safety_polytope` (PolygonStamped, transient local) |
| `parameter_bridge` | Gazebo pose, joint states; `cmd_vel` | `pose`, `joint_states`; Gazebo `cmd_vel` |
| `robot_state_publisher` | `joint_states`, `urdf/r2.urdf.xacro` | TF `<name>/base_footprint -> <name>/*_link`, at most 50 Hz |
| `pose_tf` (C++) | `pose` | TF `map -> <name>/base_footprint` |

`controller` clamps each command to `|v| <= v_max` and
`|w| <= min(w_max, kappa_max |v|)`, and commands zero when the pose is older than
`pose_timeout`. It starts disabled unless `start_enabled`, and `~/enable` (SetBool)
switches it; enabling fails while the trajectory breaks the limits. Trajectory, gains
and limits are in `config/params.yaml` (`kappa_max` in `sim.yaml` or `live.yaml`) and
Change these parameters at runtime. Frames follow
REP-105/120.

Every node runs on Gazebo's simulation clock (`use_sim_time`, `/clock` bridged by
`gazebo_multi.launch.py`). The body TF from `pose` and the wheel TF from `joint_states`
carry Gazebo's stamps, so RViz draws the wheels on the body, and the trajectory clock
follows the simulation when Gazebo runs slower than real time.

`webui_bridge` runs in the root namespace. It plots the first robot in the fleet
file or any other ("Plots for"), and draws every robot, its p*, and the first robot's
reference path. Reset stops the controllers and clears the plots.

### Safety polytope

A convex safe area on the floor, drawn in the web UI (Gazebo only): corners are added by
clicking the map (rounded to 5 cm) or from the plotted robot's latest pose ("Add robot
position", which with mocap means carrying the car to each corner). "Apply and save"
sends the corners to `webui_bridge`, which takes their counter-clockwise convex hull
(`camslab_webui/polytope.py`), sets `polytope` and `polytope_margin` atomically on the
first controller and, once it accepts them, on the others, and writes the controller
parameter file given by the `polytope` launch argument:

    /**/controller:
      ros__parameters:
        polytope: [x1, y1, x2, y2, ...]   # map frame, counter-clockwise, convex
        polytope_margin: 0.2              # [m]

The default is `config/polytope.yaml` in the source tree (found through the
`--symlink-install` link of the launch file), which is ignored by git because it belongs to
one lab setup. `robot.launch.py` loads it after `params.yaml` when it exists. Applying no
corners removes the polytope and deletes the file. Mocap coordinates move with every
calibration, so the corners are data in this file, not code, and are drawn again after
each calibration.

With a non-empty `polytope`, `controller` (`src/polytope.cpp`):

- rejects a trajectory, at startup or on a parameter change, whose sampled p*(t) over
  `[0, t_max]` comes closer than `polytope_margin` to an edge, and keeps the previous
  trajectory,
- refuses `~/enable` while the robot's pose is outside the polytope,
- disables itself, and so commands zero for 1 s, on the first fresh pose outside it.

The robot check uses the tracked point (`base_footprint`, the rear axle centre) with no
margin. A `polytope` that is not convex and counter-clockwise is rejected, and at startup
it stops the node. The offline sim has no polytope and hides the editor.

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

## RoboMaster S1

`live.launch.py` runs the same controller on a real DJI RoboMaster S1 in place of
Gazebo. The pose comes from the lab mocap through
[vrpn_mocap](https://github.com/alvinsunyixiao/vrpn_mocap) and the commands go to the
robot through [robomaster_ros](https://github.com/jeguzzi/robomaster_ros). The Docker
image installs both; robomaster_ros is built in `/opt/robomaster_ws`, which `.bashrc`
sources before `/ws/install`.

    ros2 launch camslab live.launch.py      # settings in config/live.yaml

| Topic | From | To |
|---|---|---|
| `/vrpn_mocap/angostura/pose`, remapped to `/angostura/pose` | vrpn_mocap, VRPN server `129.97.71.49:3883` | controller, pose_tf, web UI |
| `/angostura/cmd_vel` (`linear.x`, `angular.z`) | controller | S1 driver `/angostura/robomaster` |

| `live.yaml` | Default | |
|---|---|---|
| `name` | `angostura` | ROS namespace and TF prefix |
| `serial_number` | `3JKDH6U001689X` | on the S1 intelligent controller |
| `conn_type` | `sta` | S1 on the lab router; `ap` for the robot's own Wi-Fi, `rndis` for USB |
| `tracker` | `name` | rigid body name in the mocap software |
| `mocap_server`, `mocap_port` | `129.97.71.49`, `3883` | VRPN server |
| `chassis_timeout` | `0.5` | the S1 stops when `cmd_vel` is older than this [s] |
| `start_enabled` | `false` | press Start in the web UI to drive |
| `controller` | `kappa_max: 2.9` | controller parameters on top of `params.yaml` |

The mocap rigid body must be named `angostura` (or set `tracker`) and streamed with Z up,
because the controller takes the heading as the yaw about Z and x forward on the robot.
Every node runs on the wall clock. The S1 driver publishes TF `angostura/odom ->
angostura/base_link` from wheel odometry, which is not connected to `map`, so RViz draws the
robot as the `angostura/pose` arrow and the `angostura/mocap` axes.

## Web UI

The map view is fixed with the origin at the bottom-left corner and 6 m along the
shorter side. Wheel zooms, drag pans, and Fit frames the current run and the reference
path. The view is kept in the browser's localStorage.

The Random button next to the shape picks a random start pose and size for the selected
shape, or random control points for the spline (`camslab_webui/random_trajectory.py`), that
stays 0.3 m inside the 0 to 6 m square and within `kappa_max` and `w_max`. The offline sim checks each candidate with
`check_limits()` and the sampled path; in Gazebo, the bridge sends it with
`set_parameters_atomically` and tries another when `controller` rejects it or returns a
limit warning.

With `shape: spline`, the Spline control points box edits `control_points` on the map:
click to add a point (it goes into the nearest edge of the control polygon), drag a point
to move it, shift-click to remove it, then Apply. Apply sends `set_params` with the flat
`control_points`, which the controller or the offline sim rejects with fewer than 4 points.

JSON over a WebSocket at `/ws`. Server code: `camslab_webui/camslab_webui/server.py`.

| Direction | Message |
|---|---|
| server -> UI | `{"type": "hello", "backend", "mode": "batch" \| "live", "can_set_pose", "can_set_polytope", "robot_name", "fields"}` |
| server -> UI | `{"type": "config", "params": {...}, "path": [[x, y], ...], "initial": {"x", "y", "theta"}, "polytope": {"points", "margin", "file"}}` (`polytope` from Gazebo only) |
| server -> UI | `{"type": "samples", "reset": bool, "samples": [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}` |
| server -> UI | `{"type": "fleet", "reset": bool, "robots": {name: [[t, x, y, qx, qy, v, w, dist, xr, yr], ...]}}` (robots after the first) |
| server -> UI | `{"type": "status", "state": "idle" \| "running" \| "error", "message"}` |
| UI -> server | `set_initial {x, y, theta}`, `set_params {params}`, `randomize_trajectory`, `set_polytope {points: [[x, y], ...], margin}`, `start`, `stop`, `reset` |

`dist` is `||p* - p||` and `(xr, yr)` is p*; `xr` and `yr` are `null` until a live robot
publishes its first `ref_pose`.

The offline sim sends one `samples` message per run (`mode: "batch"`) and accepts
`set_initial`. The Gazebo fleet streams one row per robot at 25 Hz (`mode: "live"`).
