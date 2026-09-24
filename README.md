# go-to-goal

Go-to-goal control for a Yahboom ROSMASTER R2 (Ackermann) driven through a
differential-drive `(v, w)` interface. Target: `lim t->inf ||p* - p(t)|| < epsilon`.

| Sim | What | Code | Run |
|---|---|---|---|
| Offline | unicycle model, Python, one run at a time | `gtg_sim/` | `cd gtg_sim && uv run python -m gtg_sim` |
| Gazebo + RViz | leader-follower fleet of R2s, C++ / ROS 2 | `gtg_ros/` | `ros2 launch gtg_ros gazebo_multi.launch.py` |

Both serve the same web UI (`gtg_webui/`) on http://localhost:8000.

## What is left to implement

| File | Function |
|---|---|
| `gtg_sim/gtg_sim/plant.py` | `derivatives()`, `step()` |
| `gtg_sim/gtg_sim/controller.py` | `control()` |
| `gtg_ros/src/controller.cpp` | `control()` |
| `gtg_ros/src/formation.cpp` | `followerGoal()` |

Until then the offline sim shows `control() is not implemented` in the UI header, and
`controller_node` logs the same error while commanding zero. Notes on integrating the
plant: [docs/ode-hints.md](docs/ode-hints.md).

## Offline sim

    cd gtg_sim
    uv run python -m gtg_sim                 # web UI, press Start
    uv run python -m gtg_sim --csv run.csv   # one run without the UI
    uv run pytest                            # passes once plant.py and controller.py are written

The robot starts at a random pose (x and y in 0 to 5 m, any heading). Pass
`--initial X Y THETA_DEG` or `--seed N` for a repeatable start.

`simulate.py` is a fixed-step loop: sample `control()`, hold `(v, w)` over `dt`, call
`step()`. Start returns the whole run and the UI plays it back. The unicycle model has
no turning limit, unlike the Ackermann car in Gazebo.

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

In RViz, the 2D Goal Pose tool (`G`) publishes the leader's goal on `/goal`; the heading
of the arrow is ignored. The controllers start enabled (`start_enabled: true`), so the
fleet drives as soon as a goal arrives. The web UI is optional: clicking its map also
sets `/goal`, it shows goals set in RViz, and its Stop and Start disable and enable every
controller.

### Fleet

`config/fleet.yaml` lists each robot's `name` (angostura, bodies, cologne, estranged), spawn
pose, and `follows` (the name of the robot it follows). The one robot without `follows`
is the leader and drives to `/goal`. Each `controller_node` publishes `followerGoal()`
of its own pose on `/<name>/follow_goal`, and each follower's `goal` is remapped to
`/<follows>/follow_goal`, so every robot runs the same node.

`gazebo_multi.launch.py` starts the world, RViz, and the web UI, and includes
`robot.launch.py` once per fleet entry with `name:=<name>`, the spawn pose, and
`goal_topic`. `robot.launch.py` can also add one robot to a running world:
`ros2 launch gtg_ros robot.launch.py name:=ferret x:=1 y:=2`.

Per robot, in namespace `<name>` (Gazebo model `<name>`):

| Node | Subscribes | Publishes |
|---|---|---|
| `controller_node` (C++) | `pose`, `goal` (PoseStamped, heading ignored) | `cmd_vel`, `follow_goal` (PoseStamped) |
| `parameter_bridge` | Gazebo pose, joint states; `cmd_vel` | `pose`, `gazebo/joint_states`; Gazebo `cmd_vel` |
| `joint_state_publisher` | `gazebo/joint_states` | `joint_states`, stamped with wall time |
| `robot_state_publisher` | `joint_states`, `urdf/r2.urdf.xacro` | TF `<name>/base_footprint -> <name>/*_link` |
| `pose_tf_node` (C++) | `pose` | TF `map -> <name>/base_footprint` |

`controller_node` clamps to `v_max`, `w_max` and commands zero when the pose is older
than `pose_timeout`. It starts disabled unless `start_enabled`, and `~/enable` (SetBool)
switches it. Gains and `spacing` are in `config/params.yaml` and can change at runtime.
Frames follow REP-105/120.

`webui_bridge_node` runs in the root namespace. It publishes `/goal`, plots the leader
or any follower ("Plots for"), and draws every robot and each follower's goal. Gains set
in the UI apply to the leader only. Reset stops the controllers and clears the plots.

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
shorter side. Wheel zooms, drag pans, Fit frames the current run, and the view is kept
in the browser's localStorage.

JSON over a WebSocket at `/ws`. Server code: `gtg_webui/gtg_webui/server.py`.

| Direction | Message |
|---|---|
| server -> UI | `{"type": "hello", "backend", "mode": "batch" \| "live", "can_set_pose", "robot_name", "fields"}` |
| server -> UI | `{"type": "config", "params": {...}, "goal": [x, y], "initial": {"x", "y", "theta"}}` |
| server -> UI | `{"type": "samples", "reset": bool, "samples": [[t, x, y, qx, qy, v, w, dist], ...]}` |
| server -> UI | `{"type": "fleet", "reset": bool, "robots": {name: [[t, x, y, qx, qy, v, w, dist], ...]}, "goals": {name: [x, y]}}` (followers) |
| server -> UI | `{"type": "status", "state": "idle" \| "running" \| "reached" \| "error", "message"}` |
| UI -> server | `set_goal {x, y}`, `set_initial {x, y, theta}`, `set_params {params}`, `start`, `stop`, `reset` |

The offline sim sends one `samples` message per run (`mode: "batch"`) and accepts
`set_initial`. The Gazebo fleet streams one row per robot at 30 Hz (`mode: "live"`).
