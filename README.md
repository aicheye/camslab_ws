# camslab_ws

Run trajectory-tracking experiments for the lab's Yahboom ROSMASTER R2 cars and DJI
RoboMaster S1. The project provides an offline Python simulator, a Gazebo/RViz simulator,
and a ROS 2 controller for the real S1.

## Start here

Read the [documentation home page](docs/index.md) for installation and the first run.

| Goal | Documentation |
|---|---|
| Install dependencies | [Install](docs/install.md) |
| Run Gazebo and RViz | [Run the simulator](docs/running.md) |
| Change or draw a path | [Paths and settings](docs/paths.md) |
| Configure trajectories and robots | [Configuration reference](docs/configuration.md) |
| Modify the C++ controller | [Modify the controller](docs/your-controller.md) |
| Write a ROS 2 controller node | [Write a custom node](docs/custom-node.md) |
| Read ROS and package details | [Developer reference](docs/developer.md) |

## Quick start

After installing ROS 2 Humble and the workspace dependencies:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch camslab gazebo_multi.launch.py
```

Open <http://localhost:8000>. Press **Start** in the web UI, then inspect the run in RViz
and download the CSV from the web UI.

## Simulation modes

| Mode | Command | Purpose |
|---|---|---|
| Offline Python | `cd camslab_sim && uv run python -m camslab_sim` | Test the plant and trajectories without ROS or Gazebo. |
| Gazebo + RViz | `ros2 launch camslab gazebo_multi.launch.py` | Test the R2 model with ROS 2. |
| Real S1 | `ros2 launch camslab live.launch.py` | Run the ROS 2 controller with mocap and the S1 driver. |

All modes use the same trajectory and controller concepts. The real-robot launch starts
disabled; verify the mocap pose and press **Start** before moving the S1.

## Build the documentation

```bash
uv run --no-project --with-requirements docs/requirements.txt \
  sphinx-build -W docs docs/_build/html
```

## Repository layout

| Directory | Purpose |
|---|---|
| `camslab/` | C++ ROS 2 controller, Gazebo model, launch files, and configuration |
| `camslab_sim/` | standalone Python simulator |
| `camslab_webui/` | shared web UI and backend |
| `camslab_estimation/` | standalone estimation exercise |
| `docs/` | user and developer documentation |
