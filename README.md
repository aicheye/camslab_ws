# camslab_ws

Run trajectory-tracking experiments for the lab's Yahboom ROSMASTER R2 cars and DJI
RoboMaster S1, and survey ArUco fiducials from a camera on a car with the adaptive
estimator of the ECC 2027 paper. The project provides a standalone Python simulator (no
ROS), a Gazebo/RViz simulator, a ROS 2 controller for the real S1 and the fiducial survey
on a real car with the motion capture.

## Start here

Read the [documentation home page](docs/index.md) for installation and the first run.

| Goal | Documentation |
|---|---|
| Install dependencies | [Install](docs/install.md) |
| Run Gazebo and RViz | [Run the simulator](docs/running.md) |
| Change or draw a path | [Paths and settings](docs/paths.md) |
| Configure trajectories and robots | [Configuration reference](docs/configuration.md) |
| Modify the controller | [Modify the controller](docs/your-controller.md) |
| Write a ROS 2 controller node | [Write a custom node](docs/custom-node.md) |
| Survey fiducials with a camera on a car | [Fiducial surveying on a real car](docs/hardware-survey.md) |
| Read ROS and package details | [Developer reference](docs/developer.md) |

## Quick start

After installing ROS 2 Humble and the workspace dependencies:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch camslab gazebo_multi.launch.py
```

Open <http://127.0.0.1:8000>. Press **Start** in the web UI, then inspect the run in RViz
and download the CSV from the web UI.

## Simulation modes

| Mode | Command | Purpose |
|---|---|---|
| Standalone Python | `cd camslab_standalone && uv run python -m camslab_standalone.tracking` | Test the plant and trajectories without ROS or Gazebo. |
| Gazebo + RViz | `ros2 launch camslab gazebo_multi.launch.py` | Test the R2 model with ROS 2. |
| Real S1 | `ros2 launch camslab live.launch.py` | Run the ROS 2 controller with mocap and the S1 driver. |
| Fiducial survey | `ros2 launch camslab survey.launch.py` | Run the camera node on a car and the surveying estimator with the mocap; `camera_test.launch.py` checks the camera alone in RViz. |

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
| `camslab/` | Python ROS 2 nodes, control law, trajectories, adaptive estimators, Gazebo model, launch files, and configuration |
| `camslab_webui/` | shared web UI and backend |
| `camslab_standalone/` | standalone Python experiments, no ROS: tracking, mass-spring-damper estimation, circular path identification, fiducial surveying |
| `hardware/` | printable parts: the fiducial marker stands and the markers for them |
| `docker/` | ROS 2 Humble image with the lab drivers |
| `docs/` | user and developer documentation |
