# CAMS Lab Cooperative and Adaptive Robotics

Run trajectory-tracking experiments with the lab's Yahboom ROSMASTER R2 cars in Gazebo. Choose a path, inspect it in RViz and the web UI, and export each run as a CSV. Use the same controller on the real cars.

## Choose a setup

Choose **host install** on Ubuntu 22.04 with ROS 2 already installed. Choose
**Docker** to run ROS 2 and Gazebo inside the repository's container.

| Goal | Start here |
|---|---|
| Install ROS 2 on Ubuntu | {doc}`install` |
| Run the existing Docker environment | {doc}`install` → Docker |
| Start the simulator after installation | {doc}`running` |
| Change a path or draw one | {doc}`paths` |
| Configure trajectories and robots | {doc}`configuration` |
| Read ROS and package details | {doc}`developer` |
| Fix a launch problem | {doc}`troubleshooting` |

```{tip}
**Quick start**, after {doc}`install`:

    source install/setup.bash
    ros2 launch camslab gazebo_multi.launch.py

Open <http://127.0.0.1:8000>.
```

```{toctree}
:caption: Using camslab
:maxdepth: 2

install
running
paths
safety-area
configuration
your-controller
custom-node
developer
troubleshooting
how-it-works
```
