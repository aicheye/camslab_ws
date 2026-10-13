# CAMS Lab Gazebo Simulator

This simulator models the lab's robot cars (Yahboom ROSMASTER R2) as they follow a path. You pick a path, a controller steers the car along it, and you watch it in 3D and download the run as a CSV. The same controller drives the real cars in the lab.

```{tip}
**Quick start**, after {doc}`install`:

    source install/setup.bash
    ros2 launch camslab gazebo_multi.launch.py

Then open <http://localhost:8000>.
```

```{toctree}
:caption: Using camslab
:maxdepth: 2

install
running
paths
safety-area
your-controller
troubleshooting
how-it-works
```
