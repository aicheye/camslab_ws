# Run the simulator

## Start and stop

```bash
cd ~/camslab_ws
source install/setup.bash
ros2 launch camslab gazebo_multi.launch.py
```

RViz opens within 20 seconds, the car starts following its path, and the web UI is at <http://localhost:8000>. Press `Ctrl + C` in the terminal to stop. Add `gui:=true` to the launch command to also open Gazebo's own window.

In RViz the green car is the robot, the orange line is its path, the orange arrow is where it should be right now ($p^*$), and a red outline is the safe area if one is set.

## Web UI

| Button | What it does |
|---|---|
| **Start** / **Stop** | Turn the controller on or off. Off, the car stops. |
| **Reset** | Stop and clear the plots. |
| **Download CSV** | Save the run. |
| **Fit** / **Reset view** | Zoom the map to the path, or back to the default. |
| **Random** | Pick a random path that fits the floor and the car's limits. |

The map has (0, 0) at the bottom left. Scroll to zoom and drag to pan. Errors and warnings appear at the top of the page.

## Record a run

1. Press **Reset**, choose the path ({doc}`paths`), and press **Start**.
2. After up to 60 s, press **Stop**, then **Download CSV**.
3. Rename the file to say what you ran, for example `circle_kperp4_trial1.csv`. The file does not record the settings.

| Column | Meaning | Unit |
|---|---|---|
| `t` | time since Start | s |
| `x`, `y` | car position $p$ | m |
| `qx`, `qy` | car heading as a unit vector; the angle is `atan2(qy, qx)` | |
| `v`, `w` | commanded forward speed and turning rate | m/s, rad/s |
| `dist` | tracking error $\lVert p^* - p \rVert$ | m |
| `xr`, `yr` | reference position $p^*$ | m |
