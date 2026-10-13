# Run the simulator

## Start and stop

Run these commands from the repository root in a terminal where ROS 2 and this workspace
are sourced:

```bash
cd ~/camslab_ws
source install/setup.bash
ros2 launch camslab gazebo_multi.launch.py
```

The default launch starts one robot, RViz, the web UI, and the controller enabled. To
disable RViz or the web UI, copy `config/sim.yaml`, change the values, and pass the copy
with `config:=/path/to/sim.yaml`. Override matching settings with launch arguments such
as `gui:=true`.

Wait about 20 seconds for RViz and the car to start following the default path. Open the
web UI at <http://localhost:8000>. Stop the simulator with `Ctrl + C`. Add `gui:=true` to
the launch command to open the Gazebo window.

In RViz, the orange line is the path and the orange arrow is the reference point $p^*$. If a safe area is set, it is drawn as a red outline.

```{image} images/rviz.png
:alt: RViz showing the car on a circular path
```

## Web UI

```{image} images/webui.png
:alt: The web UI with the map on the left and the controls and plots on the right
```

| Button | What it does |
|---|---|
| **Start** / **Stop** | Enable or disable the controller. The car stops when it is disabled. |
| **Reset** | Stop and clear the plots. |
| **Download CSV** | Save the logged run as a CSV file. |
| **Fit** / **Reset view** | Zoom the map to fit the path, or reset the zoom. |
| **Random** | Generate a random path within the floor and the car's turning limits. |

Use (0, 0) as the bottom-left map origin. Scroll to zoom and drag to pan. Read errors
and warnings at the top of the page.

If the browser cannot connect, keep the launch terminal running while the web server
starts after the ROS nodes. Open `http://localhost:8000`, or change `port` in
`config/sim.yaml` when another process uses port 8000.

## Record a run

1. Press **Reset**, set the path ({doc}`paths`), and press **Start**.
2. Press **Stop**, then **Download CSV**. A run is logged for up to 60 s.
3. Rename the file descriptively, for example `circle_kperp4_trial1.csv`. Record the
   parameters separately; the CSV does not include them.

| Column | Meaning | Unit |
|---|---|---|
| `t` | time since Start | s |
| `x`, `y` | car position $p$ | m |
| `qx`, `qy` | car heading as a unit vector, angle = `atan2(qy, qx)` | |
| `v`, `w` | commanded forward speed and turning rate | m/s, rad/s |
| `dist` | tracking error $\lVert p^* - p \rVert$ | m |
| `xr`, `yr` | reference position $p^*$ | m |
