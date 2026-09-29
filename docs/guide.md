# camslab user guide

camslab simulates trajectory tracking for a Yahboom ROSMASTER R2: an Ackermann-steered car driven through a differential-drive `(v, ω)` interface, tracking a reference p\*(t) (circle, lemniscate of Gerono or ellipse) with a 50 Hz controller in Gazebo. Use this guide to install it, run it, record data and swap in your own controller.

> [!TIP]
> **Quick start** (after the one-time setup in section 1)
>
> 1. Open a terminal in `camslab_ws`.
> 2. Run `xhost +local:`, then `docker compose -f docker/compose.yaml run --rm ros`.
> 3. In the container, run `colcon build --symlink-install && source install/setup.bash`.
> 4. Run `ros2 launch camslab gazebo_multi.launch.py`.
> 5. Open http://localhost:8000.

---

## 1. Set up (once per computer)

These steps assume Linux (Ubuntu or Fedora). Everything else (ROS 2 Humble, Gazebo Fortress, RViz) runs inside a Docker container, so you do not install it yourself.

### 1.1 Clone the repository

```bash
cd ~
git clone https://github.com/aicheye/camslab_ws.git
cd camslab_ws
```

If git asks for a password or reports "not found", send Sean your GitHub username to get access.

### 1.2 Install Docker

1. Install Docker Engine by following https://docs.docker.com/engine/install/ for your distribution.
2. Check the install:
   ```bash
   docker run hello-world
   ```
   It should print "Hello from Docker!". If it does not, see section 7.

### 1.3 Build the container image

From `camslab_ws`, run:

```bash
docker compose -f docker/compose.yaml build
```

Expect 10 to 20 minutes and a few GB of downloads. Repeat this only when Sean says a dependency changed.

---

## 2. Run the simulator

### 2.1 Start it

1. On your computer (not in Docker), allow the container to open windows. Do this once per login:
   ```bash
   xhost +local:
   ```
2. Enter the container:
   ```bash
   cd ~/camslab_ws
   docker compose -f docker/compose.yaml run --rm ros
   ```
   The prompt changes to `root@<your computer>:/ws#`. Run every command below inside it.
3. Build the workspace (about a minute the first time, seconds after). Rebuild after every code change or `git pull`:
   ```bash
   colcon build --symlink-install && source install/setup.bash
   ```
4. Launch:
   ```bash
   ros2 launch camslab gazebo_multi.launch.py
   ```
   Within 20 seconds RViz opens, the car starts tracking the reference, and the web UI goes live at http://localhost:8000.
5. Keep this terminal open. It shows the log, and closing it stops the sim.

To also open Gazebo's own window, add `gui:=true` to the launch command. It is slower and shows the same scene as RViz.

### 2.2 Read the RViz window

- Green model: the car.
- Orange line: the reference path.
- Orange arrow: p\*, the current reference pose.

### 2.3 Stop it

Press `Ctrl + C` in the launch terminal, then type `exit` to leave the container.

---

## 3. Use the web UI

Open http://localhost:8000 while the sim runs.

### 3.1 Controls

| Button | Effect |
|---|---|
| **Start** / **Stop** | Enable or disable the controller. |
| **Reset** | Stop and clear the plots. |
| **Download CSV** | Export the logged run. |
| **Fit** / **Reset view** | Fit the map to the path, or restore the default zoom. |

Scroll to zoom the map; drag to pan.

### 3.2 Set the reference

1. Pick a **shape**: `circle`, `gerono` or `ellipse`.
2. Enter the parameters (section 5.1), or press **Random** for a reference that fits the floor and the car's limits.
3. Press Enter or click out of the field to apply.

The UI rejects a reference whose curvature exceeds κ_max or whose angular rate exceeds ω_max, states the reason, and keeps the previous reference. A shape marked "not implemented" is unfinished; use `circle`.

### 3.3 Read the CSV

| Column | Quantity | Unit |
|---|---|---|
| `t` | time since start | s |
| `x`, `y` | position p | m |
| `qx`, `qy` | heading q (unit vector) | none |
| `v` | commanded forward speed | m/s |
| `w` | commanded angular rate | rad/s |
| `dist` | tracking error ‖p\* − p‖ | m |
| `xr`, `yr` | reference position p\* | m |

---

## 4. Record a run

1. Start the sim (section 2.1).
2. Press **Reset**.
3. Set the reference (section 3.2).
4. Press **Start** and let it run. The default horizon is 60 s.
5. Press **Stop**, then **Download CSV**.
6. Rename the file after the run, for example `circle_kperp4_trial1.csv`.

---

## 5. Change parameters

### 5.1 Change the defaults

1. Open `camslab/config/params.yaml`. Each line is commented.
2. Edit and save.
3. Restart the launch (section 2.1, step 4). No rebuild is needed.

| Parameter | Meaning |
|---|---|
| `shape` | `circle`, `gerono` or `ellipse` |
| `x0`, `y0` | reference position on the floor, m |
| `kappa` | circle curvature, 1/m (radius = 1/κ; κ = 0 is a straight line) |
| `circle_speed` | circle speed, m/s |
| `a`, `b` | Gerono or ellipse size, m |
| `period` | Gerono or ellipse lap time, s |
| `k_par`, `k_perp`, `k_q` | controller gains, all > 0 |
| `v_max`, `w_max` | command saturation limits |

### 5.2 Change a parameter during a run

1. Open a second terminal on your computer and enter the container:
   ```bash
   cd ~/camslab_ws
   docker compose -f docker/compose.yaml run --rm ros
   ```
2. Set the parameter on the car's controller node. The car is named `angostura`:
   ```bash
   ros2 param set /angostura/controller shape gerono
   ros2 param set /angostura/controller k_perp 8.0
   ```

Changing a reference parameter restarts the reference clock.

---

## 6. Test your own controller

### 6.1 Replace the built-in control law

1. Edit `control()` in `camslab/src/controller.cpp`. It takes the state (p, q) and the reference (p\*, q\*, v\*, ω\*) and returns `(v, ω)`.
2. Rebuild (section 2.1, step 3) and relaunch.

Commands are saturated at `v_max`, `w_max` and the steering limit before they reach the car. The offline Python sim has the same function in `camslab_sim/camslab_sim/controller.py`.

### 6.2 Drive the car from another program

Use this for a controller in Python, MATLAB's ROS Toolbox, or anything else that speaks ROS 2.

1. Press **Stop** in the web UI to release the car from the built-in controller.
2. Subscribe to `/angostura/pose` for the car's pose.
3. Publish `geometry_msgs/Twist` on `/angostura/cmd_vel`, with v in `linear.x` and ω in `angular.z`.

After Stop, the built-in controller sends zero briefly, then stops publishing.

### 6.3 Add cars

1. Open `camslab/config/fleet.yaml`.
2. Uncomment a car and relaunch.

> [!WARNING]
> All cars currently track the same p\*(t) and collide. Ask Sean before running more than one.

---

## 7. Troubleshoot

| Symptom | Fix |
|---|---|
| `Cannot connect to the Docker daemon` | Start Docker: `sudo systemctl start docker`. |
| `permission denied` from docker | Run `sudo usermod -aG docker $USER`, then log out and back in. |
| RViz error mentioning `display` or `xcb` | Run `xhost +local:` on your computer (section 2.1, step 1). |
| `Package 'camslab' not found` | Build and source inside the container (section 2.1, step 3). |
| Web UI does not load | Wait 20 s for the launch to finish, then reload. |
| Car does not move | Press **Start**. If it still does not move, read the red `not enabled:` line in the log. |
| Reference rejected | Make it larger or slower. |
| Anything else | Send Sean the red text from the terminal. |

---

## 8. Reference

### 8.1 Vehicle limits

- Ackermann steering: the car cannot turn in place, and ω requires v ≠ 0.
- Minimum turning radius is about 0.34 m: κ_max = tan(0.6) / 0.235 ≈ 2.9 1/m. Every reference is checked against κ_max and ω_max over the full horizon before a run.
- The car stops if its pose is older than 0.5 s.

### 8.2 Control law

$$v = \langle v^* q^* + k_\parallel (p^* - p),\ q \rangle$$
$$\omega = \omega^* + \langle k_\perp v^* (p^* - p) + k_q q^*,\ S q \rangle$$

p is the position, q the unit heading, starred quantities come from the reference, and $S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$ rotates by 90°. The speed command corrects the error along the heading; the angular rate corrects the error across it. The [README](../README.md) has the reference derivation and limit checks.

### 8.3 Offline sim

`camslab_sim/` is a Python unicycle-model sim that computes a 60 s run in about a second, for developing the reference and control code. Run it with `cd camslab_sim && uv run python -m camslab_sim` (requires `uv`). Use the Gazebo sim for research results.

### 8.4 Repository layout

| Path | Contents |
|---|---|
| `camslab/` | Gazebo sim: car model, controller, launch files |
| `camslab/config/params.yaml` | Default reference and controller parameters |
| `camslab/config/fleet.yaml` | Cars and spawn poses |
| `camslab/src/controller.cpp` | Control law (C++) |
| `camslab_sim/` | Offline sim (Python) |
| `camslab_webui/` | Web UI |
| `docker/` | Container definition |
| `README.md` | Developer documentation |
| `docs/guide.md` | This guide |
