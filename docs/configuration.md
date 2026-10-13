# Configuration reference

Set shared controller values in `camslab/config/params.yaml`. Set simulator values in
`camslab/config/sim.yaml` and real-robot values in `camslab/config/live.yaml`.

## Load order

The launch files load configuration in this order:

1. `params.yaml` supplies the shared trajectory, gains, and limits.
2. `sim.yaml` or `live.yaml` adds robot-specific launch settings and a `controller:` block.
3. A `polytope:=...` file, when present, adds the saved safety area.
4. A launch argument with the same name overrides a top-level YAML value.

Use a copy of a YAML file for local experiments:

```bash
ros2 launch camslab gazebo_multi.launch.py config:=/path/to/sim.yaml
```

Restart the simulator after editing a file. Runtime parameter changes do not require a
rebuild.

## Shared controller parameters

| Parameter | Meaning | Default |
|---|---|---:|
| `shape` | `circle`, `gerono`, `ellipse`, `rectangle`, or `spline` | `circle` |
| `x0`, `y0` | circle start point or shape centre [m] | `2.0`, `1.0` |
| `theta0` | circle heading or shape axis [rad] | `0.0` |
| `circle_speed` | circle speed [m/s] | `0.3` |
| `kappa` | circle curvature [1/m]; zero gives a straight line | `1.0` |
| `a`, `b` | shape half-axes or rectangle half-sides [m] | `2.0`, `1.0` |
| `period` | lap duration for non-circles [s] | `40.0` |
| `phi0` | Gerono or ellipse starting phase [rad] | `0.0` |
| `control_points` | flat `[x0, y0, ...]` spline list; at least 4 points | see YAML |
| `t_max` | reference and limit-check duration [s] | `60.0` |
| `k_par`, `k_perp`, `k_q` | controller gains | `1.0`, `4.0`, `2.0` |
| `v_max` | forward-speed limit [m/s] | `0.6` |
| `w_max` | turning-rate limit [rad/s] | `2.0` |
| `pose_timeout` | stale-pose timeout [s] | `0.5` |
| `rate` | controller frequency [Hz] | `50.0` |

Write numeric values with a decimal point, such as `2.0` rather than `2`.

Change a value while Gazebo is running:

```bash
ros2 param set /angostura/controller k_perp 8.0
```

Trajectory changes restart the reference clock and republish the path. Gain and limit
changes preserve the path. A trajectory beyond `kappa_max` or `w_max` is accepted with a
warning, but commands remain saturated.

## Gazebo settings

Set these top-level values in `sim.yaml`:

| Parameter | Meaning | Default |
|---|---|---:|
| `gui` | open the Gazebo window | `false` |
| `rviz` | launch RViz | `true` |
| `webui` | launch the web UI | `true` |
| `port` | web UI port | `8000` |
| `start_enabled` | enable controllers after a fresh pose arrives | `true` |
| `controller.kappa_max` | R2 curvature limit [1/m] | `2.9` |

Each `robots` entry sets `name`, `x`, `y`, optional `yaw`, and optional RViz `color`.
The first robot supplies the web UI's default plot. Multiple robots currently follow the
same reference, so use separate references before enabling a fleet experiment.

## Live-robot settings

Set these values in `live.yaml`:

| Parameter | Meaning | Default |
|---|---|---|
| `name` | S1 namespace and TF prefix | `angostura` |
| `serial_number` | S1 intelligent-controller serial number | lab-specific |
| `conn_type` | `sta`, `ap`, or `rndis` | `sta` |
| `chassis_timeout` | stale-command timeout [s] | `0.5` |
| `mocap_server`, `mocap_port` | VRPN server address and port | lab-specific |
| `tracker` | mocap rigid-body name; empty uses `name` | empty |
| `start_enabled` | enable immediately after launch | `false` |
| `controller.kappa_max` | S1 controller curvature limit [1/m] | `2.9` |

Leave `start_enabled: false` for manual startup. Confirm the mocap pose before pressing
**Start**.

## Safety-polytope settings

The web UI saves the polygon to `config/polytope.yaml` by default. Override the file with
the launch argument:

```bash
ros2 launch camslab gazebo_multi.launch.py polytope:=/path/to/polytope.yaml
```

The file contains map-frame, counter-clockwise, convex coordinates and a margin:

```yaml
/**/controller:
  ros__parameters:
    polytope: [x1, y1, x2, y2, ...]
    polytope_margin: 0.2
```

Clear the polygon in the web UI to remove the file. Reset it after each mocap calibration.
