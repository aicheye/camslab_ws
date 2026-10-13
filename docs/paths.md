# Paths and settings

## Shapes

| `shape` | Path | Set by |
|---|---|---|
| `circle` | circle, or a line when `kappa` is 0 | `x0`, `y0`, `theta0`, `kappa`, `circle_speed` |
| `gerono` | figure eight | `x0`, `y0`, `theta0`, `a`, `b`, `period`, `phi0` |
| `ellipse` | ellipse | `x0`, `y0`, `theta0`, `a`, `b`, `period`, `phi0` |
| `rectangle` | rectangle | `x0`, `y0`, `theta0`, `a`, `b`, `period` |
| `spline` | custom closed curve (see below) | `control_points`, `period` |

Change the path in the web UI by selecting a shape, entering its parameters (or pressing
**Random**), and pressing Enter. The path restarts from the beginning.

The car has Ackermann steering and a minimum turning radius of about 0.34 m. A tighter
path or a path faster than `w_max` still runs, but the car cannot follow those sections
exactly. Read the warning in the web UI. Expect the tracking error to jump at each
rectangle corner.

## Settings

Edit the default parameters in `camslab/config/params.yaml`, then restart the simulator.
Do not rebuild. Write every number with a decimal point: `2.0`, not `2`.

| Setting | Meaning |
|---|---|
| `x0`, `y0` | circle start point, or centre of the other shapes, m |
| `theta0` | circle start heading, or the direction of side `a`, rad |
| `kappa` | circle curvature = 1 / radius, 1/m. Negative turns right. |
| `circle_speed` | circle speed, m/s |
| `a`, `b` | half-lengths along and perpendicular to `theta0`, m |
| `period` | time for one lap, s |
| `phi0` | starting phase on the figure eight or ellipse, rad |
| `k_par` | gain on the error along the car's heading |
| `k_perp` | gain on the error across the car's heading |
| `k_q` | gain on the heading error |
| `v_max`, `w_max` | command limits, m/s and rad/s |

Change a parameter while the simulator is running:

```bash
ros2 param set /angostura/controller k_perp 8.0
```

Changing a trajectory parameter restarts the reference clock and republishes the path.
Changing a gain or command limit takes effect without changing the path. A parameter
change that violates `kappa_max` or `w_max` is accepted with a warning, but the command
still respects the configured saturation limits.

## Draw a custom path (spline)

The `spline` shape is a closed cubic B-spline defined by a list of control points. The curve does not pass through the control points, but it always stays inside their convex hull. Placing points closer together makes tighter turns.

```{image} images/spline.png
:alt: Editing spline control points in the web UI
```

1. Select `spline` and check **edit** in the **Spline control points** box.
2. Click the map to add a point, drag a point to move it, and Shift-click a point to remove it. Add at least 4 points.
3. Press **Apply**, then uncheck **edit**.

The web UI does not save control points when the simulator stops. Save them in
`params.yaml` as a flat list of `x, y` pairs:

```yaml
control_points: [1.0, 0.8, 2.5, 1.2, 4.0, 0.8, 4.4, 2.5, 2.5, 3.2, 0.6, 2.5]
```
