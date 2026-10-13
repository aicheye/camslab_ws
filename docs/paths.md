# Paths and settings

## Shapes

| `shape` | Path | Set by |
|---|---|---|
| `circle` | circle, or a line when `kappa` is 0 | `x0`, `y0`, `theta0`, `kappa`, `circle_speed` |
| `gerono` | figure eight | `x0`, `y0`, `theta0`, `a`, `b`, `period`, `phi0` |
| `ellipse` | ellipse | `x0`, `y0`, `theta0`, `a`, `b`, `period`, `phi0` |
| `rectangle` | rectangle | `x0`, `y0`, `theta0`, `a`, `b`, `period` |
| `spline` | your own smooth loop (below) | `control_points`, `period` |

In the web UI, pick a shape, type its numbers or press **Random**, and press Enter. The path restarts from its beginning.

The car steers like a normal car, so its tightest turn has a radius of about 0.34 m. A path that bends more sharply than that, or turns faster than `w_max`, still runs, but the car falls behind there and the web UI says where. The rectangle's corners are an example: the error rises after each one.

## Settings

Defaults are in `camslab/config/params.yaml`. Edit it and restart the simulator; no rebuild is needed. **Write every number with a decimal point** (`2.0`, not `2`).

| Setting | Meaning |
|---|---|
| `x0`, `y0` | circle start point, or centre of the other shapes, m |
| `theta0` | circle start heading, or the direction of side `a`, rad |
| `kappa` | circle curvature = 1 / radius, 1/m. Negative turns right. |
| `circle_speed` | circle speed, m/s |
| `a`, `b` | half-sizes along and across `theta0`, m |
| `period` | time for one lap, s |
| `phi0` | starting point on the figure eight or ellipse, rad |
| `k_par` | gain on the error along the car's heading |
| `k_perp` | gain on the error across the car's heading |
| `k_q` | gain on the heading error |
| `v_max`, `w_max` | command limits, m/s and rad/s |

To change a setting while the simulator runs, from a second terminal:

```bash
ros2 param set /angostura/controller k_perp 8.0
```

## Draw your own path (spline)

With `shape` set to `spline`, the car drives a smooth loop around a list of control points. The loop passes near the points, not through them, and stays inside the outline of the outermost points. Points closer together make tighter turns.

1. Pick `spline` and tick **edit** in the **Spline control points** box.
2. Click to add a point, drag to move one, Shift-click to remove one. At least 4 points.
3. Press **Apply**, then untick **edit**.

Points set in the web UI are lost when the simulator stops. To keep them, copy them into `params.yaml` as `x, y` pairs:

```yaml
control_points: [1.0, 0.8, 2.5, 1.2, 4.0, 0.8, 4.4, 2.5, 2.5, 3.2, 0.6, 2.5]
```
