# Troubleshooting

| Message | Fix |
|---|---|
| `Package 'camslab' not found` | Run `source install/setup.bash`, or build first ({doc}`install`). |
| Web UI does not load | Wait 20 s after starting, then reload. |
| Car does not move | Press **Start**. If it still does not move, read the `not enabled:` line in the terminal. |
| "kappa_max" or "w_max" | The path is too sharp or too fast. Make it larger or slower. |
| "... is not implemented" | That shape is not written yet. Use `circle`. |
| A "type" error at start | A YAML number has no decimal point. Write `2.0`, not `2`. |
| "safety polytope" | The path comes too close to the safe area's edge. Move it in or make it smaller. |
| "left the safety polytope, disabling" | Bring the car back inside, then press **Start**. |
| `polytope turns right` | `camslab/config/polytope.yaml` was edited by hand. Delete it and redraw the area. |

Anything else: send Sean the red text from the terminal.
