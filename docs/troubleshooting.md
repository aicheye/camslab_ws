# Troubleshooting

## First checks

Run these commands from the repository root before investigating a specific message:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix camslab
```

If `ros2 pkg prefix camslab` fails, the workspace is not built or its setup file is not
sourced. See {doc}`install`.

| Problem | Solution |
|---|---|
| `Package 'camslab' not found` | Run `source install/setup.bash`, or build first ({doc}`install`). |
| Web UI does not load | Wait about 20 s after launching, then refresh the page. |
| Car does not move | Press **Start**. If it still does not move, read the `not enabled:` message in the terminal. |
| "kappa_max" or "w_max" warning | The path turns too tightly or too fast. Make it bigger or slower. |
| "... is not implemented" | That shape hasn't been implemented yet. Use `circle`. |
| "type" error on startup | A number in a YAML file is missing its decimal point. Use `2.0`, not `2`. |
| Path rejected with "safety polytope" | The path is too close to the edge of the safe area. Move it inward or make it smaller. |
| "left the safety polytope, disabling" | Move the car back inside the safe area and press **Start**. |
| `polytope turns right` | `camslab/config/polytope.yaml` is not convex, usually because it was edited by hand. Delete it and set the safe area again. |
| `Address already in use` or port 8000 error | Stop the other web UI, or set a different `port` in the launch config and open that port in the browser. |
| Gazebo or RViz never opens in Docker | Start the container from a graphical Ubuntu session. Check `$DISPLAY` and `$XAUTHORITY`; set `gui: false` when only the web UI is needed. |
| `rosdep` cannot resolve a dependency | Source ROS 2 Humble, run `rosdep update`, and repeat `rosdep install --from-paths . --ignore-src -y`. |
