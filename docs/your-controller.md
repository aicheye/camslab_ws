# Modify the controller

Modify `control()` to change the built-in controller while keeping the existing
reference, command limits, pose timeout, and safety-polytope checks.

For additional sensor data or a separate process, use a {doc}`custom-node` instead.

## Controller interface

At 50 Hz, the controller node gets the latest pose, evaluates the reference, and calls
`control()` from `camslab/camslab/controller.py`:

```{literalinclude} ../camslab/camslab/controller.py
:language: python
:start-at: "def control("
```

The offline simulator (`camslab_offline.tracking`) calls the same function.

### Inputs and output

| Name | Symbol | Meaning | Unit |
|---|---|---|---|
| `state.p` | $p$ | car position (rear axle centre) | m |
| `state.q` | $q$ | car heading, unit vector $(\cos\theta, \sin\theta)$ | |
| `ref.p` | $p^*$ | current reference position | m |
| `ref.q` | $q^*$ | current path direction, unit vector | |
| `ref.v` | $v^*$ | path speed | m/s |
| `ref.w` | $\omega^*$ | path turning rate | rad/s |
| `ref.kappa` | $\kappa^*$ | path curvature, $\omega^* = \kappa^* v^*$ | 1/m |
| `params.k_par`, `params.k_perp`, `params.k_q` | $k_\parallel, k_\perp, k_q$ | controller gains | |
| return `(v, w)` | $v, \omega$ | forward speed and turning rate | m/s, rad/s |

`p`, `q`, `ref.p` and `ref.q` are NumPy arrays of shape `(2,)`. `Params` also contains the
path parameters and the limits `v_max`, `w_max`, and `kappa_max`. Read
`camslab/camslab/types.py` for the complete definitions.

For $e = p^* - p$ and $S = [[0, -1], [1, 0]]$, calculate the longitudinal error with
`e @ q` and the lateral error with `e @ (S @ q)`.

## Build and test

1. Edit `camslab/camslab/controller.py`.
2. Test it offline from `camslab_offline/`:

   ```bash
   uv run pytest tests/tracking
   uv run python -m camslab_offline.tracking
   ```

3. Restart the Gazebo simulator ({doc}`running`). With `colcon build --symlink-install`,
   a Python change needs no rebuild.
4. Press **Reset**, then **Start**. Check the tracking-error plot.

Restore the original controller with `git checkout camslab/camslab/controller.py`.

## Add a parameter

Add a gain such as `k_i` in four places:

1. Add `k_i: float = 0.5` to `Params` in `camslab/camslab/types.py`.
2. Add `"k_i"` to `GAIN_FIELDS` in `camslab/camslab/controller_node.py`.
3. Add `k_i: 0.5` under the gains in `camslab/config/params.yaml`.
4. Add `"k_i"` to `PARAM_NAMES` in `camslab_webui/camslab_webui/webui_bridge.py` to show
   it in the web UI.

Use `params.k_i` in `control()`. Change it at runtime with:

```bash
ros2 param set /angostura/controller k_i 1.0
```

## Safety rules

- Saturate $|v| \le$ `v_max` and $|\omega| \le \min($`w_max`$, \kappa_{max}|v|)$.
- Reject `NaN` and infinite commands.
- Stop when the pose is older than 0.5 s.
- Stop when the robot leaves the safe area.
- Reset controller state, such as an integral term, when the run restarts.
