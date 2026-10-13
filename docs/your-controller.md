# Modify the controller

Modify `control()` to change the built-in C++ controller while keeping the existing
reference, command limits, pose timeout, and safety-polytope checks.

For a different language or additional sensor data, use a {doc}`custom-node` instead.

## Controller interface

At 50 Hz, the node gets the latest pose, evaluates the reference, and calls:

```cpp
Command control(const State & state, const Reference & ref, const Params & params);
```

The implementation is in `camslab/src/controller.cpp`:

```{literalinclude} ../camslab/src/controller.cpp
:language: cpp
:start-at: "Command control("
:end-before: "}  // namespace camslab"
```

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
| return `{v, w}` | $v, \omega$ | forward speed and turning rate | m/s, rad/s |

`Params` also contains path parameters and the limits `v_max`, `w_max`, and `kappa_max`.
Read `camslab/include/camslab/types.hpp` for the complete definitions.

### Vector helpers

Use the `Vec2` and `Mat2` helpers from `types.hpp`:

| Code | Maths |
|---|---|
| `a + b`, `a - b`, `s * a`, `a / s` | vector sum, difference, and scaling |
| `Vec2::dot(a, b)` | $\langle a, b \rangle$ |
| `a.norm()` | $\lVert a \rVert$ |
| `Mat2 S{0, -1, 1, 0}` | rotation by +90° |
| `Mat2::dot(S, a)` | $S a$ |
| `std::atan2(q.y, q.x)` | heading angle $\theta$ |

For $e = p^* - p$, calculate longitudinal error with $\langle e, q \rangle$ and lateral
error with $\langle e, S q \rangle$.

## Build and test

1. Edit `camslab/src/controller.cpp`.
2. Rebuild the package:

   ```bash
   colcon build --symlink-install --packages-select camslab
   ```

3. Restart the simulator ({doc}`running`).
4. Press **Reset**, then **Start**. Check the tracking-error plot.

Restore the original controller with `git checkout camslab/src/controller.cpp`, then
rebuild.

## Add a parameter

Add a gain such as `k_i` in four places:

1. Add `double k_i{0.5};` to `struct Params` in `camslab/include/camslab/types.hpp`.
2. Add `{"k_i", &Params::k_i, false},` to `kParamFields` in
   `camslab/src/controller_node.cpp`.
3. Add `k_i: 0.5` under the gains in `camslab/config/params.yaml`.
4. Add `"k_i"` to `PARAM_NAMES` in `camslab_webui/camslab_webui/webui_bridge.py` to show
   it in the web UI.

Use `params.k_i` in `control()`. After rebuilding, change it at runtime:

```bash
ros2 param set /angostura/controller k_i 1.0
```

## Safety rules

- Saturate $|v| \le$ `v_max` and $|\omega| \le \min($`w_max`$, \kappa_{max}|v|)$.
- Reject `NaN` and infinite commands.
- Stop when the pose is older than 0.5 s.
- Stop when the robot leaves the safe area.
- Reset controller state, such as an integral term, when the run restarts.
