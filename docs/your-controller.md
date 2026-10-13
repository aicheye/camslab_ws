# Your own controller

Choose one of two ways to test a controller:

| | Modify `control()` | Write a custom node |
|---|---|---|
| Language | C++ | Python, MATLAB, C++, anything with ROS 2 |
| What you write | one function | a full ROS 2 node |
| Reference $p^*$, $v^*$, $\omega^*$ | provided | compute it |
| Saturation and safe area | built in | implement them |
| Path and gain settings in the web UI | used | not used |

Modify `control()` to keep the existing safety checks. Write a separate node when the
controller needs another language or additional sensor data.

## Modify `control()`

### Overview

At 50 Hz, the controller node gets the latest pose, evaluates the reference at the current time and calls

```cpp
Command control(const State & state, const Reference & ref, const Params & params);
```

in `camslab/src/controller.cpp`. The returned command is saturated and sent to the car. This is the provided control function:

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
| `ref.p` | $p^*$ | where the car should be now | m |
| `ref.q` | $q^*$ | direction of the path now, unit vector | |
| `ref.v` | $v^*$ | path speed now | m/s |
| `ref.w` | $\omega^*$ | path turning rate now | rad/s |
| `ref.kappa` | $\kappa^*$ | path curvature now, $\omega^* = \kappa^* v^*$ | 1/m |
| `params.k_par`, `params.k_perp`, `params.k_q` | $k_\parallel, k_\perp, k_q$ | gains from `params.yaml` or the web UI | |
| return `{v, w}` | $v, \omega$ | forward speed and turning rate | m/s, rad/s |

`Params` also contains the path parameters and the limits `v_max`, `w_max` and `kappa_max`. See `camslab/include/camslab/types.hpp` for the full definitions.

### Vector helpers

`types.hpp` defines `Vec2` and `Mat2` for 2D vector maths:

| Code | Maths |
|---|---|
| `a + b`, `a - b`, `s * a`, `a / s` | vector sum, difference, scaling |
| `Vec2::dot(a, b)` | $\langle a, b \rangle$ |
| `a.norm()` | $\lVert a \rVert$ |
| `Mat2 S{0, -1, 1, 0}` | $S$, rotation by +90° |
| `Mat2::dot(S, a)` | $S a$ |
| `std::atan2(q.y, q.x)` | heading angle $\theta$ |

With $e = p^* - p$, these expressions are useful:

- longitudinal error $\langle e, q \rangle$ and lateral error $\langle e, S q \rangle$
- $\cos(\theta^* - \theta) = \langle q^*, q \rangle$ and $\sin(\theta^* - \theta) = \langle q^*, S q \rangle$

### Example: Kanayama controller

This is the Kanayama et al. (1990) tracking controller. The only difference from the provided law is that the heading term is $k_q v^* \sin(\theta^* - \theta)$ instead of $k_q \sin(\theta^* - \theta)$. It uses the same three gains.

```cpp
Command control(const State & state, const Reference & ref, const Params & params)
{
  const Mat2 S{0, -1, 1, 0};
  const Vec2 e{ref.p - state.p};
  const Vec2 Sq{Mat2::dot(S, state.q)};

  const double e_lon{Vec2::dot(e, state.q)};  // longitudinal error [m]
  const double e_lat{Vec2::dot(e, Sq)};       // lateral error [m]
  const double cos_eth{Vec2::dot(ref.q, state.q)};
  const double sin_eth{Vec2::dot(ref.q, Sq)};

  const double v{ref.v * cos_eth + params.k_par * e_lon};
  const double w{ref.w + ref.v * (params.k_perp * e_lat + params.k_q * sin_eth)};
  return {v, w};
}
```

### Building and testing

1. Rebuild from the `camslab_ws` folder:

   ```bash
   colcon build --symlink-install --packages-select camslab
   ```

2. Restart the simulator ({doc}`running`).
3. Press **Reset** and then **Start**, and check the tracking error plot.

Restore the original controller with `git checkout camslab/src/controller.cpp`, then rebuild.

### Adding a parameter

Add a new gain, such as `k_i`, as follows:

1. Add `double k_i{0.5};` to `struct Params` in `camslab/include/camslab/types.hpp`.
2. Add `{"k_i", &Params::k_i, false},` to `kParamFields` in `camslab/src/controller_node.cpp`. The `false` means changing it does not affect the path.
3. Add `k_i: 0.5` under the gains in `camslab/config/params.yaml`.
4. (Optional) Add `"k_i"` to `PARAM_NAMES` in `camslab_webui/camslab_webui/webui_bridge.py` to show it in the web UI.

Use `params.k_i` in `control()`. After rebuilding, change it at runtime with
`ros2 param set /angostura/controller k_i 1.0`.

### Notes

- The output is saturated to $|v| \le$ `v_max` and $\omega$ to $|\omega| \le \min($`w_max`$, \kappa_{\max} |v|)$.
- `NaN` or infinite commands are not sent.
- The car stops if the pose is older than 0.5 s or if it leaves the safe area.
- `control()` is stateless. Store state, such as an integral term, in a `static` variable and reset it when the run restarts.

## Write a custom node

Subscribe to the pose and publish velocity commands. Disable the built-in controller
first; otherwise both controllers publish commands.

### Topics

| Topic | Type | | Contents |
|---|---|---|---|
| `/angostura/pose` | `geometry_msgs/PoseStamped` | subscribe | position in `pose.position.x`, `.y`; heading as a quaternion in `pose.orientation`, $\theta = 2\,\mathrm{atan2}(z, w)$ |
| `/angostura/cmd_vel` | `geometry_msgs/Twist` | publish | $v$ in `linear.x`, $\omega$ in `angular.z` |

The pose is published at 100 Hz. The built-in controller publishes the reference only
while enabled, so compute the reference in the custom node.

### Steps

1. Start the simulator ({doc}`running`).
2. Press **Stop** in the web UI. The built-in controller publishes zero for 1 s and then stops publishing.
3. Run the node in another terminal. Set `use_sim_time` to use the Gazebo clock:

   ```bash
   source ~/camslab_ws/install/setup.bash
   python3 my_controller.py --ros-args -p use_sim_time:=true
   ```

4. Stop it with `Ctrl + C`. The example publishes a zero command on exit.

### Example: Python

The same control law as the built-in controller, tracking a circle with a 1 m radius:

```{literalinclude} examples/my_controller.py
:language: python
```

### Example: MATLAB

Using the ROS Toolbox:

```matlab
node = ros2node("/my_controller");
poseSub = ros2subscriber(node, "/angostura/pose", "geometry_msgs/PoseStamped");
cmdPub = ros2publisher(node, "/angostura/cmd_vel", "geometry_msgs/Twist");
cmd = ros2message(cmdPub);

rate = ros2rate(node, 50);
for k = 1:3000                      % 60 s at 50 Hz
    msg = poseSub.LatestMessage;
    if isempty(msg), waitfor(rate); continue, end
    p = [msg.pose.position.x; msg.pose.position.y];
    theta = 2 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
    % ... compute v and w from p and theta ...
    cmd.linear.x = v;
    cmd.angular.z = w;
    send(cmdPub, cmd);
    waitfor(rate);
end
cmd.linear.x = 0; cmd.angular.z = 0; send(cmdPub, cmd);
```

### Safety

```{warning}
The example does not saturate commands or check the safe area. Before running on a real
car, make the node:

- saturates $|v| \le 0.6$ m/s and $|\omega| \le \min(2, 2.9 |v|)$ rad/s, like the example
- publishes zero if the pose is older than 0.5 s
- publishes zero before exiting
```

The car keeps executing the last command it received. Publish at a fixed rate; the
built-in controller uses 50 Hz.

## Comparing controllers

Use the same path for each controller. For each controller, press **Reset** and
**Start**, run for the same duration, and download the CSV. Compare the `dist` column
for tracking error.
