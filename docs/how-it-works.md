# How it works

## System overview

```text
Gazebo / mocap
      │ pose
      ▼
  controller ───────────────► cmd_vel ───────────────► robot
      │
      ├── ref_path ──────────► RViz and web UI
      ├── ref_pose ──────────► RViz and web UI
      └── safety_polytope ───► RViz and web UI
```

The offline simulator uses the same reference and control concepts without ROS or
Gazebo. The web UI changes parameters and displays the resulting run; it does not replace
the controller's safety checks.

## Car model

The controller tracks the centre of the rear axle, which follows unicycle kinematics:

$$
\dot p = v\, q, \qquad \dot q = \omega\, S q, \qquad S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

where $p$ is the position, $q$ is the heading as a unit vector, $v$ is the forward speed and $\omega$ is the turning rate. The maximum steering angle is 0.6 rad and the wheelbase is 0.235 m, so

$$
|\omega| \le \kappa_{\max} |v|, \qquad \kappa_{\max} = \frac{\tan 0.6}{0.235} \approx 2.9\ \text{m}^{-1}
$$

## Control law

$$
v = \langle v^* q^* + k_\parallel (p^* - p),\ q \rangle
$$

$$
\omega = \omega^* + \langle k_\perp v^* (p^* - p) + k_q q^*,\ S q \rangle
$$

where starred quantities come from the reference and the gains $k_\parallel$, $k_\perp$, $k_q$ (`k_par`, `k_perp`, `k_q`) are positive. The commands are saturated to

$$
|v| \le v_{\max}, \qquad |\omega| \le \min(\omega_{\max},\ \kappa_{\max} |v|)
$$

The controller runs at 50 Hz. If no new pose arrives for 0.5 s, it commands zero.

## Files

| Path | Contents |
|---|---|
| `camslab/config/params.yaml` | path and controller settings |
| `camslab/config/sim.yaml` | simulated cars and their start poses |
| `camslab/config/live.yaml` | real RoboMaster S1 and motion capture connection |
| `camslab/src/controller.cpp` | control law |
| `camslab/src/trajectory.cpp` | path shapes |
