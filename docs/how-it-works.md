# How it works

## Car model

The controller tracks the middle of the rear axle, which moves as a unicycle:

$$
\dot p = v\, q, \qquad \dot q = \omega\, S q, \qquad S = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}
$$

$p$ is the position, $q$ the heading as a unit vector, $v$ the forward speed and $\omega$ the turning rate. The front wheels steer at most 0.6 rad on a 0.235 m wheelbase, so

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

Starred quantities are the reference, and the gains $k_\parallel$, $k_\perp$, $k_q$ (`k_par`, `k_perp`, `k_q`) are positive. Commands are limited to

$$
|v| \le v_{\max}, \qquad |\omega| \le \min(\omega_{\max},\ \kappa_{\max} |v|)
$$

The controller runs at 50 Hz and stops the car if the pose is more than 0.5 s old.

## Files

| Path | Contents |
|---|---|
| `camslab/config/params.yaml` | path and controller settings |
| `camslab/config/sim.yaml` | simulated cars and their start poses |
| `camslab/config/live.yaml` | real RoboMaster S1 and motion capture connection |
| `camslab/src/controller.cpp` | control law |
| `camslab/src/trajectory.cpp` | path shapes |
