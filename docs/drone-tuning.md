# Tune the quadrotor's controllers

PX4 controls a multicopter with nested loops: the rate controller is innermost, the
attitude controller commands it, and the velocity and position controllers command the
attitude. Tune them in that order, starting with the rate controller.

This page follows the PX4 1.15 tuning guides for the {doc}`Q-ball quadrotor <drone>`.
The gains on the vehicle on 2026-11-26 are listed in {doc}`drone`.

## State of the tuning

Autotune was run for the rate controller and for the attitude controller on 2026-11-26,
between 16:07 and 16:42, while the vehicle was flown on a tether. It replaced all rate
and attitude gains.

| Gain | Before (16:07) | After (16:49) |
|---|---|---|
| Roll rate P, I, D | 0.15, 0.2, 0.003 | 0.220, 0.248, 0.0014 |
| Pitch rate P, I, D | 0.15, 0.2, 0.003 | 0.256, 0.274, 0.0032 |
| Yaw rate P, I, D | 0.2, 0.1, 0 | 0.143, 0.171, 0.0006 |
| Roll, pitch, yaw attitude P | 4.0, 4.0, 2.8 | 4.35, 3.73, 6.5 |

With the new gains, a 34 s tethered hover in Altitude mode held roll and pitch within
1.2° (standard deviation). The gains have not been checked in free flight, and gains
identified on a tether include the tether's effect
([Tuning on a tether](#tuning-on-a-tether)).

## Before tuning

- The vehicle must lift off level. Take off in Altitude mode, or raise the throttle
  quickly in Stabilized mode ({ref}`tilt-on-takeoff`). No gain corrects that tilt.
- No motor may be at its output limit. Output 4 reached the 1900 µs limit in hover on
  2026-11-26 ({doc}`drone`). A saturated motor cannot follow the controller, so the
  response in the log no longer shows the effect of the gains.
- Flight logging must be on (`SDLOG_MODE=0` logs from arming to disarming). The telemetry
  radio is too slow for the live plots on QGroundControl's PID Tuning page, so judge each
  flight from its log.
- Change gains on the ground. Land before changing a parameter.

## Which method to use

| Situation | Method |
|---|---|
| The vehicle is on a tether, or it does not yet hover steadily | [Manual tuning](#manual-tuning) of the rate controller |
| The vehicle hovers steadily in free flight with room around it | [Autotune](#autotune), then check the result by hand |

## Tuning on a tether

The vehicle is currently flown on a tether. A tether changes what can be tuned:

- **Do not run autotune on a tether.** Autotune identifies a model of the vehicle from
  its response to its own torque commands. A taut tether adds a torque that the model
  then includes, so the resulting gains are wrong for free flight.
- Keep the hover height below the tether's length. In Altitude mode the altitude
  controller keeps raising the thrust when the tether stops a climb, so centre the stick
  once the vehicle is in the air. Altitude mode gave the cleaner takeoff on this vehicle
  ({doc}`drone-setup`).
- Attach the tether under the centre of the frame and keep it slack at hover height. A
  taut line attached below the centre of mass pulls the vehicle level, which hides a P
  gain that is too low.
- Tune only P and D of the roll and pitch rate controllers, with small stick steps of 10
  to 15 degrees. Leave the I gains and `MPC_THR_HOVER` unchanged, because both are
  distorted when the tether carries weight or pulls down.
- Treat the result as a starting point and check it again in free flight.

On a rigid test stand or a single-axis gimbal, only the free axis can be tuned, and the
friction and inertia of the stand lower the gains that are found.

## Manual tuning

### Rate controller

PX4 recommends Acro mode for tuning the rate controller, because the sticks then command
angular rates directly. Stabilized and Altitude modes also work.

Tune roll and pitch first, then yaw. For each axis:

1. Fly, give a sharp stick step, and land.
2. Change one gain by 20 to 30%. Use steps of 5 to 10% for the final adjustment.
3. Repeat until the response is fast and does not overshoot or oscillate.

| Gain | Parameters | Too low | Too high |
|---|---|---|---|
| P | `MC_ROLLRATE_P`, `MC_PITCHRATE_P`, `MC_YAWRATE_P` | The vehicle reacts slowly to input changes | High-frequency oscillation |
| D | `MC_ROLLRATE_D`, `MC_PITCHRATE_D` | Overshoot after a step input | The motors twitch and may run hot, because the D term amplifies noise |
| I | `MC_ROLLRATE_I`, `MC_PITCHRATE_I`, `MC_YAWRATE_I` | The attitude does not hold against a constant disturbance | Slow oscillation |

To test the I gain in Acro mode, tilt the vehicle about 45 degrees and hold it. If it
drifts back, raise the I gain. With `MC_ROLLRATE_K=1` and P near 0.15, PX4 gives 0.3 to
0.5 as the typical range for the roll and pitch I gains, and the pitch gain is usually a
little higher than the roll gain.

### Attitude controller

After the rate controller, adjust `MC_ROLL_P`, `MC_PITCH_P` and `MC_YAW_P`. Lower a gain
if the vehicle overshoots the commanded angle. The vehicle currently has 4.35 for roll
and 3.73 for pitch. The PX4 default is 6.5.

### Reading a flight log

PX4 writes one `.ulg` file per flight to the SD card. Download it in QGroundControl with
**Analyze Tools**, then **Log Download**. Compare the rate setpoint with the measured
rate on each axis:

- [PlotJuggler](https://github.com/facontidavide/PlotJuggler) opens `.ulg` files on the
  laptop.
- [Flight Review](https://logs.px4.io) plots the same signals, but it uploads the log to
  a public server.

## Autotune

Autotune applies small disturbances in flight, identifies a model of the vehicle and
computes the rate and attitude gains from it.

PX4 asks for free flight at 4 to 20 m above the ground in a clear space. That condition
cannot be met on a tether, and it was not checked whether the lab's flight space allows
it. The run of 2026-11-26 was made on the tether, so repeat it in free flight before
relying on the gains.

1. Take off in Altitude mode and hover.
2. Check the stability first. Tilt the vehicle about 20 degrees with the stick and
   release. It must settle within two oscillations. If it does not, land and tune the
   rate controller by hand until it does.
3. In QGroundControl open **Vehicle Configuration**, then **PID Tuning**, select the
   **Rate Controller** tab and turn on **Autotune enabled**. Press **Autotune**.
4. Wait for the test to finish. It takes about 40 seconds, between 19 and 68 seconds.
   The vehicle makes fast roll, pitch and yaw movements. The sticks still control the
   vehicle during the test. To abort it, change the flight mode.
5. Land and disarm. With `MC_AT_APPLY=1`, the new gains are applied after disarming.
6. Fly again and check for oscillation.

On 2026-11-26 the Altitude slot of the flight mode switch was in the position that also
engages the kill switch ({ref}`radio-and-modes`). Assign Altitude to another slot or move
the kill switch before starting.

## Reading

- [Auto-tuning](https://docs.px4.io/v1.15/en/config/autotune.html), PX4 user guide
- [PID Tuning (Manual/Basic)](https://docs.px4.io/v1.15/en/config_mc/pid_tuning_guide_multicopter_basic.html), PX4 user guide
- [PID Tuning Guide (Manual/Advanced)](https://docs.px4.io/v1.15/en/config_mc/pid_tuning_guide_multicopter.html), PX4 user guide
