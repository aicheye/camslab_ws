# Q-ball quadrotor

The lab's Q-ball quadrotor was repaired in November 2026 and refitted with a Holybro
Pixhawk 6C Mini running PX4 1.15.4. These pages record how the vehicle is wired and
configured, how to connect a computer to it, how to prepare it for flight, and what has
been measured on it.

No package in this repository controls the quadrotor yet. Every procedure here uses
QGroundControl, MAVROS or the PX4 shell.

| Goal | Page |
|---|---|
| Connect a laptop over USB or the telemetry radio, with QGroundControl or ROS 2 | {doc}`drone-connect` |
| Wire the vehicle, test the motors, calibrate it and take off | {doc}`drone-setup` |
| Tune the rate and attitude controllers | {doc}`drone-tuning` |

```{warning}
Remove all four propellers before any bench test that powers the ESCs. Keep a kill switch
assigned on the transmitter for every flight.
```

## How to read these pages

Each statement about the vehicle falls into one of three groups, and the text says which:

- **Measured** values were measured on the date given, with the conditions stated. Flight
  measurements come from the telemetry logs that QGroundControl records over the radio
  ({doc}`drone-connect`).
- **Read from the vehicle** means the value comes from QGroundControl's saved copy of the
  PX4 parameters. The latest copy was saved on 2026-11-26 at 16:49.
- **Not tested** means the procedure follows the PX4 documentation but has not been run
  to completion on this vehicle.

## Status on 2026-11-26

| Item | State |
|---|---|
| Telemetry radio link | Works in both directions. Measured on 2026-11-16. |
| QGroundControl | Connects over USB and over the radio. |
| MAVROS (ROS 2 Humble, in Docker) | Connects over USB and over the radio. Measured on 2026-11-16. |
| Motor outputs | Assigned to I/O PWM OUT 1 to 4. Order, spin direction and the sign of the attitude feedback are correct. |
| Sensor calibration | Compass, gyroscope, accelerometer and level horizon completed on 2026-11-20. |
| Radio control | Sticks calibrated. Arm switch on channel 6. Flight mode switch and kill switch are both on channel 5 (see open problems). |
| Takeoff and hover | **Works in Altitude mode on a tether.** A 34 s hover on 2026-11-26 held roll and pitch within 1.2° (standard deviation). |
| Tilt on takeoff | Explained. It occurs in Stabilized mode when the throttle is raised slowly ({ref}`tilt-on-takeoff`). |
| Controller gains | Set by autotune on 2026-11-26 while the vehicle was tethered. They have not been checked in free flight ({doc}`drone-tuning`). |
| Position estimate | None. No GPS reports and motion capture is not fused, so only the manual modes can arm. |

## Hardware

| Part | Description |
|---|---|
| Flight controller | Holybro Pixhawk 6C Mini, PX4 1.15.4 |
| Airframe setting | Generic quadcopter, `SYS_AUTOSTART=4001`, X geometry |
| Propellers | Two 10x4.7 and two 10x4.7P |
| Battery | 3 cells (`BAT1_N_CELLS=3`) |
| ESCs | Four PWM ESCs on I/O PWM OUT 1 to 4, 400 Hz |
| Radio control | Receiver that reports 7 channels |
| Telemetry | 915 MHz SiK radio pair, 57600 baud, air radio on TELEM1 |
| Ground station | QGroundControl 5.1.5 |

PX4 calls the I/O PWM OUT group "MAIN" in its parameter names (`PWM_MAIN_*`).

### Motors and outputs

Read from the vehicle (`CA_ROTOR*` and `PWM_MAIN_FUNC*`). Positions are relative to the
arrow on the Pixhawk, and directions are seen from above. A positive `CA_ROTORn_KM` means
counter-clockwise.

| Motor | Position | Direction | Output |
|---|---|---|---|
| 1 | front right | counter-clockwise | I/O PWM OUT 1 |
| 2 | rear left | counter-clockwise | I/O PWM OUT 3 |
| 3 | front left | clockwise | I/O PWM OUT 4 |
| 4 | rear right | clockwise | I/O PWM OUT 2 |

Each ESC connects to its motor with blue on A, black on B and red on C. Swapping any two
of the three wires reverses that motor.

### Propellers

| Arm | Propeller |
|---|---|
| Arm with the barcode sticker, and the arm opposite it | 10x4.7P |
| Arm with the IP address label, and the arm opposite it | 10x4.7 |

The "P" propellers have the reverse pitch, so they belong on the two motors that turn the
other way from the plain 10x4.7 pair. Which pair of arms carries the clockwise motors was
not recorded. Before fitting a propeller, run its motor from the Actuators page and check
that the raised edge of each blade leads the rotation and that the printed side faces up.

## Configuration

Read from the vehicle on 2026-11-26 at 16:49. Values not listed were not inspected.

| Parameter | Value | Meaning |
|---|---|---|
| `SYS_AUTOSTART` | 4001 | Generic quadcopter |
| `SENS_BOARD_ROT` | 0 | Pixhawk arrow points to the front of the vehicle |
| `PWM_MAIN_MIN1` to `4` | 1100 µs | Lowest command while armed |
| `PWM_MAIN_MAX1` to `4` | 1900 µs | Highest command |
| `PWM_MAIN_DIS1` to `4` | 1000 µs | Command while disarmed |
| `CBRK_IO_SAFETY` | 22027 | The safety switch is disabled |
| `RC_MAP_ROLL`, `PITCH`, `THROTTLE`, `YAW` | 1, 2, 3, 4 | Stick channels |
| `RC_MAP_FLTMODE` | 5 | Flight mode switch channel |
| `RC_MAP_KILL_SW` | 5 | Kill switch channel |
| `RC_MAP_ARM_SW` | 6 | Arm switch channel. The stick gesture is disabled while an arm switch is assigned. |
| `RC_KILLSWITCH_TH` | 0.75 | The kill switch engages above 75% of the channel range |
| `COM_FLTMODE1`, `COM_FLTMODE6` | 8, 1 | Stabilized in the lowest switch position, Altitude in the highest. Slots 2 to 5 are unassigned. |
| `MAV_0_CONFIG`, `SER_TEL1_BAUD` | 101, 57600 | MAVLink on TELEM1 at 57600 baud |
| `MAV_0_RATE` | 1200 B/s | MAVLink data rate on TELEM1 |
| `EKF2_EV_CTRL` | 0 | External vision (motion capture) is not fused |
| `MC_ROLLRATE_P`, `I`, `D` | 0.220, 0.248, 0.0014 | Roll rate gains |
| `MC_PITCHRATE_P`, `I`, `D` | 0.256, 0.274, 0.0032 | Pitch rate gains |
| `MC_YAWRATE_P`, `I`, `D` | 0.143, 0.171, 0.0006 | Yaw rate gains |
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | 4.35, 3.73, 6.5 | Attitude gains |
| `MC_RR_INT_LIM`, `MC_PR_INT_LIM` | 0.3, 0.3 | Limits of the roll and pitch rate integrators |
| `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF` | 40 Hz, 20 Hz | Gyro and D-term low-pass filters |
| `MPC_THR_HOVER`, `MPC_USE_HTE` | 0.5, 1 | Hover thrust and the hover thrust estimator. The measured hover thrust is 0.72. |
| `MPC_TKO_RAMP_T`, `MPC_TKO_SPEED` | 3 s, 1.5 m/s | Thrust ramp time and climb speed of a takeoff in Altitude mode |
| `MPC_Z_VEL_MAX_UP` | 3 m/s | Climb rate at full stick in Altitude mode |
| `MC_AT_EN`, `MC_AT_APPLY` | 1, 1 | Autotune is enabled and applies its gains after disarm |
| `SDLOG_MODE` | 0 | Flight logging to the SD card is on |

The copy saved on the same day at 16:07 held different controller gains: 0.15, 0.2 and
0.003 for the roll and pitch rate controllers, 0.2, 0.1 and 0 for yaw rate, and 4.0, 4.0
and 2.8 for attitude. Autotune replaced them before 16:42 ({doc}`drone-tuning`).

To read the current values, open **Vehicle Configuration**, then **Parameters** in
QGroundControl.

## Flight measurements on 2026-11-26

Measured from the telemetry logs of tethered flights indoors. The radio delivers attitude
at about 6 Hz and motor outputs at about 0.5 Hz, so fast motion is not resolved.

### Hover in Altitude mode

One flight of 34 s, with small stick inputs during the hover.

| Quantity | Value |
|---|---|
| Roll error | standard deviation 1.2°, maximum 5.8° |
| Pitch error | standard deviation 1.2°, maximum 6.5° |
| Roll and pitch rate | standard deviation 5 to 7 °/s |
| Thrust | mean 0.72, range 0.59 to 0.76 |
| Height by barometer | 0.25 to 1.2 m above the start, with about ±0.5 m of noise |
| Battery voltage | 11.8 V at rest, 10.8 V minimum in flight |

### Motor outputs in hover

| Quantity | Value |
|---|---|
| Hover thrust | 0.71 to 0.75 in every flight |
| Front pair minus rear pair, mean per session | +73, +81 and +89 µs |
| Right pair minus left pair, mean per session | -40, +6 and +8 µs |
| Clockwise pair minus counter-clockwise pair, mean per session | -58, +36 and +54 µs |
| Highest output observed | 1900 µs on output 4 (front left), which is the limit |
| Battery warnings | "Low battery" 10 and 31 s after takeoff in two flights, "Critical battery" after 18 s in one. The same warnings appeared on 2026-11-23. |

The session means are taken over hover samples with the yaw stick centred, in three
sessions of 10 to 31 samples each. The front motors run about 80 µs higher than the rear
motors in every session, so the vehicle needs a constant nose-up correction. A centre of
mass ahead of the frame centre, weaker front motors or the tether would each produce
this, and the cause was not investigated. The yaw correction changes sign between
sessions.

## Open problems

1. **One motor reaches its output limit.** Output 4 reached 1840 to 1900 µs in hover and
   the limit is 1900 µs. Hover takes 72% thrust, and the battery drops by about 1 V under
   load. A saturated motor limits attitude control and makes tuning results unreliable.
2. **The controller gains were set on a tether.** Autotune was run for the rate and
   attitude controllers on 2026-11-26 during tethered flights. Autotune on a tether
   identifies a model that includes the tether, so check the gains in free flight
   ({doc}`drone-tuning`).
3. **The kill switch shares channel 5 with the flight mode switch.** With the values
   above, the switch position that selects the Altitude slot also engages the kill
   switch. Move one of them to another channel ({ref}`radio-and-modes`).
4. **A 4° bump follows each takeoff in Altitude mode.** About one second after the feet
   leave the ground, roll moves by about -4° and pitch by about +4°, and both recover
   within a second. It had the same direction in both flights. The cause is unknown.
5. **The heading estimate was not confirmed stable.** After the calibration on
   2026-11-20 the yaw estimate drifted 5.7° in 43 s at rest, and PX4 reported
   "heading estimate not stable" while the vehicle was handled.
6. **The telemetry radio is too slow for control.** The round trip is about 250 ms and
   the pose arrives at 0.5 Hz ({doc}`drone-connect`). Closed-loop control from a laptop
   needs a different link.
7. **There is no position source.** Position, Hold and Offboard modes need motion capture
   fused as external vision or a GPS. Neither is configured.
