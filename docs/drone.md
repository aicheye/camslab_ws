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
| Wire the vehicle, test the motors, calibrate it and attempt a first hover | {doc}`drone-setup` |
| Tune the rate and attitude controllers | {doc}`drone-tuning` |

```{warning}
Remove all four propellers before any bench test that powers the ESCs. Keep a kill switch
assigned on the transmitter for every flight.
```

## How to read these pages

Each statement about the vehicle falls into one of three groups, and the text says which:

- **Measured** values were measured on the date given, with the conditions stated.
- **Read from the vehicle** means the value comes from QGroundControl's saved copy of the
  PX4 parameters. The latest copy was saved on 2026-11-26 at 16:07.
- **Not tested** means the procedure follows the PX4 documentation but has not been run
  to completion on this vehicle.

## Status on 2026-11-26

| Item | State |
|---|---|
| Telemetry radio link | Works in both directions. Measured on 2026-11-16. |
| QGroundControl | Connects over USB and over the radio. |
| MAVROS (ROS 2 Humble, in Docker) | Connects over USB and over the radio. Measured on 2026-11-16. |
| Motor outputs | Assigned to I/O PWM OUT 1 to 4. Motor order and spin direction were checked on the Actuators page on 2026-11-23. |
| Sensor calibration | Compass, gyroscope, accelerometer and level horizon completed on 2026-11-20. |
| Radio control | Sticks calibrated. Flight mode switch and kill switch are both on channel 5 (see open problems). |
| First takeoff | **Failed on 2026-11-23.** The vehicle tilted as soon as thrust came up. The cause is not identified. |
| Controller tuning | Not done. The rate gains are the values listed below. The vehicle is flown on a tether. |
| Position estimate | None. No GPS reports and motion capture is not fused, so only the manual modes can arm. |

## Hardware

| Part | Description |
|---|---|
| Flight controller | Holybro Pixhawk 6C Mini, PX4 1.15.4 |
| Airframe setting | Generic quadcopter, `SYS_AUTOSTART=4001`, X geometry |
| Propellers | Two 10x4.7 and two 10x4.7P |
| Battery | 3 cells (`BAT1_N_CELLS=3`). PX4 reported 11.50 V on 2026-11-20. |
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

Read from the vehicle on 2026-11-26. Values not listed were not inspected.

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
| `RC_KILLSWITCH_TH` | 0.75 | The kill switch engages above 75% of the channel range |
| `COM_FLTMODE1`, `COM_FLTMODE6` | 8, 1 | Stabilized in the lowest switch position, Altitude in the highest. Slots 2 to 5 are unassigned. |
| `MAV_0_CONFIG`, `SER_TEL1_BAUD` | 101, 57600 | MAVLink on TELEM1 at 57600 baud |
| `MAV_0_RATE` | 1200 B/s | MAVLink data rate on TELEM1 |
| `EKF2_EV_CTRL` | 0 | External vision (motion capture) is not fused |
| `MC_ROLLRATE_P`, `I`, `D` | 0.15, 0.2, 0.003 | Roll rate gains |
| `MC_PITCHRATE_P`, `I`, `D` | 0.15, 0.2, 0.003 | Pitch rate gains |
| `MC_YAWRATE_P`, `I`, `D` | 0.2, 0.1, 0 | Yaw rate gains |
| `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P` | 4.0, 4.0, 2.8 | Attitude gains |
| `IMU_GYRO_CUTOFF`, `IMU_DGYRO_CUTOFF` | 40 Hz, 20 Hz | Gyro and D-term low-pass filters |
| `MPC_THR_HOVER`, `MPC_USE_HTE` | 0.5, 1 | Hover thrust and the hover thrust estimator |
| `MC_AT_EN`, `MC_AT_APPLY` | 1, 1 | Autotune is enabled and applies its gains after disarm |
| `SDLOG_MODE` | 0 | Flight logging to the SD card is on |

To read the current values, open **Vehicle Configuration**, then **Parameters** in
QGroundControl.

## Open problems

1. **The vehicle tilts on takeoff.** On 2026-11-23, in Stabilized mode, the vehicle tilted
   as soon as thrust came up. Motor order, spin direction and ESC calibration were
   checked afterwards. The remaining candidates and the test for each are in
   {ref}`tilt-on-takeoff`. The flight log from that attempt has not been examined.
2. **The kill switch shares channel 5 with the flight mode switch.** With the values
   above, the switch position that selects Altitude mode also engages the kill switch, so
   Altitude mode cannot be flown from the transmitter. Move one of them to another
   channel ({ref}`radio-and-modes`).
3. **The heading estimate was not confirmed stable.** After the calibration on
   2026-11-20 the yaw estimate drifted 5.7° in 43 s at rest, and PX4 reported
   "heading estimate not stable" while the vehicle was handled. No longer rest period was
   recorded.
4. **The telemetry radio is too slow for control.** The round trip is about 250 ms and
   the pose arrives at 0.5 Hz ({doc}`drone-connect`). Closed-loop control from a laptop
   needs a different link.
5. **There is no position source.** Position, Hold and Offboard modes need motion capture
   fused as external vision or a GPS. Neither is configured.
