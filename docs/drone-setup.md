# Set up the quadrotor for flight

This page is the bench procedure for the {doc}`Q-ball quadrotor <drone>`: wiring, motor
test, ESC calibration, radio control, sensor calibration and takeoff. It ends with the
explanation of the tilt on takeoff and the method used to splice the motor wires.

All menu paths are for QGroundControl 5.1.5. Open the setup pages with the **Q** icon at
the top left, then **Vehicle Configuration**.

```{warning}
Remove all four propellers before every step on this page except the takeoff. The
motor test and a failed ESC calibration can run a motor at full speed.
```

| Step | Time |
|---|---|
| [Wiring](#wiring) | 10 min per ESC |
| [Motor test](#motor-test) | 10 min |
| [ESC calibration](#esc-calibration) | 3 min |
| [Radio control and flight modes](#radio-control-and-flight-modes) | 10 min |
| [Sensor calibration](#sensor-calibration) | 5 min |
| [Takeoff and hover](#takeoff-and-hover) | 10 min |

## Wiring

Each ESC has three connections:

| Connection | Wires |
|---|---|
| Battery to ESC | Thick red to positive, thick black to negative |
| ESC to motor | Blue to A, black to B, red to C |
| ESC signal lead to the Pixhawk | Signal wire to the `S` pin, dark wire to the `−` pin of one I/O PWM OUT column |

The output assigned to each motor is listed in {doc}`drone`.

- Each output is one column of three pins marked `S`, `+` and `−`. Read the marking on
  the board to find the rows.
- The signal plug fits both ways around. If it is reversed, the ESC receives no signal
  and beeps continuously. Nothing is damaged.
- The Pixhawk does not power the ESCs and does not use the red centre wire of the signal
  lead. The battery powers the ESCs, and USB or the power module powers the Pixhawk.

### ESC beeps at power-on

These meanings are the usual behaviour of hobby PWM ESCs. They were not checked against
the manual of the Q-ball ESCs.

| Sound | Meaning |
|---|---|
| A short tune, then silence | Normal start. The ESC sees a zero-throttle signal. |
| One beep every 1 to 2 seconds | No signal. Check that the Pixhawk is powered, that the plug is on the right pins and the right way around, and that the output is assigned to a motor. |
| Fast continuous beeping | The signal is above zero throttle. |

PX4 sends pulses only on outputs that are assigned to a function. Outputs 5 to 8 and the
FMU PWM OUT group are unassigned on this vehicle and carry no signal.

## Motor test

PX4 can run one motor at a time while the vehicle is disarmed.

1. Remove the propellers. Connect the Pixhawk over USB and plug in the battery.
2. Open **Actuators**. Under **Actuator Outputs**, confirm that outputs 1 to 4 are
   assigned to motors.
3. Under **Actuator Testing**, turn on **Propellers are removed - Enable sliders**.
4. Raise one motor slider at a time. The slider returns to zero when released.
5. Compare each motor with the picture on the page: the motor that spins must be at the
   position drawn for that number, and it must turn in the direction drawn.

To fix a wrong position, press **Identify & Assign Motors** and follow the prompts. To
fix a wrong direction, swap any two of that motor's three wires.

The same test is available from the PX4 shell (**Analyze Tools**, then **MAVLink
Console**). This command runs motor 1 at 50% for 3 seconds:

```text
actuator_test set -m 1 -v 0.5 -t 3
```

PX4 converts 0 to 100% throttle into a pulse width. On this vehicle the range is 1100 to
1900 µs, so the slider at 100% reaches about 90% of an ESC range of 1000 to 2000 µs.

## ESC calibration

A PWM ESC stores which pulse width means stop and which means full throttle. ESC
calibration makes all four ESCs store the same range, so that the same command produces
the same speed on every arm. An ESC that was replaced or reset can hold a different
range, and its motor then starts later or runs slower than the others. DShot and CAN ESCs
have no stored range and do not need this step.

1. Remove the propellers.
2. Unplug the battery and keep USB connected.
3. Open **Power** and press **Calibrate** in the ESC section.
4. Plug in the battery when QGroundControl asks, and wait for the ESCs to finish beeping.

The calibration only takes effect if the battery is unplugged when **Calibrate** is
pressed and the ESCs play their confirmation beeps afterwards.

### Per-motor minimum

If one motor still starts later than the others, raise its minimum on the **Actuators**
page:

1. Turn on the sliders and move one motor's slider to its lowest running position.
2. If the motor does not spin or stutters, raise its **Minimum** field by 20 and repeat
   until it spins slowly and steadily.
3. Repeat for the other motors until all four idle at about the same speed.

PX4 has no setting that makes a weak motor produce more thrust. The flight controller
compensates for small differences between motors in flight.

(radio-and-modes)=
## Radio control and flight modes

1. Open **Radio**, press **Calibrate** and move the sticks as shown. PX4 ignores the
   sticks until a calibration is saved.
2. Open **Flight Modes**. Set **Mode Channel** to the channel of a switch on the
   transmitter. The **Channel Monitor** on that page shows which channel a switch moves.
3. Set the flight modes. For the first flights, set every slot to **Stabilized**, which
   needs no position estimate, so that no switch position can select a mode that cannot
   fly indoors.
4. Under **Switch Settings**, set **Kill switch channel** to a second switch.
5. Flip each switch and check that the toolbar shows the expected mode and that
   "Kill switch engaged" appears only in one position of the kill switch.

The kill switch stops all motors immediately. The motors restart if the switch is moved
back within 5 seconds. To reverse the direction of a switch, reverse its channel in the
transmitter's menu.

```{warning}
On 2026-11-26 the flight mode switch and the kill switch were both mapped to channel 5
(`RC_MAP_FLTMODE=5`, `RC_MAP_KILL_SW=5`). The kill switch engages above 75% of the
channel range, which is also where the Altitude slot is selected. Assign a second
transmitter switch to channel 6 or 7, calibrate the radio again, and set
**Kill switch channel** to that channel.
```

### Arming

The vehicle arms and disarms with the switch on channel 6 (`RC_MAP_ARM_SW=6`). The motors
start idling when it arms. PX4 disables the stick gesture while an arm switch is assigned.
With no arm switch assigned, hold the throttle stick down and to the right for one second
to arm, and down and to the left to disarm.

PX4 refuses to arm while the throttle stick is raised ("Arming denied: high throttle").

When arming is refused, click **Not Ready** in the toolbar to read the reason. The
common reasons are listed in {doc}`drone-connect`.

## Sensor calibration

Recalibrate after the Pixhawk is remounted or moved on the frame. Calibration does not
spin the motors.

Open **Sensors**. First check that **Autopilot Orientation** matches the mounting. It is
`ROTATION_NONE` when the arrow on the Pixhawk points to the front of the vehicle. Then
run the four calibrations in order:

1. **Compass**. Hold the vehicle in each pictured orientation and rotate it about the
   indicated axis until the picture turns green. Do this away from steel desks, power
   supplies and the laptop.
2. **Gyroscope**. Set the vehicle on a flat surface and do not touch it.
3. **Accelerometer**. Hold the vehicle still in each of the six pictured orientations.
   Hold each side square against a wall or table edge. A tilted hold biases the result.
4. **Level Horizon**. Set the vehicle level in its flying attitude.

If a step stalls over the radio, connect USB and repeat it.

### Results of 2026-11-20

Read from the telemetry log of that session.

| Step | Result |
|---|---|
| Compass | Offsets under 0.03 gauss, scales 0.990 to 1.022 |
| Gyroscope | Rates at rest under 0.06 °/s |
| Accelerometer | Scales 0.985 to 0.994 on the first accelerometer |
| Level horizon | At rest the attitude read 0.57° roll and -0.56° pitch |

Two observations from that session:

- In the tail-down step of the accelerometer calibration the vehicle was held 13° off
  vertical, which biased the X offset by about 0.13 m/s², or 0.75° of pitch. The X offset
  stored on the vehicle on 2026-11-26 differs from the one computed in that session, so
  the accelerometer has been calibrated again since.
- The yaw estimate drifted 5.7° in 43 s at rest while the gyroscope read 0.04 °/s. To
  check the heading, leave the vehicle untouched for 60 seconds and watch the compass
  heading in QGroundControl.

## Takeoff and hover

Take off in Altitude mode. In this mode the throttle stick commands a climb rate and PX4
performs the takeoff, so the stick position does not have to be precise. Altitude mode
uses the barometer and needs no position estimate.

1. Complete every step above with the propellers off.
2. Fit the propellers according to {doc}`drone`.
3. Tether the vehicle or fly it inside a net. Leave more than 1 m of slack in a tether.
4. Select Altitude mode and arm.
5. Push the throttle stick to about three quarters and hold it there. PX4 raises the
   thrust over about 3 seconds and the vehicle lifts off.
6. Centre the stick as soon as the vehicle is in the air. It then holds its height.
7. To land, lower the stick slightly below centre. Disarm after touchdown.

Measured on 2026-11-26 in two tethered flights:

- During the thrust ramp on the ground, roll and pitch moved by less than 0.4°.
- About one second after liftoff, roll and pitch moved by about 4° and recovered within
  a second.
- In a 34 s hover, the roll and pitch errors had a standard deviation of 1.2°.
- The height reported by the barometer varied between 0.25 and 1.2 m, with about ±0.5 m
  of noise. Expect the hover height to wander by that amount.
- With the stick at the bottom, the vehicle descended fast enough to bounce to 19° of
  roll at touchdown.

Three settings to review before flying in Altitude mode on a tether. They were read from
the vehicle at 16:49, before those flights, and may have been changed since:

| Parameter | Value read | Note |
|---|---|---|
| `MPC_TKO_SPEED` | 1.5 m/s | Climb speed during takeoff. 0.5 m/s suits a tether. |
| `MPC_Z_VEL_MAX_UP` | 3 m/s | Climb rate at full stick. 1 m/s suits a tether. |
| `MPC_THR_HOVER` | 0.5 | The measured hover thrust is 0.72. |

If a tether stops a climb, the altitude controller adds thrust. Centre the stick so that
the vehicle holds a height below the tether's length.

Flying by hand needs the RC transmitter. A joystick through QGroundControl sends the
sticks over the telemetry radio, where the round trip is about 250 ms.

(tilt-on-takeoff)=
## Tilt on takeoff

On 2026-11-23 the vehicle tilted as soon as thrust came up in Stabilized mode. The
telemetry logs of that day and of 2026-11-26 explain it.

### Cause

In Stabilized mode the attitude controller is active while the vehicle still stands on
the ground:

1. The vehicle rests a few degrees off level according to the estimator, while the sticks
   command a level attitude.
2. PX4 reports "Takeoff detected" once the throttle leaves the bottom of its range, at 1
   to 36% thrust in the logs. The vehicle needs about 65 to 72% to lift. From that
   message on, the rate integrator runs.
3. The vehicle cannot rotate while it stands on its feet, so the attitude error stays and
   the integrator keeps raising the motors on the low side and lowering the others.
4. When the feet leave the ground, the stored difference rotates the vehicle. A larger
   resting tilt or a slower throttle rise gives a larger rotation.

| Flight | Resting attitude | Outputs before liftoff | At liftoff |
|---|---|---|---|
| 2026-11-23 16:10, throttle held at 33 to 47% for 13 s | pitch -5.6°, roll -1.9° | front 1550 to 1730 µs, rear 1100 to 1350 µs | The nose rose 9° before the kill switch was used |
| 2026-11-26 16:14, throttle raised over 5 s | pitch -2.4°, roll -1.3° | front about 1690 µs, rear about 1390 µs | Pitch went from -2.6° to +5.8° and settled in 2 s |
| 2026-11-26 16:42 and 16:43, throttle raised in 1 s | pitch -3.3° and -0.9° | not sampled | The vehicle levelled without overshoot. Roll and pitch were within 1.7° of level during the first second in the air. |

The roll stick adds a constant command. It rests at 1491 µs against a calibrated centre
of 1503 µs, which commands -1.0° of roll.

### What was ruled out

| Candidate | Evidence |
|---|---|
| Motor order or spin direction | Checked with the [motor test](#motor-test) on 2026-11-23 |
| Attitude feedback with the wrong sign | In the log of 2026-11-23 at 16:17, the vehicle was tilted by up to 46° at a constant 36% throttle. On every axis the motors on the lowered side sped up. |
| One weak motor or one ESC with a different range | The hover outputs averaged over three sessions show a front-rear difference of about 80 µs and no motor that is consistently the highest ({doc}`drone`). A difference of that size does not produce the tilt. |

### Remedies

- Take off in Altitude mode ([Takeoff and hover](#takeoff-and-hover)). PX4 keeps treating
  the vehicle as landed during the thrust ramp, and the rate integrator does not run
  while landed.
- In Stabilized mode, raise the throttle from idle to about 70% within one second. Do not
  go far above the hover thrust of 72%, because the vehicle then climbs quickly.
- Run the level horizon calibration with the vehicle standing on a level takeoff surface,
  so that the resting attitude error is small.
- Repeat the radio calibration with the transmitter trims centred, to remove the constant
  roll command.

### Jerk after liftoff in Stabilized mode

In both fast-throttle flights of 2026-11-26, roll, pitch and yaw jumped together 1.2 to
1.3 s after liftoff, followed by a swing of 10 to 40° at about 1 Hz. The first flight
ended when the throttle was cut to 20%. The second recovered within 1.5 s at 47 to 78%
throttle. The vehicle tipped toward a different corner in each flight. The tether going
taut during the climb is the suspected cause, and this was not confirmed. The onboard
logs `21_42_24.ulg` and `21_43_41.ulg` of that day record the motion at full rate.

Do not cut the throttle when the vehicle is disturbed in the air. The motors have little
control authority at low thrust.

### If the vehicle tilts for another reason

Remove the propellers and work through these checks:

| Candidate | Test, propellers off | Fix |
|---|---|---|
| Motor order or spin direction | [Motor test](#motor-test) | **Identify & Assign Motors**, or swap two motor wires |
| Attitude feedback has the wrong sign | Arm in Stabilized at a quarter throttle and tip the frame by hand. The motors on the side pushed down must speed up, on all four sides. | Correct **Autopilot Orientation**, or repeat the level horizon calibration |
| Unequal ESC ranges | Raise the slider that drives all motors together. All four must start at the same slider position and sound the same. | [ESC calibration](#esc-calibration), then the per-motor minimum |
| Stick input off-centre or reversed | On the Radio page, roll and pitch must sit at the centre with the sticks released, and the roll bar must move right when the stick moves right. | Repeat the radio calibration |
| Propeller on the wrong arm or upside down | Compare with the propeller table in {doc}`drone`. | Refit the propeller |
| Damaged motor, ESC or wire joint | Spin each motor by hand with the battery unplugged. Then run each at about one third. A motor that stutters, grinds, will not start or gets hot is faulty. | Move the motor's three wires to a neighbouring ESC that works. If the fault moves with the motor, replace the motor. Otherwise inspect the ESC and its solder joints. |

The flight log shows the command sent to each motor and the attitude. Download it with
**Analyze Tools**, then **Log Download**, and open the `.ulg` file in PlotJuggler or
upload it to [Flight Review](https://logs.px4.io). Flight Review stores the log on a
public server.

## Splicing a motor wire

The motor wires were joined by soldering during the repair. The procedure for one joint:

1. Unplug the battery.
2. Slide a 25 mm piece of heat shrink onto one wire and park it at least 10 cm from the
   joint, so that heat conducted along the wire does not shrink it early.
3. Strip about 10 mm of insulation from both ends.
4. Fan the strands of each end apart, push the two ends into each other so that the
   strands interleave, and twist them flat into one bundle.
5. Melt a small amount of solder onto the iron tip, hold the wet tip against the
   underside of the joint, and feed solder onto the copper on the opposite side. The
   joint is done when solder has flowed through the whole bundle and looks shiny.
6. Hold the joint still for 5 seconds, then slide the heat shrink over it and shrink it.
7. Pull on the joint firmly. A good joint does not move.

Notes from the repair:

- Rosin-core electronics solder contains its own flux. Do not use plumbing flux or
  plumbing solder, which are acidic and corrode the wire.
- Motor wire is thick and draws heat away. Use a high iron setting and the side of the
  tip so that the joint takes solder within a few seconds. A long contact at low heat
  melts the insulation.
- If the heat shrink has already shrunk away from the joint, leave it in place and
  insulate the joint with three tight layers of electrical tape, secured with a small
  cable tie at each end.

## Reading

- [Actuator Configuration and Testing](https://docs.px4.io/v1.15/en/config/actuators.html), PX4 user guide
- [Safety configuration](https://docs.px4.io/v1.15/en/config/safety.html), PX4 user guide, for the kill switch and failsafes
