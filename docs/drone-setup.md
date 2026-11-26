# Set up the quadrotor for flight

This page is the bench procedure for the {doc}`Q-ball quadrotor <drone>`: wiring, motor
test, ESC calibration, radio control, sensor calibration and a first hover. It ends with
the checks for the unresolved tilt on takeoff and the method used to splice the motor
wires.

All menu paths are for QGroundControl 5.1.5. Open the setup pages with the **Q** icon at
the top left, then **Vehicle Configuration**.

```{warning}
Remove all four propellers before every step on this page except the first hover. The
motor test and a failed ESC calibration can run a motor at full speed.
```

| Step | Time |
|---|---|
| [Wiring](#wiring) | 10 min per ESC |
| [Motor test](#motor-test) | 10 min |
| [ESC calibration](#esc-calibration) | 3 min |
| [Radio control and flight modes](#radio-control-and-flight-modes) | 10 min |
| [Sensor calibration](#sensor-calibration) | 5 min |
| [First hover](#first-hover) | 10 min |

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

Arm with the stick gesture: hold the throttle stick down and to the right for one second.
The motors start idling. Disarm by holding it down and to the left. This assumes that
throttle and yaw are on the left stick.

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

## First hover

1. Complete every step above with the propellers off.
2. Fit the propellers according to {doc}`drone`.
3. Tether the vehicle or fly it inside a net.
4. Arm in Stabilized mode and raise the throttle slowly until the vehicle is light on
   its feet. Lower the throttle at once if it tips or spins.
5. Hover at about 0.5 m for 30 seconds, land, and disarm.

Flying by hand needs the RC transmitter. A joystick through QGroundControl sends the
sticks over the telemetry radio, where the round trip is about 250 ms.

(tilt-on-takeoff)=
## Tilt on takeoff

On 2026-11-23 the vehicle tilted as soon as thrust came up. More throttle makes a tilting
vehicle flip, so remove the propellers before investigating. The cause has not been
identified. The candidates, in the order to test them:

| Candidate | Test, propellers off | Fix |
|---|---|---|
| Motor order or spin direction | [Motor test](#motor-test). Checked on 2026-11-23 and reported correct. | **Identify & Assign Motors**, or swap two motor wires |
| Attitude feedback has the wrong sign | Arm in Stabilized at a quarter throttle and tip the frame by hand. The motors on the side pushed down must speed up, on all four sides. Also check that the horizon in QGroundControl reads level on a flat floor and rolls right when the vehicle is tipped right. | Correct **Autopilot Orientation**, or repeat the level horizon calibration |
| Unequal ESC ranges | On the Actuators page, raise the slider that drives all motors together. All four must start at the same slider position and sound the same. | [ESC calibration](#esc-calibration), then the per-motor minimum |
| Stick input off-centre or reversed | On the Radio page, roll and pitch must sit at the centre with the sticks released, and the roll bar must move right when the stick moves right. | Repeat the radio calibration |
| Propeller on the wrong arm or upside down | Compare with the propeller table in {doc}`drone`. | Refit the propeller |
| Damaged motor, ESC or wire joint | Spin each motor by hand with the battery unplugged. Then run each at about one third. A motor that stutters, grinds, will not start or gets hot is faulty. | Move the motor's three wires to a neighbouring ESC that works. If the fault moves with the motor, replace the motor. Otherwise inspect the ESC and its solder joints. |

The motor test bypasses the attitude controller, so a correct motor test does not rule
out the second row.

The flight log settles the question without further tests. It records the command sent to
each motor and the attitude, so it shows whether one motor was driven to its limit while
its corner still dropped. Download it with **Analyze Tools**, then **Log Download**, and
open the `.ulg` file in PlotJuggler or upload it to
[Flight Review](https://logs.px4.io). Flight Review stores the log on a public server.

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
