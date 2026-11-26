# Connect to the quadrotor

The Pixhawk 6C Mini on the {doc}`Q-ball quadrotor <drone>` speaks MAVLink on its USB port
and on TELEM1, where the telemetry radio is attached. This page describes both links,
the two programs that use them, and the measured quality of the radio link.

## Links

| Link | Device on the laptop | Use it for |
|---|---|---|
| USB-C cable | `/dev/ttyACM0` | Bench work: setup, calibration, parameter changes, log download |
| Telemetry radio | `/dev/ttyUSB0`, 57600 baud | Monitoring, arming, mode changes and parameter changes while the vehicle is untethered from the laptop |

MAVROS measured these rates on 2026-11-16:

| Quantity | Radio | USB |
|---|---|---|
| `/mavros/imu/data` | 5 Hz | 50 Hz |
| `/mavros/local_position/pose` | 0.5 Hz | 30 Hz |
| Request and reply round trip | 135 to 440 ms, median 250 ms | not measured |

The radio is not suitable for closed-loop control from the laptop, and the USB cable
cannot be used in flight. See [Limits of the radio](#limits-of-the-radio).

Only one program can hold a serial port. Close QGroundControl before starting MAVROS on
the same port, and the reverse.

`/dev/ttyUSB0` can change to `ttyUSB1` when another USB serial adapter is plugged in
first. This path always refers to the ground radio:

```text
/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30JUHDW-if00-port0
```

## QGroundControl

QGroundControl 5.1.5 is the ground station used for all setup on these pages.

### Install

QGroundControl for Linux is a single AppImage file:

```bash
mkdir -p ~/.local/opt/qgroundcontrol && cd ~/.local/opt/qgroundcontrol
curl -LO https://github.com/mavlink/qgroundcontrol/releases/download/v5.1.5/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```

Two things are required on the laptop:

- The user is in the `dialout` group, so that the serial ports can be opened. Add it with
  `sudo usermod -aG dialout $USER`, then log out and back in.
- `libfuse.so.2` is installed (`fuse-libs` on Fedora, `libfuse2` on Ubuntu). Without it,
  start the AppImage with `--appimage-extract-and-run`.

On the lab laptop the AppImage is linked as `qgroundcontrol` on the `PATH`.

### Connect

Plug in the USB cable or the ground radio and start QGroundControl. It connects to both
kinds of device without configuration, and the vehicle's flight mode appears in the
toolbar. A full parameter download over the radio takes noticeably longer than over USB.

If QGroundControl does not connect within about 15 seconds, ModemManager may be probing
the port. Stop it and plug the device in again:

```bash
sudo systemctl stop ModemManager
```

ModemManager was running during every successful connection so far, so this step has not
been needed.

QGroundControl 5.1.5 prints a schema warning about the actuator metadata of PX4 1.15.4
(`Unknown 'function': "center"`) and an error about geolocation access. The Actuators
page and the vehicle link work despite both messages.

### Arming checks

The toolbar shows **Not Ready** while PX4 refuses to arm. Click it to see the failed
checks. With no transmitter switched on and no position source, PX4 reported these on
2026-11-20:

| Failed check | Modes it blocks | Cause on this vehicle |
|---|---|---|
| No manual control input | Manual, Stabilized, Altitude, Acro | The transmitter is off or not bound |
| No valid position estimate | Position, Hold, Takeoff, Land, Orbit | Nothing gives the estimator a horizontal position |
| No valid global position estimate | Hold, Mission, Return | No GPS is reporting |
| No offboard signal | Offboard | Nothing is sending setpoints |

With the transmitter on and the mode switch in Stabilized, the first row clears and the
vehicle can arm. The other rows remain until a position source is added.

### Telemetry logs

QGroundControl records every MAVLink message of a session and saves the recording when it
closes:

```text
~/Documents/QGroundControl/Telemetry/<date> <time>.tlog
```

Each record is an 8-byte timestamp followed by one MAVLink packet. Over the radio a log
contains the attitude at about 6 Hz, the attitude setpoint and thrust at about 1 Hz, the
stick channels at about 1 Hz and the motor outputs at about 0.5 Hz, together with all
status messages. That is enough to examine a takeoff or a hover, and too slow to examine
an oscillation. The flight measurements in {doc}`drone` come from these logs.

PX4 also writes a full-rate `.ulg` log of every armed period to its SD card. Download it
with **Analyze Tools**, then **Log Download**.

## Telemetry radios

The two radios are from different manufacturers and run different SiK firmware versions.

| | Air radio | Ground radio |
|---|---|---|
| Hardware | SunFounder HM-TRP board, no USB | HM-TRP with an FTDI FT231X USB adapter |
| Firmware | SiK 1.7 | RFD SiK 2.0 |
| Connected to | Pixhawk TELEM1 (`/dev/ttyS5` inside PX4) | Laptop USB (`/dev/ttyUSB0`) |

Both radios use these settings:

| Setting | Value |
|---|---|
| `SERIAL_SPEED` | 57 (57600 baud) |
| `AIR_SPEED` | 64 |
| `NETID` | 25 |
| `MIN_FREQ`, `MAX_FREQ` | 915000, 928000 kHz |
| `NUM_CHANNELS` | 50 |
| `TXPOWER` | 20 (100 mW) |
| `ECC` | **0** |
| `MAVLINK` | 1 |

```{warning}
`ECC` must be 0 on both radios. The air radio was delivered with `ECC=1`. With `ECC=1`
on both, the 2.0 ground radio passes the 1.7 radio's error-corrected packets through
undecoded and nothing parses as MAVLink. With mismatched values the radios do not link.
```

The settings are stored in each radio and persist through power loss. This was checked on
2026-11-16 by unplugging both radios and observing the link return.

### Wiring the air radio

The air radio connects to TELEM1 with four wires. Screw the antennas onto both radios
before applying power, because transmitting without an antenna can damage the radio.

| TELEM1 pin | Signal | Radio wire |
|---|---|---|
| 1 (marked with a triangle) | 5 V | red |
| 2 | TX | radio RX |
| 3 | RX | radio TX |
| 6 | GND | black |

Reversed power can destroy the radio. Swapped TX and RX cause no damage and only stop
the data.

### Read or change the ground radio's settings

Open `/dev/ttyUSB0` at 57600 baud in a serial terminal, wait one second, type `+++`
without pressing Enter and wait for `OK`. Then:

| Command | Effect |
|---|---|
| `ATI` | Print the firmware version |
| `ATI5` | Print all settings |
| `ATS5=0` | Set setting number 5 (`ECC`) to 0 |
| `AT&W` | Save the settings |
| `ATZ` | Restart the radio |
| `ATO` | Leave command mode |

### Read or change the air radio's settings

The air radio has no USB port. Its settings can be reached through the Pixhawk, without
a USB serial adapter:

1. Connect the Pixhawk over USB and open the PX4 MAVLink shell (**Analyze Tools**, then
   **MAVLink Console** in QGroundControl).
2. Stop MAVLink on TELEM1 and print what the radio returns:

   ```text
   mavlink stop -d /dev/ttyS5
   cat /dev/ttyS5 &
   echo -n "+++" > /dev/ttyS5
   ```

3. The shell cannot send a carriage return, which every AT command needs. Write each
   command to a file that ends in `\r`, upload the file to `/fs/microsd` over MAVLink
   FTP, and send it with `cat /fs/microsd/<file> > /dev/ttyS5`.
4. Reboot the Pixhawk afterwards. Restarting MAVLink on TELEM1 from the shell left the
   port unable to receive.

### Measured link quality

Measured on 2026-11-16 with both radios on the bench, about -36 dBm at both ends, and the
Pixhawk powered from USB.

| Check | Result |
|---|---|
| Downlink frames in 20 s, checksum verified | 456 received, 1 missing (0.2% loss) |
| Downlink data rate | about 800 B/s |
| Uplink parameter requests | 10 of 10 answered |
| Uplink heartbeats counted by PX4 | 6 of 6 |
| MAVROS full parameter download | all parameters received, 2 requests retried |
| Round trip of a parameter read, 10 samples | 135 to 440 ms, median 250 ms |
| MAVROS time-sync round trip, link idle | 200 to 360 ms |
| MAVROS time-sync round trip, during the parameter download | 550 to 690 ms |

Damaged frames fail their checksum and are discarded, so interference appears as loss and
not as wrong values. Range, flight on battery power and one-way latency were not
measured. Loss in flight will be higher than on the bench.

### Limits of the radio

The two radios share one channel by taking turns to transmit. Each turn lasts about
114 ms with the current settings, so a request waits for one turn and its reply waits for
another. This is the cause of the 250 ms round trip.

- **Time synchronisation does not work.** MAVROS rejects the time-sync exchange because
  the round trip is over 200 ms, so timestamps received over the radio are not aligned to
  the laptop clock.
- **Closed-loop control from the laptop is not possible.** The pose arrives at 0.5 Hz.
- **PX4's native ROS 2 interface (uXRCE-DDS) needs more bandwidth** than the radio
  provides. Use MAVROS on this link.

Two changes would reduce the delay. Neither has been tried:

1. Raise `AIR_SPEED` from 64 to 250 on both radios. Each packet takes less time to send,
   so the turns become shorter, at the cost of range. `MAX_WINDOW`, the setting intended
   for this, does not exist in the air radio's SiK 1.7 firmware.
2. Use another link for control: a WiFi MAVLink bridge or a companion computer wired to
   TELEM2. `UXRCE_DDS_CFG` is set to 102 on the vehicle, which appears to select TELEM2
   for uXRCE-DDS. Nothing is connected to TELEM2 and this was not tested.

## MAVROS

MAVROS translates MAVLink into ROS 2 topics and services, and converts between PX4's
north-east-down frame and the ROS east-north-up frame. It is packaged for Ubuntu, so on
other distributions run it in Docker.

Build the image once:

```dockerfile
FROM ros:humble-ros-base
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-humble-mavros ros-humble-mavros-extras \
 && rm -rf /var/lib/apt/lists/*
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

```bash
docker build -t mavros:humble .
```

Start MAVROS on the radio:

```bash
docker run --rm -it --network none --security-opt label=disable \
  --device /dev/ttyUSB0:/dev/radio mavros:humble bash -c \
  'source /opt/ros/humble/setup.bash && ros2 launch mavros px4.launch fcu_url:=serial:///dev/radio:57600 namespace:=mavros'
```

For USB, pass `--device /dev/ttyACM0:/dev/radio` instead. `namespace:=mavros` is
required: without it, MAVROS 2.17 publishes `/state` and not `/mavros/state`.

Check the connection from a second terminal:

```bash
docker exec -it $(docker ps -q) bash -c \
  'source /opt/ros/humble/setup.bash && ros2 topic echo /mavros/state'
```

`connected: true` confirms the link. The topics most often needed are:

| Topic or service | Contents |
|---|---|
| `/mavros/state` | Connection, armed state and flight mode |
| `/mavros/imu/data` | Orientation, angular rate and acceleration |
| `/mavros/local_position/pose` | Local position and attitude |
| `/mavros/cmd/arming` | Service that arms or disarms |
| `/mavros/setpoint_position/local` | Position setpoint for Offboard mode |

Only read access was tested: state, topics, the parameter download and one acknowledged
read command on each link. Arming, mode changes and setpoints through MAVROS were not
tested.

## Reading

- [Holybro Pixhawk 6C Mini](https://docs.px4.io/v1.15/en/flight_controller/pixhawk6c_mini.html), PX4 user guide
- [SiK Radio](https://docs.px4.io/v1.15/en/telemetry/sik_radio.html), PX4 user guide
