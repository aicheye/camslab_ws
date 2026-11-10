# Fiducial surveying on a real car

This page adapts the fiducial surveying estimator (`docs/fiducial-hints.md`) to a real car:
a USB camera on the car, ArUco markers in the lab, and the motion capture for the robot
pose. It follows the Yahboom ROSMASTER R2 vision guide (Daniel Xie, June 2026) for the
robot side and replaces its ArUco detector and follower with the surveying pipeline:

    on the car:   USB camera -> car_node.py (OpenCV, ArUco) -> /<name>/observations, /<name>/preview
    here:         motion capture -> vrpn_mocap -> /<name>/pose
                  observations + pose -> estimator -> /<name>/estimate -> web UI, RViz

The car runs one node, which reads the camera, detects the markers and streams their
poses in the camera frame plus a low-rate JPEG preview; no image stream crosses the
network. Everything else, the data capture included, runs on your computer.

The estimator identifies the camera extrinsic `X = R_T_C` and the pose of every marker
while the car drives. Nothing has to be known about the camera mounting or the marker
placement except the height of at least one marker above the floor.

## Hardware

- A car tracked by the motion capture. The rigid body must be defined with x forward at
  the rear-axle centre and z up, because that is the robot frame of the paper. The
  RoboMaster S1 of {doc}`running` or a Yahboom ROSMASTER R2 both work; the estimator only
  needs the pose.
- A USB camera fixed to the car, anywhere, looking in any direction. Its intrinsics come
  from a calibration file in `config/cameras/`; calibrate once per camera model (below). A rolling shutter is acceptable at the lab's speeds.
- At least three printed ArUco markers of one dictionary (the default is `DICT_4X4_50`,
  ids 0, 1, 2), side 0.14 m, stood up on tripods or taped to walls at a known height. Measure
  the height of each marker's centre above the motion capture floor; the estimator needs
  at least one of them (Assumption 3 of the paper). Keep the markers well separated in
  angle so that each is seen from more than one heading. `hardware/marker_mounts/` has
  printable one-piece stands that hold a marker upright, face down toward the car,
  rolled by 45 degrees, and tilted back and rolled; its README gives the heights and the
  angles at which the R2's camera sees each.

## Software

The car runs only the camera node, `camslab/camslab/car_node.py`. It is one Python file
with no dependency on this workspace: `ros2 run camslab car` and the launch files copy it
to `/tmp/camslab_car_node.py` on the car over ssh before every start, so a change here
reaches the car without a build there. It opens the camera with OpenCV, detects the
markers in every frame and publishes `observations` (an `aruco_msgs/MarkerArray`: one
marker per accepted fiducial with its id and pose in the estimator's camera frame, stamped
at capture, empty when nothing is accepted, see [Which images are used](#which-images-are-used)),
`camera_info` and `preview/compressed`, a JPEG of the frame at `preview_rate` Hz with the
detections drawn. The motion capture client, the estimator, RViz and the web UI run on
your computer, which must be on the car's network and in the same `ROS_DOMAIN_ID` as the
car (the vision guide uses 32):

    export ROS_DOMAIN_ID=32

Put that line, and the sourcing of `/opt/ros/humble/setup.<shell>` and
`install/setup.<shell>`, in your shell's startup file once, so every terminal has them.

On the car, the node needs `python3` with `rclpy`, `numpy`, OpenCV with the aruco
module and the `aruco_msgs` package in its ROS environment. On the Yahboom R2 that is the
Jetson's `yahboom_robot` Docker container (user `jetson`), which ships with OpenCV 4.8
and aruco. The cars have no internet on the lab network (apt in the container fails at a
`10.0.0.1` proxy), so `car setup` installs a missing `aruco_msgs` by downloading the
`.deb` from packages.ros.org here and sending it over ssh; the same works for any ROS
package whose dependencies are already there:

    ros2 run camslab car install ros-humble-aruco-msgs

Here, build the workspace with `camslab` and `camslab_webui` ({doc}`install`), set
`camera_host` in `camslab/config/survey.yaml` to the car's `user@address` (`container`
and `ros_distro` name its container and ROS release), and check the way to the car:

    ros2 run camslab car setup

When a car's address is unknown (no display on the car),

    ros2 run camslab car find

scans the `/24` of `camera_host` (or `--subnet 192.168.1.0/24`) for hosts with ssh and
prints, for each, whether its ssh host key is the car image's (every car is flashed from
the same image, so they all share it), whether the login works, and its hostname and
Jetson serial number. The serial tells identical cars apart; unplugging one does too.

It accepts the car's ssh host key and installs this computer's ssh key on the car (asks
the car's password once), then checks the container, ROS, the Python modules, that the
node uploads and runs, the camera device, that the configured pixel format and image size
are among the camera's formats, and the car's clock. The clock matters: the observations
are stamped by the car and the motion capture poses by this computer, and the estimator
pairs them by time, so an offset above 20 ms fails the check. The Jetson has no
battery-backed clock, starts months behind after every power-up and drifts about 20 ms
in ten minutes, so once per car

    ros2 run camslab car chrony

makes its clock follow this computer's: here it adds `allow <car address>` and
`local stratum 10` to `/etc/chrony.conf` and a firewalld rule that accepts NTP from that
one address in the zone of the interface toward the car (nothing is opened to anyone
else); on the car's host it installs chrony, or configures `systemd-timesyncd` when
chrony cannot be installed without internet, with this computer as the only time source.
Both sides ask for sudo. It then waits for the offset to come under 20 ms and prints it.
Run it again when this computer gets another address on the lab network, since the car
is configured with that address. `ros2 run camslab car clock` sets the clock once over
ssh instead, for a quick session without chrony. Fix any `FAIL` line setup prints and run
it again. Run it once for every car, and again
for a car at a new address: `camera` and `calibrate` run over ssh without a terminal, so
they stop at once, instead of waiting for images, when the car would prompt. Every `car`
command and the launch files read `config/survey.yaml`; a value is overridden on the
command line as `name:=yahboom1`.

To see the camera alone,

    ros2 run camslab car camera

uploads and starts the camera node on the car in the foreground and prints, every 2 s,
the preview and observation rates received here and the markers seen. Ctrl-C stops both
ends.

## Checking the camera streaming in RViz

Before a survey, with the car on and a marker at hand:

    ros2 launch camslab camera_test.launch.py

starts the camera node on the car (or here, with `camera_host` empty), the
`observation_markers` node and RViz with the camera frame fixed at the origin. RViz shows
the car's preview image and, for every marker the car sees, a flat square of the marker's
size at its observed pose with its id and distance; the terminal logs the same once a
second. Hold a marker in front of the camera and check:

1. The square appears where the marker is relative to the camera: ahead is +x, left is
   +y, up is +z, and the label's distance is the tape-measure distance from the lens to
   the marker centre to within a few centimetres. Measure at 1 m and at 2 m. A distance
   off by a constant factor means a wrong `marker_size` or `camera` calibration. A
   distance that reads long by an amount that grows with the square of the distance is
   image blur, which rounds the corners inward: on rendered images with a blur of 0.8 px
   it is 4 mm at 1 m and 20 mm at 2 m.
2. The square tilts with the marker.
3. The preview runs at `preview_rate` and the observation rate in the terminal is the
   camera's frame rate. The car node logs its detection time per frame; it must stay
   below the frame period.

A marker that the preview outlines but that has no square was detected and dropped: the
preview labels it `small` or `ambiguous` in red instead of with its distance, see
[Which images are used](#which-images-are-used).

No motion capture is needed for this check.

## Which images are used

A survey estimates constants: the camera mount and the marker poses. An image without a
marker costs driving time, and an image with a wrong marker pose moves the estimate, so
the survey uses fewer images and only the ones whose pose is well determined. Nothing
fills a gap: the estimator leaves a marker alone in the images without it, and pairs an
observation with the earlier ones of the last `l_buffer` of travel, so a marker may come
and go. In RViz a square stays for 0.5 s after its last observation and the TF frame
`marker_<id>` stays where it was last seen; neither reaches the estimator.

The camera node refines the detected corners to subpixel accuracy (about 0.3 px instead
of 0.8 px on rendered images) and drops a detected marker in two cases:

- `small`: its shortest side in the image is under `min_marker_pixels`. The depth error
  is `distance * corner error / side`, and a marker of side `s` at distance `z` seen at
  the angle `a` from head-on has a shortest side of `f s cos(a) / z` pixels. With
  `f` = 523 px, `s` = 0.14 m and 30 px, a marker is used up to 2.4 m head-on and 2.1 m
  at 30 degrees, so the path has to pass every marker within about 2 m.
- `ambiguous`: a flat square has two poses that project onto almost the same corners
  when it is seen head-on or from far away, and the wrong one is tens of degrees off.
  The marker is dropped when the two poses differ by more than 5 degrees and the
  reprojection error of the second is under `ambiguity_ratio` times that of the first.

The estimator skips an image taken while the car turned faster than `max_yaw_rate`. An
error `dt` between the image time and the pose time moves a marker at distance `z` by
`z * yaw rate * dt`: 10 mm at 2 m, 0.5 rad/s and 10 ms, and a turn blurs the image.
Rotation pairs need two images at different headings, and neither has to be taken in
the turn. Every 5 s the estimator logs how many images it used and skipped, and the
camera node logs the markers per second it published and dropped for each reason.

On 1500 rendered views between 0.6 m and 3.5 m, up to 65 degrees from head-on, with a
blur of 0.8 px and pixel noise, the camera node without these checks published 96 % of
the markers with an orientation error of 17 degrees rms (152 over 15 degrees) and a
position error of 71 mm rms. With them it publishes 45 %, all within 2.4 m, with 0.7
degrees rms (3.4 at most) and 13 mm rms. These numbers are from rendered images; the
real camera's blur and noise were not measured. To measure them, park the car 1 m and
2 m from a marker and take the spread of the logged positions.

## Camera calibration

ArUco pose estimation needs the camera matrix and the distortion coefficients. The lab
calibrates with its AprilTag board: a 6 x 6 grid of 36 AprilTag 36h11 tags, ids 0 to 35,
tags of 8.8 cm with 2.64 cm between them (a Kalibr AprilGrid). The tool finds the width
of the tags' black border and the order of the ids on the board from the first images;
another 36h11 grid needs only `--cols`, `--rows`, `--tag` and `--spacing` (metres).

Calibrations are shared through the repository. `camslab/config/cameras/` holds one file
per camera model and image size, and `camera` in `config/survey.yaml` names one of them.
Cars with the same camera use the same file, so a camera model is calibrated once, by
whoever first uses it, and the file is committed. Units of one model differ by a few
percent in focal length; calibrate a car's own camera under a new name when that matters.

Check `ls camslab/config/cameras/` first. If the camera is there, set `camera` and skip to
{ref}`settings`. Otherwise:

1. Stop `survey.launch.py` if it runs, because the camera node holds the camera.
2. Run

       ros2 run camslab car calibrate --name yahboom_r2_640x480

   It starts the camera node on the car with detection off and every frame streamed as
   JPEG (quality 95), subscribes to them here and opens a window with the camera image,
   the tag corners it finds and the image rate. The terminal shows the same status
   and one line per kept view. The name is `<camera model>_<width>x<height>`.
3. Hold the board in front of the camera and move it slowly: near enough to fill most of
   the image and far enough to see the whole board, in each corner and along each edge of
   the image, flat and tilted by 20 to 45 degrees toward each side. Keep it flat. It keeps
   a view only when it differs from the ones it already has, and it is done when the
   board has been seen in each of nine areas of the image, twice each far, mid and near,
   and twice each flat and tilted left, right, up and down, with 20 views at least. The
   window shows the nine areas in its top right corner, filled as they are covered, and a
   `need:` line with what is still missing, which the terminal repeats. Press `c` in the
   window to calibrate with the views so far (5 at least) and `q` to give up. It warns
   when fewer than 5 views are tilted, because the focal length needs them.
4. Read the result. A reprojection error below 0.5 px is good; above 1 px means a bent
   board or blurred images, so repeat. The pictures in `~/camslab_calibration/<name>/`
   open by themselves:
   - `before_after.png`: the view nearest the camera, as recorded and undistorted with the
     new calibration. The board edges and the tag rows are straight on the right.
   - `distortion_map.png`: how far undistortion moves each pixel. The colour and the white
     contours give the distance in pixels, the arrows its direction, and the cross marks
     the principal point. Arrows pointing outward at the corners are barrel distortion,
     usual for a wide-angle webcam; inward is pincushion.
5. The tool copies the calibration to `camslab/config/cameras/<name>.yaml` and sets
   `camera: <name>` in `config/survey.yaml`. Commit the file so that every car with this
   camera uses it.

`--no-camera` calibrates from a camera that already streams on `preview/compressed` or
`image_raw/compressed`, for instance a `usb_cam` started by hand. Without a display the
window is skipped and `latest.png` in the report directory is refreshed every 2 s
instead. To redo a calibration from saved frames, record them with `--save views/` and
run `ros2 run camslab calibrate --name <name> --images views/`; the same tool calibrates
a camera plugged into this computer with `--device /dev/video0`.

`survey.launch.py` refuses to start when the file's image size differs from `image_width`
and `image_height`, because the intrinsics scale with the resolution.

### Skipping the calibration

To run the chain before a calibration exists, leave `camera` empty and set
`nominal_fov` in `config/survey.yaml` to the camera's horizontal field of view in degrees
(60 for a typical webcam). The camera node then builds the camera matrix from the image
size and that angle, with no distortion, and warns once at startup. Detection and the
marker directions are fine; every marker distance is scaled by the ratio of the true focal
length to the nominal one, so the estimated camera position and marker positions carry the
same scale error. Set `nominal_fov: 0` once the calibration is in place so a missing
calibration is reported instead of guessed.

(settings)=
## Settings

`camslab/config/survey.yaml` holds the lab values. Set:

| Key | Meaning |
|---|---|
| `name`, `tracker`, `mocap_server` | the car's namespace, its rigid body name and the motion capture server, as in {doc}`configuration` |
| `camera_host`, `container`, `ros_distro` | the car as `user@address`, its ROS container and release; the camera node is uploaded and started there over ssh. Empty `camera_host`: the camera node runs on this computer |
| `video_device`, `image_width`, `image_height`, `framerate`, `pixel_format` | the camera, as `v4l2-ctl --list-formats-ext` reports it |
| `camera` | the calibration: a name in `config/cameras/` or a path to a file |
| `nominal_fov` | horizontal field of view [deg] used while `camera` is empty; 0 requires a calibration |
| `camera_delay` | image latency [s]; the robot pose is taken this long before the image stamp |
| `max_yaw_rate` | the estimator skips an image taken while the car turned faster [rad/s]; 0 for no limit |
| `preview_rate` | JPEG frames per second streamed from the car for RViz and `car status --show`; 0 for none |
| `drive`, `driver_cmd_vel`, `command_timeout` | whether the path controller and the command relay run, the Twist topic of the car's driver, and the age at which the relay replaces a command by zero [s] |
| `controller` | controller parameters on top of `params.yaml`, as in {doc}`configuration`: the path, `kappa_max`, `v_max`, `w_max` |
| `dictionary`, `marker_size`, `marker_ids` | the printed markers; the camera node publishes only these ids, because floor and shelf texture decodes as a random id of a 4x4 dictionary now and then |
| `min_marker_pixels`, `ambiguity_ratio` | which detections become observations: the shortest side in the image [px] and the ratio of the two pose solutions' reprojection errors; 0 switches a check off |
| `marker_height`, `marker_height_known` | the centre heights above the floor and which of them the estimator may use |
| `stands` | where the stands are planned to stand, one per marker: the stand, x, y [m] and the facing direction [deg]. Only the rehearsal reads it |

The estimator gains and pair-selection thresholds stay in `config/fiducial.yaml`.

## Running

    ros2 launch camslab survey.launch.py

This uploads and starts the camera node on the car over ssh (with the intrinsics of
`camera` on its command line) and, here, `vrpn_mocap`, `pose_tf`, the estimator with
`sensor: aruco`, RViz with the preview image and the web UI on <http://127.0.0.1:8000>.
With `drive: true` it also starts the path controller and the command relay on the car,
see [Driving and safety](#driving-and-safety). The path is the bowtie of `survey.yaml`
and the stands go where [Rehearsal and stand layout](#rehearsal-and-stand-layout) puts
them: the camera orientation needs both pure translations and turns
(`docs/fiducial-hints.md`), and with the real camera's limits a gerono gives no pure
translation.

Watch the chain in a second terminal:

    ros2 run camslab car status --show

It redraws the rates twice a second and, with `--show`, opens the car's preview with the
markers it found and their distances. Check, from the top line down:

1. `preview` runs at `preview_rate`, `observations` at the camera rate, and
   `camera_info` says `fx ...`, not `uncalibrated`.
2. `seen` lists one marker per accepted fiducial with its distance; in RViz they are
   squares and TF frames `marker_<id>` hanging off `<name>/camera` (from the
   `observation_markers` node), and a marker straight ahead of the camera has a positive x.
3. `pose` arrives at 100 Hz with the mocap position and yaw, and the TF frame
   `<name>/mocap` moves with the car.
4. `estimate` arrives and the web UI's camera panel shows the two camera errors as NaN
   (the truth is unknown) and the dashed covariance radii appear once the laws switch on.
   The RViz text above the car shows the same radii, prefixed `r`.

The estimate is the first six values of `/<name>/estimate`: the Cayley vector `x` of the
camera orientation and `t_X`, the camera position in the robot frame. The marker poses
follow, three Cayley components and three positions per marker, in `marker_ids` order.

## Driving and safety

With `drive: true` in `survey.yaml`, `survey.launch.py` runs the path controller of
{doc}`running` on this computer and the web UI controls it: **Start** and **Stop**, the
path ({doc}`paths`) and the safe area ({doc}`safety-area`). The controller starts
disabled and drives only after **Start**. With `drive: false` nothing here commands the
car and it is driven with its own teleop.

The car's own driver must be running on the car; it is not started from here. The
controller publishes `/<name>/cmd_vel`. On the car, the command relay
(`camslab/camslab/car_drive.py`, uploaded and started over ssh like the camera node)
passes the newest command to `driver_cmd_vel`, the topic the driver listens on, 20 times
a second. Find that topic with `ros2 topic list` in the car's container while the driver
runs. The relay logs a warning every 5 s while nothing listens on it.

What stops the car:

| Event | What happens |
|---|---|
| **Stop** in the web UI | the controller sends zero |
| The path comes within `polytope_margin` (0.2 m) of the safe area's edge | the controller refuses **Start** |
| The car is outside the safe area | the controller refuses **Start** |
| The car leaves the safe area | the controller disables itself and sends zero |
| No motion capture pose for `pose_timeout` (0.5 s) | the controller sends zero |
| The WiFi, this computer or the controller fails | no command reaches the car, and the relay sends zero after `command_timeout` (0.5 s) |

The controller limits a command to `v_max` and to a turning rate of
`min(w_max, kappa_max |v|)`, and the relay applies `v_max` and `w_max` of the
`controller` block again on the car. The relay measures the age of a command on the
car's own clock, so it does not depend on the clock synchronization.

Nothing here sees obstacles or people. The safe area is a convex polygon for the
rear-axle centre: the front of the car reaches 0.2 m beyond it, and leaving it is
detected after the car crossed the edge, so keep the edge at least 0.5 m from walls and
keep every marker stand outside it. A marker is used up to about 2 m away
([Which images are used](#which-images-are-used)), so stands 0.5 m outside the edge are
in range of a path near the edge. The relay cannot stop the car if the relay itself or
the car's driver hangs; keep the car within reach.

Before the first drive of a car, and after a change of its driver:

1. Put the car on a box with the wheels off the floor and start its driver.
2. Run `ros2 run camslab car drive`. It starts the relay on the car and prints its log.
3. In a second terminal send a slow command:

       ros2 topic pub -r 20 /<name>/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

   The wheels turn forward. With `angular: {z: 0.3}` added, the front wheels steer left,
   because a positive `angular.z` is a left turn. If they steer right or stay straight,
   the driver reads `angular.z` differently from the controller, and the controller must
   not drive this car until that is resolved. If nothing moves and the relay warns that
   nothing listens, `driver_cmd_vel` is not the driver's topic.
4. Stop the command with Ctrl-C. The wheels stop within about half a second and the relay
   logs `no command for 0.5 s`.
5. Set the safe area in the web UI after every motion capture calibration, then drive the
   first lap with a hand near the car.

The default path of `survey.yaml` is a bowtie of 4.0 m by 2.0 m with a lap of 80 s. It
is 0.14 m/s on the straight edges, and its corners are arcs at the R2's steering limit of
2.9 1/m, driven at 0.42 rad/s.

## Rehearsal and stand layout

The simulated camera of `docs/fiducial-hints.md` sees every marker at any distance. The
real one accepts a marker only within about 2.4 m, inside its 63 degree view, facing it
([Which images are used](#which-images-are-used)), so each marker is in use on a few
stretches of the path only. The estimator needs, of the same marker and within 4 m of
travel, two images at the same heading 0.2 m apart (a pure-translation pair) and two
images at headings 30 degrees or more apart (a rotation pair). Whether it gets them
depends on the path and on where the stands are.

    cd camslab_standalone
    uv run python -m camslab_standalone.fiducial.rehearsal

runs the simulator with the real camera's acceptance rule, the calibration `camera`,
`max_yaw_rate`, a measurement noise of 13 mm and 0.7 degrees, the path of the
`controller` block and the planned stands of `stands` in `survey.yaml`. It prints the
images used per marker, the pairs stored, whether the two adaptive laws switch on and
the errors after each lap, and it draws the floor plan. Run it again after moving a
stand or changing the path.

```{image} images/survey-layout.png
:alt: Floor plan of the bowtie with one marker stand beyond each corner and the stretches of the path on which each marker is in use
```

What the rehearsal showed for the lab's camera:

- On a gerono no marker gives a pure-translation pair, because the heading never repeats
  while the marker is in view. The orientation law stays off and the camera's
  orientation is not identified. The bowtie's straight edges give the pure-translation
  pairs and its corners the rotation pairs.
- The stands go beyond the corners of the bowtie, facing back along the edge that leads
  to them. The two at the diagonals' ends give pure-translation pairs, the two below
  the short sides also give rotation pairs:

  | Marker | Stand | x [m] | y [m] | Facing [deg] |
  |---|---|---|---|---|
  | 0 | upright | -0.75 | 1.95 | -30 |
  | 1 | overhang | 4.75 | 1.70 | -150 |
  | 2 | diamond | -0.50 | -0.80 | 120 |
  | 3 | faceup | 4.50 | -1.05 | 120 |

  The positions are in motion capture coordinates for a bowtie centred at (2, 1) with
  `theta0: 0`; facing is the direction of the marker's normal on the floor. A stand
  misplaced by 0.15 m and 10 degrees still works.
- With this layout both laws are on by the end of the first lap. After four laps
  (320 s) the camera orientation error is 0.005 to 0.012, the camera position error 16
  to 31 mm and the marker position error 25 to 48 mm over three noise seeds.
- A safe area with the corners (-0.25, -0.05), (4.25, -0.05), (4.25, 2.05) and
  (-0.25, 2.05) holds the path with the 0.2 m margin and leaves every stand 0.5 m or
  more outside.
- A marker's orientation is estimated in Cayley coordinates, which are singular for a
  rotation by 180 degrees from the world frame. Each stand has a range of facings near
  it, roughly toward +y, which the rehearsal flags:

  | Stand | Do not face it between [deg] |
  |---|---|
  | upright | 48 and 132 |
  | overhang | 28 and 152 |
  | diamond | 3 and 87 |
  | faceup | 24 and 96 |

The rehearsal does not model lens distortion, markers hiding each other, or the
ambiguity check. Its car follows the path within 3 mm. With the turning rate disturbed
so that the heading wanders by 2 or by 4 degrees rms around the path, the errors after
four laps stay in the same range (25 and 41 mm for the marker positions). The camera's
height of 0.12 m is from `hardware/marker_mounts/README.md`; its position 0.20 m ahead
of the rear axle is assumed.

## Ground truth

The paper's errors need the true camera extrinsic and marker poses, which the lab does not
know. Two ways to get them:

- Put motion capture markers on each fiducial board and define it as a rigid body with its
  origin at the marker centre. Its pose is the true `Y_j`, and the marker position error
  `||t~_j||` follows directly. This is the Vicon integration of the vision guide's future
  work.
- Replay a recorded run through `camslab_standalone.fiducial.offline`'s batch least
  squares. The batch optimum over all observations is a reference for the online estimate
  even without a truth, and the two should agree inside the covariance radius.

Record a run with `ros2 bag record /<name>/pose /<name>/observations /<name>/estimate`.

## Frames

OpenCV's optical frame has z forward, x right and y down. The estimator's camera frame has
x forward and z up like the robot frame, so the camera node rotates every observation by
`OPTICAL_TO_BODY` before it is published (the same constant as `camslab.aruco`, which the
tests keep equal); the extrinsic `X` is estimated for that frame. The marker frame is
OpenCV's: x and y in the marker plane, z out of it.
