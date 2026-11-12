# Fiducial surveying on a real car

This page adapts the fiducial surveying estimator (`docs/fiducial-hints.md`) to a real car:
a USB camera on the car, ArUco markers in the lab, and the motion capture for the robot
pose. It follows the Yahboom ROSMASTER R2 vision guide (Daniel Xie, June 2026) for the
robot side and replaces its ArUco detector and follower with the surveying pipeline:

```{image} images/survey-system.svg
:alt: The camera node and the command relay run on the car; the estimator, the motion capture client, the controller, RViz and the web UI run on this computer. Observations and the preview go from the car to this computer over the lab WiFi, cmd_vel goes back.
:width: 100%
```

The topics are in the namespace `/<name>`; `driver_cmd_vel` is the topic of the car's
own driver. The car runs two uploaded files. The camera node reads the camera, detects
the markers and streams their poses in the camera frame plus a low-rate JPEG preview; no
image stream crosses the network. The command relay passes the controller's commands to
the car's driver and stops the car when they stop arriving
([Driving and safety](#driving-and-safety)); it runs only with `drive: true`. Everything
else, the data capture included, runs on your computer.

The estimator identifies the camera extrinsic `X = R_T_C` and the pose of every marker
while the car drives. Nothing has to be known about the camera mounting or the marker
placement except the height of at least one marker above the floor.

## Hardware

- A car tracked by the motion capture. The rigid body must be defined with x forward at
  the rear-axle centre and z up, because that is the robot frame of the paper. The
  RoboMaster S1 of {doc}`running` or a Yahboom ROSMASTER R2 both work; the estimator only
  needs the pose.
- A USB camera fixed to the car, anywhere, looking in any direction. Its intrinsics come
  from a calibration file in `config/cameras/`; calibrate once per camera model (below).
  A rolling shutter is acceptable at the lab's speeds.
- Two or more printed ArUco markers of `DICT_4X4_50`, side 0.14 m, on the printable
  stands of `hardware/marker_mounts/`, whose README gives each stand's marker height.
  The estimator needs the height of at least one marker's centre above the motion
  capture floor (Assumption 3 of the paper). `marker_ids` in `survey.yaml` lists the
  markers in the room.

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
`name`, `camera_host` (the car's `user@address`) and `camera` in
`camslab/config/survey.yaml`, and check the way to the car:

    ros2 run camslab car setup

It accepts the car's ssh host key, creates an ssh key for this computer if it has none
and installs it on the car (asks the car's password once), then checks the container,
ROS, the Python modules, that the node uploads and runs, the camera device, that the
pixel format and image size are among the camera's formats, and the car's clock. Fix
any `FAIL` line it prints and run it again. Run it once for every car and computer, and
again for a car at a new address: the launch files run over ssh without a terminal, so
they stop at once when the car would prompt.

The clock matters: the observations are stamped by the car and the motion capture poses
by this computer, and the estimator pairs them by time, so an offset above 20 ms fails
the check. The Jetson has no battery-backed clock, starts months behind after every
power-up and drifts about 20 ms in ten minutes. Two ways to set it:

- `ros2 run camslab car clock` sets it once over ssh. It changes nothing on this
  computer; run it after the car is switched on and again before each survey run.
- `ros2 run camslab car chrony` makes the car's clock follow this computer's for good.
  Here it adds `allow <car address>` and `local stratum 10` to `/etc/chrony.conf` and a
  firewalld rule that accepts NTP from that one address; on the car it installs chrony,
  or configures `systemd-timesyncd` when chrony cannot be installed without internet.
  Both sides ask for sudo. Run it again when this computer gets another address on the
  lab network. Use it on your own computer, not on a borrowed one.

When a car's address is unknown (no display on the car),

    ros2 run camslab car find

scans the `/24` of `camera_host` (or `--subnet 192.168.1.0/24`) for hosts with ssh and
prints, for each, whether its ssh host key is the car image's (every car is flashed from
the same image, so they all share it), whether the login works, and its hostname and
Jetson serial number. The serial tells identical cars apart; unplugging one does too.

Every `car` command and the launch files read `config/survey.yaml`; a value is
overridden on the command line as `name:=bodies`.

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

The estimator pairs every image with the motion capture pose at the image time minus
`camera_delay`. An image whose time has no pose is dropped and reported every 5 s; this
happens when the car's clock is off or the motion capture stops.

The estimator skips an image taken while the car turned faster than `max_yaw_rate`. An
error `dt` between the image time and the pose time moves a marker at distance `z` by
`z * yaw rate * dt`: 10 mm at 2 m, 0.5 rad/s and 10 ms, and a turn blurs the image.
Rotation pairs need two images at different headings, and neither has to be taken in
the turn.

A pure-translation pair is two images of a marker between which the car's orientation,
as the motion capture measures it, changed by less than `straight_tol`. The simulation
uses 0.0005 rad, which is below the orientation noise of a motion capture pose and the
heading error of a real car on a straight edge, so on hardware almost no pair would
pass and the camera orientation would not be identified. The survey uses 0.01 rad
(0.6 degrees). A pair with a rotation `psi` biases the camera orientation by about
`psi |t_X| / displacement`, which is 0.002 for the camera 0.2 m from the rear axle and
a displacement of 1 m. Every 5 s the estimator logs how many images it used and skipped, and the
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

To run the chain before a calibration exists, leave `camera` empty and add
`nominal_fov` to `config/survey.yaml`, the camera's horizontal field of view in degrees
(60 for a typical webcam). The camera node then builds the camera matrix from the image
size and that angle, with no distortion, and warns once at startup. Detection and the
marker directions are fine; every marker distance is scaled by the ratio of the true focal
length to the nominal one, so the estimated camera position and marker positions carry the
same scale error. Remove `nominal_fov` once the calibration is in place so a missing
calibration is reported instead of guessed.

(settings)=
## Settings

`camslab/config/survey.yaml` holds what changes between cars and sessions:

| Key | Meaning |
|---|---|
| `name` | the car's ROS namespace and its rigid body name in the motion capture |
| `camera_host` | the car as `user@address`; the camera node and the command relay are uploaded and started there over ssh. Empty: the camera node runs on this computer |
| `camera` | the calibration: a name in `config/cameras/` or a path to a file |
| `mocap_server` | the motion capture's VRPN server. `python3 scripts/find_vicon.py` looks for it on every network this computer is on, without ROS |
| `camera_delay` | image latency [s]; the robot pose is taken this long before the image stamp. The batch check of a recorded run prints the value of this camera |
| `drive`, `driver_cmd_vel` | `teleop`: the car is driven with `car teleop`; `true`: the path controller drives the bowtie; `false`: nothing here commands the car. And the Twist topic of the car's driver |
| `controller` | controller parameters on top of `params.yaml`, as in {doc}`configuration`: `kappa_max`, `v_max`, `w_max` and `corner_radius`, the radius of the bowtie's corner arcs [m] (0: `1 / kappa_max`). The bowtie's centre, direction, size and lap time are fitted into the safe area and are not set here |
| `marker_ids`, `marker_height`, `marker_height_known` | the markers in the room, their centre heights above the motion capture's z = 0 plane and which of them the estimator may use. An error in a known height goes into the camera height and into every marker height as the mean over the stored height observations: one of three heights 10 mm too high puts all of them about 4 mm too high |

What is the same for every car and survey of the lab is in `DEFAULTS` of
`camslab/camslab/car.py`. A key of that table added to `survey.yaml`, or given as
`key:=value` on a command line, replaces the value there:

| Key | Default | Meaning |
|---|---|---|
| `tracker` | empty | rigid body name in the motion capture when it is not `name` |
| `mocap_port` | 3883 | the VRPN port |
| `container`, `ros_distro` | `yahboom_robot`, `humble` | the car's ROS container and release; an empty `container` runs on the car's host |
| `video_device`, `image_width`, `image_height`, `framerate`, `pixel_format` | `/dev/video0`, 640, 480, 30, `yuyv` | the camera, as `v4l2-ctl --list-formats-ext` reports it |
| `nominal_fov` | 0 | horizontal field of view [deg] used while `camera` is empty; 0 requires a calibration |
| `preview_rate` | 2 | JPEG frames per second streamed from the car for RViz and `car status --show`; 0 for none |
| `record_video` | false | the car keeps every camera frame on its own disk; `ros2 run camslab car video` copies them here |
| `dictionary`, `marker_size` | `DICT_4X4_50`, 0.14 | the printed markers. The camera node publishes only the ids of `marker_ids`, because floor and shelf texture decodes as a random id of a 4x4 dictionary now and then |
| `min_marker_pixels`, `ambiguity_ratio` | 30, 2 | which detections become observations: the shortest side in the image [px] and the ratio of the two pose solutions' reprojection errors; 0 switches a check off |
| `max_yaw_rate` | 0.5 | the estimator skips an image taken while the car turned faster [rad/s]; 0 for no limit |
| `straight_tol` | 0.01 | two robot poses form a pure-translation pair when their orientations differ by less [rad], see [Which images are used](#which-images-are-used) |
| `marker_position_noise`, `marker_rotation_noise`, `marker_height_noise` | 0.013, 0.7, 0.002 | the measurement noise the covariance radii are computed for: the scatter of the camera's marker positions [m] and orientations [deg] per axis and the accuracy of the measured heights [m]. The batch check of a recorded run prints the first two for this camera |
| `command_timeout` | 0.5 | the age at which the relay replaces a command by zero [s] |
| `rviz`, `webui`, `port` | true, true, 8000 | whether RViz and the web UI start, and the web UI's port |

The estimator gains and the other pair-selection thresholds stay in `config/fiducial.yaml`.

## A survey session

The steps of one session in the lab, in order. No position has to be measured: the
path is fitted into the safe area, and the stands go by eye.

Before the session, once per car and computer: `ros2 run camslab car setup` passes, and
the car passed the test with its wheels off the floor in
[Driving and safety](#driving-and-safety). Bring the stands of `marker_ids` in
`survey.yaml` with their markers glued on.

1. **Motion capture.** Define the car as a rigid body named like `name` in
   `survey.yaml`, with its origin at the rear-axle centre, x forward and z up. Switch
   the car on, start its driver, set its clock and start the survey:

       ros2 run camslab car clock
       ros2 launch camslab survey.launch.py

   In a second terminal, `ros2 run camslab car status` must show `pose` at 100 Hz and
   `observations` at 30 Hz.

2. **The safe area.** In the web UI on <http://127.0.0.1:8000>, in the **Safety
   polytope** box, check **edit**, carry the car to each corner of the free floor and
   press **Add robot position** there, then press **Apply and save** and uncheck
   **edit**. This also sets the path: the largest bowtie that fits in the area, at most
   4 m by 2 m, centred, with its long side along the longest edge of the area. An area
   of 4.5 m by 2.5 m gives the full size; under 2.7 m by 1.6 m no bowtie fits and the
   web UI says so. The area and the path are kept for the next launch.

3. **A lap without the stands.** Put the car in the middle of the area, pointing along
   the orange path where it leaves the centre. Press **Start** and stay within reach
   for the lap; **Stop** stops it. If the car runs wide in the corners, raise
   `corner_radius` in `survey.yaml` (0.5 m; at most 0.6 m for the full-size bowtie) and
   start the launch again.

4. **The stands.** A stand goes about half a metre past a corner of the bowtie, on the
   line the car drives toward that corner on, facing the car as it comes. The car turns
   before it reaches the stand. Exact places do not matter, because the survey
   estimates them. Seen with the long side across the page, the car drives
   X, B, D, A, C, X:

       A               B        A, B: the corners where a diagonal ends
       |\             /|
       | \           / |
       v  \         /  v        both short sides are driven this way
       |   \       /   |
       |    \     /    |
       |      X        |        X: the centre, where a lap starts
       |    /     \    |
       |   /       \   |
       C               D        C, D: the corners where a short side ends

   With four stands, one goes past each corner. With two, put one past A or B and the
   other past the opposite one of C and D: past A, on the line from D, facing D, and
   past D, on the line from B, facing B. The stand past C or D is seen while the car
   turns onto the short side, which the camera position needs. With both stands past A
   and B the car only ever drives straight at them, and the camera position is not
   identified. Put 200 g on the foot of an overhang stand. If a marker's centre is not
   at the height in `marker_height` to within 2 mm, correct the value and start the
   launch again.

5. **The survey.** Start the recording in a third terminal, in the repository folder
   and with the car's name in the topics:

       ros2 bag record -o survey_1 /angostura/pose /angostura/observations /angostura/estimate

   Set the clock again (`ros2 run camslab car clock`), put the car in the middle as in
   step 3 and press **Start**, which also resets the estimator. Let it drive four
   laps, press **Stop**, and end the recording with Ctrl-C. During the run the `markers` line of `car status` names each marker as the
   car drives toward it, and the dashed covariance radii appear in the web UI's camera
   panel once both adaptive laws are on.

**Driving by hand.** Instead of steps 2, 3 and the bowtie of step 5, the car can be
driven from the keyboard, with the stands anywhere:

    ros2 run camslab car teleop

A key drives while it is held, and for 1 s after: `w` forward, `q` and `e` forward to
the left and right, `s`, `z`, `c` the same backward. Space or any other key stops at
once, `+` `-` change the speed, `[` `]` the steering curvature, `r` resets the
estimator and `x` quits. At rest it sends zero 20 times a second, and the relay on the
car stops the car when the commands stop. The safe area of the web UI stays in force:
without one the car is held, and outside it, or without a motion capture pose, only a
command that drives back the way the car came is passed on. Do not drive while a
large copy from the car, such as `car video`, fills the WiFi. Set `drive: teleop` in `survey.yaml`
for this: the web UI then draws no path, takes any safe area, and its **Start** only
resets the estimator and the plots. Below the keys it shows the
status of `car status`, with the estimator's report: per marker a bar of the straight
pairs and one of the turn pairs formed, and a bar per adaptive law that is full when
the law switches on. Drive straight toward each marker for a metre or more (straight
pairs), and past each marker in a curve while it stays in view (turn pairs), until
every bar is full. Press `r` when the car is in place, to start the estimator from
zero.

`car teleop` records the run by itself, from its start to `x`, into
`surveys/<name>_<date>_<time>`: the robot pose, the observations, the estimator's
output and report, the commands, the camera's preview images, the motion capture's
`marker_<id>` bodies and the car driver's speed and inertial measurements, with a copy
of `survey.yaml` and the commit of this repository. `--no-record` switches it off.

The camera's frames do not cross the network, so the recording has only the preview at
2 Hz. To keep every frame, start the launch with `record_video:=true`: the camera node
then writes them on the car, as Motion JPEG with the stamp of each frame in a CSV, at
2 to 4 MB/s, which the car's 22 GB of free disk holds for more than an hour. A writing
problem stops the frame recording and not the camera node. Afterwards

    ros2 run camslab car video

copies the recordings to `surveys/video` here and prints the command that frees the
car's disk.

On hardware the web UI plots, besides the two error panels of the simulation: the
position error of each fiducial against the motion capture with the covariance radius,
the camera orientation and position estimates, the four covariance radii, the
excitation of the two laws against their gate, and the pairs formed per marker.
**Download CSV** saves all of them.

**Markers in the motion capture.** A stand that is also a rigid body `marker_<id>` in
the motion capture, with its origin at the marker's centre, gives the truth of that
marker's position. A body whose origin is not at the marker's centre, for instance on
the floor, gives the horizontal position, and `marker_height` is taken as the height.
The report and the plots then show the distance of the estimated position to it,
and `car teleop` records it. With `ros2 bag record` by hand, add its topic,
`/vrpn_mocap/marker_<id>/pose`.

Afterwards, on any computer, fit the recording ([Ground truth](#ground-truth)):

    cd camslab_standalone
    uv run python -m camslab_standalone.fiducial.recorded ../survey_1

What the placement needs: every marker is seen while the car drives straight toward
it, and at least one is seen from two directions 30 degrees or more apart, which the
turn onto a short side gives. A marker is used up to about 2 m away. A marker whose face
points along the motion capture's +y axis, to within 40 degrees, gets a poor
orientation estimate, because that orientation is the singularity of the estimator's
coordinates; its position and the camera's mount are not affected. The fit turns the
bowtie so that no stand placed as in step 4 faces that way when the area's long side
is along the x axis.

## On another computer

The survey needs nothing from the computer it was prepared on:

1. Clone the repository and build the Docker image ({doc}`install`), which needs the
   internet once. Without it, carry the image over: `docker save camslab-ros:humble |
   gzip > camslab-ros.tar.gz` here and `docker load < camslab-ros.tar.gz` there.
2. `export ROS_DOMAIN_ID=32` before `docker compose -f docker/compose.yaml run --rm
   ros`, and build the workspace in the container. Further terminals are further
   `docker compose run` shells.
3. `ros2 run camslab car setup` installs that computer's ssh key on the car and asks
   the car's password once; the key is kept in a Docker volume. `ros2 run camslab car
   clock` sets the car's clock from that computer, so `car chrony`, which changes the
   computer's own time service, is not needed for a session.
4. Without a display for RViz, add `rviz:=false` to the launch and use the web UI.

The safe area is saved in `camslab/config/polytope.yaml` of that clone, and recordings
made in the repository folder stay there after the container ends.

## Running

    ros2 launch camslab survey.launch.py

This uploads and starts the camera node on the car over ssh (with the intrinsics of
`camera` on its command line) and, here, `vrpn_mocap`, `pose_tf`, the estimator with
`sensor: aruco`, RViz with the preview image and the web UI on <http://127.0.0.1:8000>.
With `drive: true` it also starts the path controller and the command relay on the car,
see [Driving and safety](#driving-and-safety). The path is a bowtie fitted into the
safe area and the stands go past its corners ([A survey session](#a-survey-session)):
the camera orientation needs both pure translations and turns
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
{doc}`running` on this computer and the web UI controls it: **Start** and **Stop** and
the safe area ({doc}`safety-area`), into which the path is fitted. The controller starts
disabled and drives only after **Start**, and the web UI refuses **Start** until a safe
area is set. With `drive: false` nothing here commands the
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
detected after the car crossed the edge, so keep the edge at least 0.5 m from walls. A
stand may be inside the area as long as it is off the path; half a metre past a corner
of the bowtie is about 0.7 m from the path. The relay cannot stop the car if the relay itself or
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

The path is the largest bowtie that fits in the safe area with 0.25 m to its edge, at
most 4.0 m by 2.0 m, which is a lap of 80 s. It is driven at 0.14 m/s, and its corners
are arcs at the R2's steering limit of 2.9 1/m, driven at 0.42 rad/s. The arcs cut the
corners, so the full-size path is 4.0 m long and 1.6 m wide. A lap starts and ends at the centre of the safe area, heading along
the first diagonal, 27 degrees from the long side. The drawn path and the safe-area check
cover the whole lap.

## Rehearsal and stand layout

A survey session does not need this section. It is the simulation behind the placement
rule of [A survey session](#a-survey-session), for trying another path or layout.

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
`max_yaw_rate`, a measurement noise of 13 mm and 0.7 degrees, a full-size bowtie and a
stand 0.6 m past a corner for each marker of `marker_ids`. It prints the images used
per marker, the pairs stored, whether the two adaptive laws switch on and the errors
after each lap, and it draws the floor plan. Another layout is rehearsed with a `stands`
list in `survey.yaml`: per marker, the stand, its position `x`, `y` [m] from the
bowtie's centre along and across the long side, and its `facing` [deg] from the long
side.

```{image} images/survey-layout.png
:alt: Floor plan of the bowtie with the upright stand past the end of a diagonal and the diamond stand past the end of a short side, and the stretches of the path on which each marker is in use
```

What the rehearsal showed for the lab's camera:

- On a gerono no marker gives a pure-translation pair, because the heading never repeats
  while the marker is in view. The orientation law stays off and the camera's
  orientation is not identified. The bowtie's straight edges give the pure-translation
  pairs.
- With four stands, one past each corner, both laws are on after two laps, on the full
  bowtie and on one of 2.8 m by 1.4 m, with the stands 0.5 m or 0.75 m past the
  corners. The camera position error is 27 to 36 mm and the marker position error 27
  to 49 mm (one seed each).
- With two stands the corners matter:

  | Stands past | Laps | Both laws on | Camera position error |
  |---|---|---|---|
  | A and D | 4 | yes | 26 mm |
  | C and D | 2 | yes | 19 mm |
  | A and C | 2 | yes | 45 mm |
  | A and B | 2 | no | 233 mm |

  A stand past C or D gives the rotation pairs. Past A and B there is none, and the
  translation law stays off. C and D was run on the full bowtie only; on a small one
  the short sides have almost no straight part.
- A marker's orientation is estimated in Cayley coordinates, which are singular for a
  rotation by 180 degrees from the world frame. An upright marker is within 30 degrees
  of it when it faces between 48 and 132 degrees in motion capture coordinates, the
  overhang between 28 and 152, the diamond between 3 and 87 and the faceup between 24
  and 96. The rehearsal flags such a stand. It affects the orientation estimate of
  that marker only. For a bowtie along the motion capture's y axis one of the stands
  past A and B faces within 30 degrees of +y; turn that stand by 30 degrees or accept
  its orientation estimate.

The rehearsal does not model lens distortion, markers hiding each other, or the
ambiguity check. Its car follows the path within 3 mm, and its robot pose has the
simulation's noise of 0.2 mm and 0.01 degrees of yaw. The motion capture's orientation
noise and the car's heading error on the floor were not measured. In a run with four
stands, 0.1 degrees of noise on all three axes of the robot's orientation and a heading
that wanders by 2 degrees rms around the path, the simulation's `straight_tol` of
0.0005 rad stored no pure-translation pair in four laps and the camera orientation
stayed at its initial value. The survey's 0.01 rad stored 74 to 85 pairs over three
seeds, and 26 to 31 with 0.3 degrees of noise and 4 degrees of wander, and both laws
were on in all six runs. With corner arcs of `kappa_max: 2.0` instead of 2.9 both laws
switch on as well. The camera's height of 0.12 m is from
`hardware/marker_mounts/README.md`; its position 0.20 m ahead of the rear axle is
assumed.

## Ground truth

The paper's errors need the true camera extrinsic and marker poses, which the lab does not
know. Two ways to get them:

- Put motion capture markers on each fiducial board and define it as a rigid body with its
  origin at the marker centre. Its pose is the true `Y_j`, and the marker position error
  `||t~_j||` follows directly. This is the Vicon integration of the vision guide's future
  work.
- Fit a recorded run in a batch. The least squares over all observations is a reference
  for the online estimate even without a truth, and the online estimate must end inside
  its covariance radius of it.

Record a run as in [A survey session](#a-survey-session) and fit it, on any computer,
without ROS:

    cd camslab_standalone
    uv run python -m camslab_standalone.fiducial.recorded <bag directory>

It takes the robot pose at each image time minus `camera_delay` and skips the images
above `max_yaw_rate`, as the estimator node does, with the marker ids and heights of
`survey.yaml`. It prints:

- the images used, skipped and without a marker, and the images per marker,
- the residuals of the fit, which are the scatter of this camera's marker positions and
  orientations and replace the 13 mm and 0.7 degrees assumed from rendered images,
- the batch estimate and its covariance radii, the online radii and the distance of the
  online estimate to the batch one at the end of the run,
- the position residual for camera delays from 0.10 s below to 0.10 s above
  `camera_delay`. The delay with the lowest residual is the delay of this camera; set
  `camera_delay` to it. The residual depends on the delay only where the car moves.

The figure is the distance of the online estimate to the batch optimum over the run,
with the online radius dashed. A marker of `marker_ids` that no used image shows is left
out of the fit and named.

## Frames

OpenCV's optical frame has z forward, x right and y down. The estimator's camera frame has
x forward and z up like the robot frame, so the camera node rotates every observation by
`OPTICAL_TO_BODY` before it is published (the same constant as `camslab.aruco`, which the
tests keep equal); the extrinsic `X` is estimated for that frame. The marker frame is
OpenCV's: x and y in the marker plane, z out of it.
