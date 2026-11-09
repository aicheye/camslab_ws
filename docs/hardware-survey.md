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
marker per visible fiducial with its id and pose in the estimator's camera frame, stamped
at capture, empty when nothing is seen),
`camera_info` and `preview/compressed`, a JPEG of the frame at `preview_rate` Hz with the
detections drawn. The motion capture client, the estimator, RViz and the web UI run on
your computer, which must be on the car's network and in the same `ROS_DOMAIN_ID` as the
car (the vision guide uses 32):

    export ROS_DOMAIN_ID=32

On the car, the node needs `python3` with `rclpy`, `numpy`, OpenCV with the aruco
module and the `aruco_msgs` package in its ROS environment. On the Yahboom R2 that is the
Jetson's `yahboom_robot` Docker container (user `jetson`); Ubuntu's OpenCV package has
aruco, so once, inside it:

    apt update && apt install python3-opencv python3-numpy ros-humble-aruco-msgs

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
   the marker centre to within a few centimetres. A distance off by a constant factor
   means a wrong `marker_size` or `camera` calibration.
2. The square tilts with the marker.
3. The preview runs at `preview_rate` and the observation rate in the terminal is the
   camera's frame rate. The car node logs its detection time per frame; it must stay
   below the frame period.

No motion capture is needed for this check.

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
| `preview_rate` | JPEG frames per second streamed from the car for RViz and `car status --show`; 0 for none |
| `dictionary`, `marker_size`, `marker_ids` | the printed markers; the camera node publishes only these ids, because floor and shelf texture decodes as a random id of a 4x4 dictionary now and then |
| `marker_height`, `marker_height_known` | the centre heights above the floor and which of them the estimator may use |

The estimator gains and pair-selection thresholds stay in `config/fiducial.yaml`.

## Running

    ros2 launch camslab survey.launch.py

This uploads and starts the camera node on the car over ssh (with the intrinsics of
`camera` on its command line) and, here, `vrpn_mocap`, `pose_tf`, the estimator with
`sensor: aruco`, RViz with the preview image and the web UI on <http://127.0.0.1:8000>. Drive the car with the teleop of its driver or with
{doc}`running`'s controller on a path with straight stretches (gerono, or a bowtie at
least 2 m wide): the camera orientation needs both pure translations and turns, see
`docs/fiducial-hints.md`.

Watch the chain in a second terminal:

    ros2 run camslab car status --show

It redraws the rates twice a second and, with `--show`, opens the car's preview with the
markers it found and their distances. Check, from the top line down:

1. `preview` runs at `preview_rate`, `observations` at the camera rate, and
   `camera_info` says `fx ...`, not `uncalibrated`.
2. `seen` lists one marker per visible fiducial with its distance; in RViz they are
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
