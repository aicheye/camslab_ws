# Fiducial surveying on a real car

This page adapts the fiducial surveying estimator of {doc}`fiducial-hints` to a real car:
a USB camera on the car, ArUco markers in the lab, and the motion capture for the robot
pose. It follows the Yahboom ROSMASTER R2 vision guide (Daniel Xie, June 2026) for the
robot side and replaces its ArUco detector and follower with the surveying pipeline:

    USB camera -> usb_cam -> /<name>/image_raw -> aruco -> /<name>/observations
    motion capture -> vrpn_mocap -> /<name>/pose
    observations + pose -> estimator -> /<name>/estimate -> web UI, RViz

The estimator identifies the camera extrinsic `X = R_T_C` and the pose of every marker
while the car drives. Nothing has to be known about the camera mounting or the marker
placement except the height of at least one marker above the floor.

## Hardware

- A car tracked by the motion capture. The rigid body must be defined with x forward at
  the rear-axle centre and z up, because that is the robot frame of the paper. The
  RoboMaster S1 of {doc}`running` or a Yahboom ROSMASTER R2 both work; the estimator only
  needs the pose.
- A USB camera fixed to the car, anywhere, looking in any direction. Its intrinsics must
  be calibrated once (below). A rolling shutter is acceptable at the lab's speeds.
- At least three printed ArUco markers of one dictionary (the default is `DICT_4X4_50`,
  ids 0, 1, 2), side 0.14 m, stood up on tripods or taped to walls at a known height. Measure
  the height of each marker's centre above the motion capture floor; the estimator needs
  at least one of them (Assumption 3 of the paper). Keep the markers well separated in
  angle so that each is seen from more than one heading.

## Software on the car

The ROS 2 side runs where the camera is plugged in. On the Yahboom R2 that is the Jetson's
Docker container, reached as in the vision guide:

    ssh jetson@192.168.1.119
    docker exec -it yahboom_robot bash
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=32

Install the camera driver and the OpenCV bindings inside the container:

    apt update
    apt install ros-humble-usb-cam ros-humble-cv-bridge ros-humble-vrpn-mocap \
        ros-humble-camera-calibration python3-opencv

Clone this repository into a workspace and build the two Python packages:

    cd ~/ws/src && git clone <camslab repository>
    cd ~/ws && colcon build --symlink-install --packages-select camslab camslab_webui
    source install/setup.bash

The motion capture client, RViz and the web UI can also run on a lab PC in the same
`ROS_DOMAIN_ID`; only `usb_cam` and `aruco` have to be on the car.

## Camera calibration

ArUco pose estimation needs the camera matrix and the distortion coefficients. Calibrate
once with the standard ROS tool and a printed checkerboard (here 8 x 6 inner corners of
0.025 m):

    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
        --ros-args -r image:=/image_raw -r camera:=/camera

Move the board through the image until the bars fill, press *Calibrate*, then *Save*. The
tool writes a tarball with `ost.yaml`; copy it to `~/.ros/camera_info/camera.yaml` and
set `camera_info_url` in `config/survey.yaml` to that file. Without it the marker poses are
wrong by the unknown focal length and the estimator cannot converge.

### Skipping the calibration

To run the chain before a calibration exists, leave `camera_info_url` empty and set
`nominal_fov` in `config/survey.yaml` to the camera's horizontal field of view in degrees
(60 for a typical webcam). The `aruco` node then builds the camera matrix from the image
size and that angle, with no distortion, and warns once at startup. Detection and the
marker directions are fine; every marker distance is scaled by the ratio of the true focal
length to the nominal one, so the estimated camera position and marker positions carry the
same scale error. Set `nominal_fov: 0` once the calibration file is in place so a missing
file is reported instead of guessed.

## Settings

`camslab/config/survey.yaml` holds the lab values. Set:

| Key | Meaning |
|---|---|
| `name`, `tracker`, `mocap_server` | the car's namespace, its rigid body name and the motion capture server, as in {doc}`configuration` |
| `video_device`, `image_width`, `image_height`, `framerate`, `pixel_format` | the camera, as `v4l2-ctl --list-formats-ext` reports it |
| `camera_info_url` | the calibration file |
| `nominal_fov` | horizontal field of view [deg] used while there is no calibration file; 0 requires the file |
| `camera_delay` | image latency [s]; the robot pose is taken this long before the image stamp |
| `dictionary`, `marker_size`, `marker_ids` | the printed markers |
| `marker_height`, `marker_height_known` | the centre heights above the floor and which of them the estimator may use |

The estimator gains and pair-selection thresholds stay in `config/fiducial.yaml`.

## Running

    ros2 launch camslab survey.launch.py

This starts `usb_cam`, `aruco`, `vrpn_mocap`, `pose_tf`, the estimator with
`sensor: aruco`, RViz and the web UI on <http://127.0.0.1:8000>. Drive the car with the
teleop of its driver or with {doc}`running`'s controller on a path with straight stretches
(gerono, or a bowtie at least 2 m wide): the camera orientation needs both pure
translations and turns, see {doc}`fiducial-hints`.

Check in order:

1. `ros2 topic hz /<name>/image_raw` runs at the camera rate, and
   `ros2 topic echo /<name>/camera_info --once` shows a non-zero `k` matrix.
2. `ros2 topic echo /<name>/observations` lists one transform per visible marker, named
   `marker_<id>`. RViz draws them as TF frames hanging off `<name>/camera`; a marker
   straight ahead of the camera has a positive x.
3. `/<name>/pose` arrives at 100 Hz and the TF frame `<name>/mocap` moves with the car.
4. The web UI's camera panel shows the two camera errors as NaN (the truth is unknown)
   and the dashed covariance radii appear once the laws switch on. The RViz text above the
   car shows the same radii, prefixed `r`.

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
x forward and z up like the robot frame, so `camslab.aruco.marker_pose` rotates every
observation by `OPTICAL_TO_BODY` before it is published; the extrinsic `X` is estimated for
that frame. The marker frame is OpenCV's: x and y in the marker plane, z out of it.
