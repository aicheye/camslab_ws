"""Fiducial surveying on a real car: the camera node (camera and ArUco detection), the
motion capture pose, the estimator, RViz and the web UI; settings in config/survey.yaml.
With `camera_host` set, the camera node is uploaded to the car and run there over ssh
(camslab.car) and everything else runs here.

With `drive` set, the path controller runs here too, disabled until Start is pressed in
the web UI, with the safe area of the `polytope` file, and its commands reach the car's
driver through the command relay on the car (camslab.car_drive), which stops the car
when the commands stop arriving. Without `drive` the car is driven by its own teleop."""

import os
import re
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from camslab import car
from camslab_webui import polytope as safe_area

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import camslab_settings


def rviz_config(share, name, drive=False, teleop=False):
    """rviz/fleet.rviz plus the mocap pose, the fiducial markers and the TF tree, and
    with `drive` the controller's safe area, path and reference."""
    with open(os.path.join(share, "rviz", "fleet.rviz")) as f:
        config = yaml.safe_load(f)
    volatile = {"Depth": 5, "Durability Policy": "Volatile", "History Policy": "Keep Last"}
    config["Visualization Manager"]["Displays"] += [
        {"Class": "rviz_default_plugins/Pose", "Name": f"{name} pose", "Enabled": True,
         "Topic": {"Value": f"/{name}/pose", **volatile, "Reliability Policy": "Best Effort"},
         "Color": "0; 160; 60", "Shape": "Arrow", "Shaft Length": 0.25, "Shaft Radius": 0.02,
         "Head Length": 0.08, "Head Radius": 0.05, "Value": True},
        {"Class": "rviz_default_plugins/MarkerArray", "Name": "fiducials", "Enabled": True,
         "Topic": {"Value": f"/{name}/fiducials", **volatile, "Reliability Policy": "Reliable"},
         "Namespaces": {}, "Value": True},
        {"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": True,
         "Show Names": True, "Show Axes": True, "Show Arrows": False, "Marker Scale": 0.5,
         "Value": True},
        {"Class": "rviz_default_plugins/MarkerArray", "Name": "observations", "Enabled": True,
         "Namespaces": {}, "Value": True,
         "Topic": {"Value": f"/{name}/observation_markers", **volatile,
                   "Reliability Policy": "Reliable"}},
        {"Class": "rviz_default_plugins/Image", "Name": "preview", "Enabled": True,
         "Normalize Range": True, "Value": True,
         "Topic": {"Value": f"/{name}/preview/compressed", **volatile,
                   "Reliability Policy": "Best Effort"}},
    ]
    if drive:
        latched = {"Depth": 1, "Durability Policy": "Transient Local",
                   "History Policy": "Keep Last", "Reliability Policy": "Reliable"}
        displays = [
            {"Class": "rviz_default_plugins/Polygon", "Name": "safety_polytope",
             "Enabled": True, "Topic": {"Value": f"/{name}/safety_polytope", **latched},
             "Color": "220; 40; 40", "Alpha": 1, "Value": True},
            {"Class": "rviz_default_plugins/Path", "Name": f"{name} ref_path", "Enabled": True,
             "Topic": {"Value": f"/{name}/ref_path", **latched},
             "Color": "235; 104; 52", "Line Style": "Lines", "Value": True},
            {"Class": "rviz_default_plugins/Pose", "Name": f"{name} ref_pose", "Enabled": True,
             "Topic": {"Value": f"/{name}/ref_pose", **volatile, "Reliability Policy": "Reliable"},
             "Color": "235; 104; 52", "Shape": "Arrow", "Shaft Length": 0.25,
             "Shaft Radius": 0.02, "Head Length": 0.08, "Head Radius": 0.05, "Value": True},
        ]
        # Driven by hand, there is no path to show: only the safe area.
        config["Visualization Manager"]["Displays"] += displays[:1] if teleop else displays
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_survey_", suffix=".rviz",
                                      delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def survey(context):
    share = get_package_share_directory("camslab")
    settings = camslab_settings.load(context, car.DEFAULTS)
    name = str(settings["name"])
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise RuntimeError(f"robot name {name!r} must match [a-z][a-z0-9_]*")
    tracker = str(settings["tracker"] or name)
    tracker_ns = re.sub(r"_+", "_", re.sub(r"[^a-zA-Z0-9_]", "_", tracker))
    calibration = car.camera_file(settings)
    if settings.get("camera_host"):
        # The camera node on the car, in the foreground of an ssh session that ends with
        # the launch; car_node.py is uploaded first.
        car.upload_node(settings)
        camera = ExecuteProcess(cmd=car.camera_ssh_command(settings, calibration, batch=True),
                                name="camera", output="screen")
    else:
        camera = Node(package="camslab", executable="car_node", name="camera", output="screen",
                      arguments=car.node_arguments(settings, calibration))

    actions = [
        Node(package="vrpn_mocap", executable="client_node", name="vrpn_mocap_client_node",
             namespace="vrpn_mocap", output="screen",
             parameters=[{"server": str(settings["mocap_server"]),
                          "port": int(settings["mocap_port"]),
                          "frame_id": "map", "update_freq": 100.0, "refresh_freq": 1.0,
                          "multi_sensor": False, "use_vrpn_timestamps": False}],
             remappings=[(f"/vrpn_mocap/{tracker_ns}/pose", f"/{name}/pose")]),
        Node(package="camslab", executable="pose_tf", namespace=name,
             parameters=[{"child_frame": f"{name}/mocap"}]),
        camera,
        Node(package="camslab", executable="observation_markers", namespace=name, output="log",
             parameters=[{"marker_size": float(settings["marker_size"])}]),
        Node(package="camslab", executable="estimator", namespace=name, output="screen",
             parameters=[os.path.join(share, "config", "fiducial.yaml"),
                         {"estimator": "fiducial", "sensor": "aruco",
                          "marker_ids": [int(i) for i in settings["marker_ids"]],
                          "marker_height": [float(h) for h in settings["marker_height"]],
                          "marker_height_known": [bool(k) for k in settings["marker_height_known"]],
                          "camera_delay": float(settings["camera_delay"]),
                          "max_yaw_rate": float(settings["max_yaw_rate"]),
                          "straight_tol": float(settings["straight_tol"]),
                          "marker_position_noise": float(settings["marker_position_noise"]),
                          "marker_rotation_noise": float(settings["marker_rotation_noise"]),
                          "marker_height_noise": float(settings["marker_height_noise"])}]),
    ]
    drive = bool(settings.get("drive"))
    teleop = str(settings.get("drive")).lower() == "teleop"
    bridge = {"backend_label": f"Fiducial survey, {name}", "port": int(settings["port"]),
              "robot_name": name, "adaptive": "fiducial"}
    if drive:
        # The controller never starts by itself on the car: Start in the web UI enables it.
        polytope = LaunchConfiguration("polytope").perform(context)
        # The path is the largest bowtie that fits in the saved safe area; the web UI
        # fits it again whenever the area is set.
        controller = dict(settings.get("controller") or {})
        corners, margin = safe_area.read_file(polytope)
        if corners and teleop:
            controller.update(safe_area.hold_point(corners))
        elif corners:
            try:
                controller.update(safe_area.fit_bowtie(
                    corners, margin, float(controller.get("kappa_max", 2.9)),
                    float(controller.get("corner_radius", 0.0))))
            except ValueError as exc:
                print(f"survey: {exc}; set the safe area again in the web UI")
        elif not teleop:
            print("survey: no safe area yet; set it in the web UI, the bowtie is fitted into it")
        settings = {**settings, "controller": controller}
        files = [LaunchConfiguration("params").perform(context),
                 camslab_settings.controller_file(settings)]
        if polytope and os.path.isfile(polytope):
            files.append(polytope)
        actions.append(Node(package="camslab", executable="controller", namespace=name,
                            output="screen", parameters=files + [{"start_enabled": False}]))
        if settings.get("camera_host"):
            car.upload_node(settings, car.DRIVE_FILE, car.REMOTE_DRIVE)
            actions.append(ExecuteProcess(cmd=car.drive_ssh_command(settings, batch=True),
                                          name="drive", output="screen"))
        else:
            actions.append(Node(package="camslab", executable="car_drive", name="drive",
                                output="screen", arguments=car.drive_arguments(settings)))
        bridge.update({"start_enabled": False, "polytope_file": polytope, "fit_bowtie": True,
                       "teleop": teleop})
    if settings["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config(share, name, drive, teleop)]))
    if settings["webui"]:
        actions.append(Node(package="camslab_webui", executable="webui_bridge",
                            output="screen", parameters=[bridge]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("camslab")
    return LaunchDescription([
        *camslab_settings.arguments(os.path.join(share, "config", "survey.yaml"),
                                    car.DEFAULTS),
        DeclareLaunchArgument("params", default_value=os.path.join(share, "config", "params.yaml"),
                              description="controller parameter file"),
        DeclareLaunchArgument("polytope",
                              default_value=camslab_settings.default_polytope_file(__file__),
                              description="safety polytope file: loaded when it exists, and "
                              "where the web UI saves it; empty for none"),
        OpaqueFunction(function=survey),
    ])
