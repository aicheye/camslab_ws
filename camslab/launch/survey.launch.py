"""Fiducial surveying on a real car: USB camera, ArUco detection, motion capture pose, the
estimator, RViz and the web UI. Drive the car by any means (teleop, or live.launch.py's
controller); settings in config/survey.yaml."""

import os
import re
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import camslab_settings


def rviz_config(share, name):
    """rviz/fleet.rviz plus the mocap pose, the fiducial markers and the TF tree."""
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
    ]
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_survey_", suffix=".rviz",
                                      delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def survey(context):
    share = get_package_share_directory("camslab")
    settings = camslab_settings.load(context)
    name = str(settings["name"])
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise RuntimeError(f"robot name {name!r} must match [a-z][a-z0-9_]*")
    tracker = str(settings["tracker"] or name)
    tracker_ns = re.sub(r"_+", "_", re.sub(r"[^a-zA-Z0-9_]", "_", tracker))
    camera_frame = f"{name}/camera"

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
        Node(package="usb_cam", executable="usb_cam_node_exe", name="camera", namespace=name,
             output="screen",
             parameters=[{"video_device": str(settings["video_device"]),
                          "image_width": int(settings["image_width"]),
                          "image_height": int(settings["image_height"]),
                          "framerate": float(settings["framerate"]),
                          "pixel_format": str(settings["pixel_format"]),
                          "camera_name": "camera", "frame_id": camera_frame,
                          "camera_info_url": str(settings["camera_info_url"] or "")}]),
        Node(package="camslab", executable="aruco", namespace=name, output="screen",
             parameters=[{"marker_size": float(settings["marker_size"]),
                          "dictionary": str(settings["dictionary"]),
                          "camera_frame": camera_frame}]),
        Node(package="camslab", executable="estimator", namespace=name, output="screen",
             parameters=[os.path.join(share, "config", "fiducial.yaml"),
                         {"estimator": "fiducial", "sensor": "aruco",
                          "marker_ids": [int(i) for i in settings["marker_ids"]],
                          "marker_height": [float(h) for h in settings["marker_height"]],
                          "marker_height_known": [bool(k) for k in settings["marker_height_known"]],
                          "camera_delay": float(settings["camera_delay"])}]),
    ]
    if settings["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config(share, name)]))
    if settings["webui"]:
        actions.append(Node(
            package="camslab_webui", executable="webui_bridge", output="screen",
            parameters=[{"backend_label": f"Fiducial survey, {name}",
                         "port": int(settings["port"]), "robot_name": name,
                         "adaptive": "fiducial"}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("camslab")
    return LaunchDescription([
        *camslab_settings.arguments(os.path.join(share, "config", "survey.yaml")),
        OpaqueFunction(function=survey),
    ])
