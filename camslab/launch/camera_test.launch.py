"""A check of the camera streaming before a survey: the camera node on the car (or here,
with `camera_host` empty), the camera frame pinned at the origin of RViz, and RViz with
the car's preview image and every marker it sees drawn as a square at its observed pose
with its id and distance. Hold a marker in front of the camera: in RViz it sits where it
is in the room relative to the camera (x forward, y left, z up), at the right distance and
tilt. No motion capture and no estimator. Settings in config/survey.yaml, overridable on
the command line: `ros2 launch camslab camera_test.launch.py name:=yahboom1`."""

import os
import re
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

from camslab import car

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import camslab_settings


def rviz_config(name):
    """A grid, the TF tree, the observation markers and the preview, seen from behind the
    camera; the fixed frame is the camera."""
    camera_frame = f"{name}/camera"
    volatile = {"Depth": 5, "Durability Policy": "Volatile", "History Policy": "Keep Last"}
    config = {
        "Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"},
                   {"Class": "rviz_common/Views", "Name": "Views"}],
        "Visualization Manager": {
            "Class": "",
            "Displays": [
                {"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True,
                 "Cell Size": 0.5, "Plane Cell Count": 12, "Offset": {"X": 2.0, "Y": 0.0, "Z": 0.0},
                 "Reference Frame": "<Fixed Frame>", "Value": True},
                {"Class": "rviz_default_plugins/TF", "Name": "TF", "Enabled": True,
                 "Show Names": True, "Show Axes": True, "Show Arrows": False, "Marker Scale": 0.5,
                 "Value": True},
                {"Class": "rviz_default_plugins/MarkerArray", "Name": "observations",
                 "Enabled": True, "Namespaces": {}, "Value": True,
                 "Topic": {"Value": f"/{name}/observation_markers", **volatile,
                           "Reliability Policy": "Reliable"}},
                {"Class": "rviz_default_plugins/Image", "Name": "preview", "Enabled": True,
                 "Normalize Range": True, "Value": True,
                 "Topic": {"Value": f"/{name}/preview/compressed", **volatile,
                           "Reliability Policy": "Best Effort"}},
            ],
            "Global Options": {"Fixed Frame": camera_frame, "Background Color": "48; 48; 48",
                               "Frame Rate": 30},
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"},
                      {"Class": "rviz_default_plugins/Measure"}],
            "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Name": "Current View",
                                  "Distance": 4.0, "Focal Point": {"X": 1.5, "Y": 0.0, "Z": 0.0},
                                  "Pitch": 0.35, "Yaw": 3.5},
                      "Saved": None},
            "Value": True,
        },
    }
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_camera_test_", suffix=".rviz",
                                      delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def camera_action(settings, calibration_file):
    """The camera node: over ssh on the car when `camera_host` is set (car_node.py is
    uploaded first), else here."""
    if settings.get("camera_host"):
        car.upload_node(settings)
        return ExecuteProcess(cmd=car.camera_ssh_command(settings, calibration_file, batch=True),
                              name="camera", output="screen")
    return Node(package="camslab", executable="car_node", name="camera", output="screen",
                arguments=car.node_arguments(settings, calibration_file))


def camera_test(context):
    settings = camslab_settings.load(context)
    name = str(settings["name"])
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise RuntimeError(f"robot name {name!r} must match [a-z][a-z0-9_]*")
    calibration_file = car.camera_file(settings)
    return [
        camera_action(settings, calibration_file),
        Node(package="camslab", executable="observation_markers", namespace=name, output="screen",
             parameters=[{"marker_size": float(settings["marker_size"])}]),
        Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config(name)]),
    ]


def generate_launch_description():
    share = get_package_share_directory("camslab")
    return LaunchDescription([
        *camslab_settings.arguments(os.path.join(share, "config", "survey.yaml")),
        OpaqueFunction(function=camera_test),
    ])
