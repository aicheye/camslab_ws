"""Yahboom R2s tracking a reference trajectory in Gazebo Fortress, with RViz and the web UI."""

import os
import re
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import camslab_settings

COLORS = {
    "green": [0, 0.63, 0.24],
    "red": [0.85, 0.15, 0.15],
    "blue": [0.12, 0.4, 0.85],
    "orange": [0.95, 0.5, 0.1],
    "purple": [0.55, 0.25, 0.75],
}
DEFAULT_COLOR = "green"

def ign_partition():
    return SetEnvironmentVariable("IGN_PARTITION", EnvironmentVariable(
        "IGN_PARTITION",
        default_value=["camslab_", EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")]))


def check_fleet(fleet, path):
    """Check the `robots` list of a settings file."""
    names = [r["name"] for r in fleet]
    for name in names:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", str(name)):
            raise RuntimeError(f"{path}: robot name {name!r} must match [a-z][a-z0-9_]*")
    if len(set(names)) != len(names):
        raise RuntimeError(f"{path}: duplicate robot names {names}")
    if not names:
        raise RuntimeError(f"{path}: `robots` is empty")
    for r in fleet:
        color = r.get("color", DEFAULT_COLOR)
        if isinstance(color, str):
            if color not in COLORS:
                raise RuntimeError(f"{path}: {r['name']} color {color!r} is not one of "
                                   f"{', '.join(COLORS)}")
        elif (not isinstance(color, list) or len(color) not in (3, 4)
                or not all(isinstance(c, (int, float)) and 0 <= c <= 1 for c in color)):
            raise RuntimeError(f"{path}: {r['name']} color must be [r, g, b] or [r, g, b, a] "
                               f"in [0, 1], got {color!r}")
    return fleet


def rgba(robot):
    """The robot's `color` as the "r g b a" string r2.urdf.xacro takes."""
    color = robot.get("color", DEFAULT_COLOR)
    color = COLORS.get(color, color) if isinstance(color, str) else color
    return " ".join(str(c) for c in (color + [1] if len(color) == 3 else color))


def rviz_config(share, fleet):
    """Path of an RViz config: rviz/fleet.rviz plus the first robot's safety_polytope and,
    per robot, a RobotModel display and its ref_path and ref_pose. Every controller has the
    same polytope, so one display draws it."""
    with open(os.path.join(share, "rviz", "fleet.rviz")) as f:
        config = yaml.safe_load(f)
    config["Visualization Manager"]["Displays"].append({
        "Class": "rviz_default_plugins/Polygon",
        "Name": "safety_polytope",
        "Enabled": True,
        "Topic": {"Value": f"/{fleet[0]['name']}/safety_polytope", "Depth": 1,
                  "Durability Policy": "Transient Local", "History Policy": "Keep Last",
                  "Reliability Policy": "Reliable"},
        "Color": "220; 40; 40",
        "Alpha": 1,
        "Value": True,
    })
    for robot in fleet:
        ns = robot["name"]
        config["Visualization Manager"]["Displays"].append({
            "Class": "rviz_default_plugins/RobotModel",
            "Name": ns,
            "Enabled": True,
            "Description Source": "Topic",
            "Description Topic": {"Value": f"/{ns}/robot_description", "Depth": 5,
                                  "Durability Policy": "Transient Local",
                                  "History Policy": "Keep Last",
                                  "Reliability Policy": "Reliable"},
            "TF Prefix": ns,
            "Visual Enabled": True,
            "Value": True,
        })
        config["Visualization Manager"]["Displays"].append({
            "Class": "rviz_default_plugins/Path",
            "Name": f"{ns} ref_path",
            "Enabled": True,
            "Topic": {"Value": f"/{ns}/ref_path", "Depth": 1,
                      "Durability Policy": "Transient Local", "History Policy": "Keep Last",
                      "Reliability Policy": "Reliable"},
            "Color": "235; 104; 52",
            "Line Style": "Lines",
            "Value": True,
        })
        config["Visualization Manager"]["Displays"].append({
            "Class": "rviz_default_plugins/Pose",
            "Name": f"{ns} ref_pose",
            "Enabled": True,
            "Topic": {"Value": f"/{ns}/ref_pose", "Depth": 5,
                      "Durability Policy": "Volatile", "History Policy": "Keep Last",
                      "Reliability Policy": "Reliable"},
            "Shape": "Arrow",
            "Color": "235; 104; 52",
            "Shaft Length": 0.25,
            "Shaft Radius": 0.02,
            "Head Length": 0.08,
            "Head Radius": 0.05,
            "Value": True,
        })
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_fleet_", suffix=".rviz", delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def robots(context):
    share = get_package_share_directory("camslab")
    world = os.path.join(share, "worlds", "camslab.sdf")
    gui_config = os.path.join(share, "worlds", "gui.config")

    settings = camslab_settings.load(context)
    fleet = check_fleet(settings["robots"], LaunchConfiguration("config").perform(context))
    gui = bool(settings["gui"])
    port = int(settings["port"])
    start_enabled = bool(settings["start_enabled"])
    polytope = LaunchConfiguration("polytope").perform(context)
    controller = camslab_settings.controller_file(settings)
    adaptive = bool(settings.get("adaptive"))

    gz_args = f"-r -s {world}" if not gui else f"-r --gui-config {gui_config} {world}"
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items())]

    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge",
                        name="clock_bridge", output="log",
                        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]))

    robot_launch = os.path.join(share, "launch", "robot.launch.py")
    for robot in fleet:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={"name": robot["name"], "x": str(robot["x"]),
                              "y": str(robot["y"]), "yaw": str(robot.get("yaw", 0.0)),
                              "color": rgba(robot),
                              "controller": controller,
                              "polytope": polytope,
                              "start_enabled": str(start_enabled).lower(),
                              "adaptive": str(adaptive).lower(),
                              }.items()))

    if settings["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config(share, fleet)]))

    if not settings["webui"]:
        return actions
    first, others = fleet[0]["name"], [r["name"] for r in fleet[1:]]
    label = f"Gazebo, {first}" if not others else f"Gazebo fleet of {len(fleet)}"
    bridge_params = {"backend_label": label, "port": port, "start_enabled": start_enabled,
                     "robot_name": first, "polytope_file": polytope,
                     "adaptive": adaptive}
    if others:
        bridge_params["others"] = others
    actions.append(Node(package="camslab_webui", executable="webui_bridge", output="screen",
                        parameters=[bridge_params]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("camslab")
    return LaunchDescription([
        *camslab_settings.arguments(os.path.join(share, "config", "sim.yaml")),
        DeclareLaunchArgument("polytope",
                              default_value=camslab_settings.default_polytope_file(__file__),
                              description="safety polytope file: loaded when it exists, and "
                              "where the web UI saves it; empty for none"),
        ign_partition(),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", [
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""), ":",
            os.path.join(share, "models")]),
        SetParameter("use_sim_time", True),
        OpaqueFunction(function=robots),
    ])
