"""Leader-follower Yahboom R2s in Gazebo Fortress + web UI.

    ros2 launch gtg_ros gazebo_multi.launch.py                       # config/fleet.yaml
    ros2 launch gtg_ros gazebo_multi.launch.py fleet:=/path/to/other.yaml

The fleet file lists the robots. The one without `follows` is the leader and drives to
/goal, set in the web UI. Every other robot drives to the follow_goal of the robot named
in its `follows`.

This file starts the Gazebo world, the web UI, and RViz, and includes robot.launch.py
once per robot with name:=<name> and goal_topic:=
  leader:    /goal
  followers: /<follows>/follow_goal

RViz is the main view: its 2D Goal Pose tool (G) publishes /goal. The controllers start
enabled, so the fleet drives as soon as a goal arrives. /webui_bridge_node (root
namespace, webui:=false to skip) draws every robot and plots; its Stop and Start disable
and enable every controller.
Spawn poses come from the fleet file; the web UI's Reset stops the controllers and clears
the plots.
"""

import os
import re
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Names a fleet file's `color` can use in place of [r, g, b].
COLORS = {
    "green": [0, 0.63, 0.24],
    "red": [0.85, 0.15, 0.15],
    "blue": [0.12, 0.4, 0.85],
    "orange": [0.95, 0.5, 0.1],
    "yellow": [0.95, 0.8, 0.1],
    "purple": [0.55, 0.25, 0.75],
    "cyan": [0.1, 0.7, 0.8],
    "pink": [0.95, 0.4, 0.65],
    "white": [0.9, 0.9, 0.9],
    "grey": [0.5, 0.5, 0.5],
    "black": [0.1, 0.1, 0.1],
}
DEFAULT_COLOR = "green"


def load_fleet(path):
    """Read and check the `robots` list of a fleet file."""
    with open(path) as f:
        fleet = yaml.safe_load(f)["robots"]
    names = [r["name"] for r in fleet]
    for name in names:
        # Valid as a ROS namespace, a Gazebo model name, and a TF prefix.
        if not re.fullmatch(r"[a-z][a-z0-9_]*", str(name)):
            raise RuntimeError(f"{path}: robot name {name!r} must match [a-z][a-z0-9_]*")
    if len(set(names)) != len(names):
        raise RuntimeError(f"{path}: duplicate robot names {names}")
    leaders = [r["name"] for r in fleet if "follows" not in r]
    if len(leaders) != 1:
        raise RuntimeError(f"{path}: need exactly one robot without `follows`, got {leaders}")
    for r in fleet:
        if "follows" in r and r["follows"] not in names:
            raise RuntimeError(f"{path}: {r['name']} follows unknown robot {r['follows']}")
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
    """Path of an RViz config: rviz/fleet.rviz plus one RobotModel display per robot."""
    with open(os.path.join(share, "rviz", "fleet.rviz")) as f:
        config = yaml.safe_load(f)
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
    out = tempfile.NamedTemporaryFile("w", prefix="gtg_fleet_", suffix=".rviz", delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def robots(context):
    share = get_package_share_directory("gtg_ros")
    world = os.path.join(share, "worlds", "gtg.sdf")
    gui_config = os.path.join(share, "worlds", "gui.config")

    fleet = load_fleet(LaunchConfiguration("fleet").perform(context))
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    port = int(LaunchConfiguration("port").perform(context))

    gz_args = f"-r -s {world}" if not gui else f"-r --gui-config {gui_config} {world}"
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items())]

    robot_launch = os.path.join(share, "launch", "robot.launch.py")
    for robot in fleet:
        goal = f"/{robot['follows']}/follow_goal" if "follows" in robot else "/goal"
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={"name": robot["name"], "x": str(robot["x"]),
                              "y": str(robot["y"]), "yaw": str(robot.get("yaw", 0.0)),
                              "goal_topic": goal, "color": rgba(robot)}.items()))

    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config(share, fleet)]))

    leader = next(r["name"] for r in fleet if "follows" not in r)
    followers = [r for r in fleet if "follows" in r]
    if LaunchConfiguration("webui").perform(context).lower() != "true":
        return actions
    bridge_params = {"backend_label": f"Gazebo fleet of {len(fleet)}, leader {leader}",
                     "port": port, "start_enabled": True,
                     "controller_node": f"/{leader}/controller_node", "robot_name": leader}
    if followers:
        bridge_params["followers"] = [r["name"] for r in followers]
        bridge_params["follows"] = [r["follows"] for r in followers]
    # Root namespace: publishes /goal, and watches the leader through these remappings.
    actions.append(Node(package="gtg_webui", executable="webui_bridge_node", output="screen",
                        parameters=[bridge_params],
                        remappings=[("pose", f"/{leader}/pose"),
                                    ("cmd_vel", f"/{leader}/cmd_vel")]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("gtg_ros")
    return LaunchDescription([
        DeclareLaunchArgument("fleet", default_value=os.path.join(share, "config", "fleet.yaml"),
                              description="YAML file with the robot list"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="true also opens the Gazebo window"),
        DeclareLaunchArgument("port", default_value="8000", description="web UI port"),
        DeclareLaunchArgument("rviz", default_value="true", description="start RViz"),
        DeclareLaunchArgument("webui", default_value="true",
                              description="start the web UI (optional; RViz sets goals too)"),
        SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.path.join(share, "models")),
        OpaqueFunction(function=robots),
    ])
