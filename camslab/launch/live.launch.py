"""A real DJI RoboMaster S1 tracking the reference trajectory, with motion capture pose,
RViz and the web UI.

    ros2 launch camslab live.launch.py                        # config/live.yaml
    ros2 launch camslab live.launch.py conn_type:=ap
    ros2 launch camslab live.launch.py config:=/path/to/other.yaml

config/live.yaml has the robot, mocap and launch settings; a launch argument with the
same name overrides a value in it. The controller tracks the trajectory set in
config/params.yaml, with live.yaml `controller` on top.

The robot and the mocap server replace Gazebo:

  vrpn_mocap         client of the VRPN server `mocap_server`:`mocap_port`. Publishes the
                     rigid body `tracker` as /vrpn_mocap/<tracker>/pose, remapped to
                     /<name>/pose, in frame `map`
  robomaster_ros     s1.launch in namespace <name>: the S1 driver takes <name>/cmd_vel
                     (linear.x, angular.z) and stops when no command arrives for
                     `chassis_timeout` seconds
  controller         <name>/pose -> <name>/cmd_vel, ref_pose, ref_path, safety_polytope;
                     parameters from params.yaml, live.yaml `controller` and, when the
                     file exists, `polytope`
  pose_tf            <name>/pose -> TF map -> <name>/mocap
  rviz2, webui_bridge  as in gazebo_multi.launch.py, for the one robot

Every node runs on the wall clock (no use_sim_time). The rigid body in the mocap software
must be named `tracker` (default <name>) and streamed with Z up, since the controller
takes the heading as the yaw about Z.

The controller starts disabled (start_enabled: false): press Start in the web UI, or call
/<name>/controller/enable. robomaster_ros publishes TF <name>/odom -> <name>/base_link from
wheel odometry, which is not connected to `map`, so RViz draws the robot from its mocap
pose (<name>/pose display and the <name>/mocap axes) instead of a RobotModel.
"""

import os
import re
import sys
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import camslab_settings  # noqa: E402


def rviz_config(share, name):
    """Path of an RViz config: rviz/fleet.rviz plus the safety polytope, the mocap pose,
    ref_path and ref_pose of robot `name`."""
    with open(os.path.join(share, "rviz", "fleet.rviz")) as f:
        config = yaml.safe_load(f)
    latched = {"Depth": 1, "Durability Policy": "Transient Local",
               "History Policy": "Keep Last", "Reliability Policy": "Reliable"}
    arrow = {"Shape": "Arrow", "Shaft Length": 0.25, "Shaft Radius": 0.02,
             "Head Length": 0.08, "Head Radius": 0.05}
    config["Visualization Manager"]["Displays"] += [
        {"Class": "rviz_default_plugins/Polygon", "Name": "safety_polytope", "Enabled": True,
         "Topic": {"Value": f"/{name}/safety_polytope", **latched},
         "Color": "220; 40; 40", "Alpha": 1, "Value": True},
        {"Class": "rviz_default_plugins/Pose", "Name": f"{name} pose", "Enabled": True,
         "Topic": {"Value": f"/{name}/pose", "Depth": 5, "Durability Policy": "Volatile",
                   "History Policy": "Keep Last", "Reliability Policy": "Best Effort"},
         "Color": "0; 160; 60", **arrow, "Value": True},
        {"Class": "rviz_default_plugins/Path", "Name": f"{name} ref_path", "Enabled": True,
         "Topic": {"Value": f"/{name}/ref_path", **latched},
         "Color": "235; 104; 52", "Line Style": "Lines", "Value": True},
        {"Class": "rviz_default_plugins/Pose", "Name": f"{name} ref_pose", "Enabled": True,
         "Topic": {"Value": f"/{name}/ref_pose", "Depth": 5, "Durability Policy": "Volatile",
                   "History Policy": "Keep Last", "Reliability Policy": "Reliable"},
         "Color": "235; 104; 52", **arrow, "Value": True},
    ]
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_robomaster_", suffix=".rviz",
                                      delete=False)
    with out:
        yaml.safe_dump(config, out, sort_keys=False)
    return out.name


def robot(context):
    share = get_package_share_directory("camslab")
    settings = camslab_settings.load(context)
    name = str(settings["name"])
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise RuntimeError(f"robot name {name!r} must match [a-z][a-z0-9_]*")
    tracker = str(settings["tracker"] or name)
    # vrpn_mocap names the tracker's sub-node, and so its topic, after the tracker name
    # with every run of characters other than [a-zA-Z0-9_] replaced by one _.
    tracker_ns = re.sub(r"_+", "_", re.sub(r"[^a-zA-Z0-9_]", "_", tracker))
    start_enabled = bool(settings["start_enabled"])
    polytope = LaunchConfiguration("polytope").perform(context)

    actions = [
        Node(package="vrpn_mocap", executable="client_node", name="vrpn_mocap_client_node",
             namespace="vrpn_mocap", output="screen",
             parameters=[{"server": str(settings["mocap_server"]),
                          "port": int(settings["mocap_port"]),
                          "frame_id": "map", "update_freq": 100.0, "refresh_freq": 1.0,
                          "multi_sensor": False, "use_vrpn_timestamps": False}],
             remappings=[(f"/vrpn_mocap/{tracker_ns}/pose", f"/{name}/pose")]),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory("robomaster_ros"), "launch", "s1.launch")),
            launch_arguments={"name": name, "serial_number": str(settings["serial_number"]),
                              "conn_type": str(settings["conn_type"]),
                              "chassis_timeout": str(float(settings["chassis_timeout"]))
                              }.items()),
    ]

    files = [LaunchConfiguration("params").perform(context),
             camslab_settings.controller_file(settings)]
    if polytope and os.path.isfile(polytope):
        files.append(polytope)
    actions += [
        Node(package="camslab", executable="controller", namespace=name, output="screen",
             parameters=files + [{"start_enabled": start_enabled}]),
        Node(package="camslab", executable="pose_tf", namespace=name,
             parameters=[{"child_frame": f"{name}/mocap"}]),
    ]

    if settings["rviz"]:
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_config(share, name)]))
    if settings["webui"]:
        actions.append(Node(
            package="camslab_webui", executable="webui_bridge", output="screen",
            parameters=[{"backend_label": f"RoboMaster S1, {name}",
                         "port": int(settings["port"]), "start_enabled": start_enabled,
                         "robot_name": name, "polytope_file": polytope}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("camslab")
    return LaunchDescription([
        *camslab_settings.arguments(os.path.join(share, "config", "live.yaml")),
        DeclareLaunchArgument("params", default_value=os.path.join(share, "config", "params.yaml"),
                              description="controller parameter file"),
        DeclareLaunchArgument("polytope",
                              default_value=camslab_settings.default_polytope_file(__file__),
                              description="safety polytope file: loaded when it exists, and "
                              "where the web UI saves it; empty for none"),
        OpaqueFunction(function=robot),
    ])
