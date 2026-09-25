"""One Yahboom R2 in a running Gazebo world, in namespace <name>.

    ros2 launch gtg_ros robot.launch.py name:=badger x:=1.9 y:=1.0

gazebo_multi.launch.py includes this once per robot in the fleet file. Everything below
runs in namespace <name>:

  Gazebo model <name>, spawned into the world `gtg`
  parameter_bridge   /model/<name>/pose -> pose,  cmd_vel -> /model/<name>/cmd_vel,
                     /world/<world>/model/<name>/joint_state -> joint_states
  controller_node    pose -> cmd_vel, reference, reference_path
  robot_state_publisher   r2.urdf.xacro (body colour `color`) + joint_states -> TF
                          <name>/<link>, at most 60 Hz (Gazebo sends joint_states every
                          1 ms physics step)
  pose_tf_node       pose -> TF map -> <name>/base_footprint

Every node uses the Gazebo simulation clock (use_sim_time), so TF from the pose and from
the joint states share one time base. gazebo_multi.launch.py bridges /clock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("gtg_ros")
    name = LaunchConfiguration("name")
    world = LaunchConfiguration("world")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "r2.urdf.xacro"),
                 " body_color:='", LaunchConfiguration("color"), "'"]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("name", description="robot name: namespace, Gazebo model, TF prefix"),
        DeclareLaunchArgument("x", default_value="0.0", description="spawn x [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="spawn y [m]"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="spawn heading [rad]"),
        DeclareLaunchArgument("params", default_value=os.path.join(share, "config", "params.yaml"),
                              description="controller parameter file"),
        DeclareLaunchArgument("color", default_value="0 0.63 0.24 1",
                              description="RViz body colour \"r g b a\" in [0, 1]"),
        DeclareLaunchArgument("world", default_value="gtg", description="Gazebo world name"),
        DeclareLaunchArgument("start_enabled", default_value="true",
                              description="controller drives as soon as it has a pose"),

        GroupAction([
            PushRosNamespace(name),
            SetParameter("use_sim_time", True),
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", world, "-name", name,
                            "-file", os.path.join(share, "models", "r2", "model.sdf"),
                            "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                            "-z", "0.01", "-Y", LaunchConfiguration("yaw")]),
            Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                 # A list of substitutions is concatenated into one string.
                 arguments=[["/model/", name,
                             "/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose"],
                            ["/model/", name,
                             "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
                            ["/world/", world, "/model/", name,
                             "/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"]],
                 remappings=[(["/model/", name, "/pose"], "pose"),
                             (["/model/", name, "/cmd_vel"], "cmd_vel"),
                             (["/world/", world, "/model/", name, "/joint_state"],
                              "joint_states")]),
            Node(package="gtg_ros", executable="controller_node", output="screen",
                 parameters=[LaunchConfiguration("params"),
                             {"start_enabled": ParameterValue(
                                 LaunchConfiguration("start_enabled"), value_type=bool)}]),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": robot_description,
                              "frame_prefix": [name, "/"], "publish_frequency": 60.0}]),
            Node(package="gtg_ros", executable="pose_tf_node",
                 parameters=[{"child_frame": [name, "/base_footprint"]}]),
        ]),
    ])
