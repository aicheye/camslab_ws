"""One Yahboom R2 in a running Gazebo world, in namespace <name>."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

def ign_partition():
    return SetEnvironmentVariable("IGN_PARTITION", EnvironmentVariable(
        "IGN_PARTITION",
        default_value=["camslab_", EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")]))


def default_polytope_file():
    """config/polytope.yaml in the source tree, where the web UI saves the safety polytope.
    Under colcon --symlink-install this launch file links into the source tree, so the file
    is found without a rebuild; otherwise it is the installed config directory."""
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    return os.path.normpath(os.path.join(launch_dir, "..", "config", "polytope.yaml"))


def controller_files(context):
    """params.yaml, then the `controller` file when set."""
    files = [LaunchConfiguration("params").perform(context)]
    overrides = LaunchConfiguration("controller").perform(context)
    if overrides:
        files.append(overrides)
    return files


def controller(context):
    """The controller node, with the `controller` file and then the polytope file after
    params.yaml when they exist."""
    files = controller_files(context)
    polytope = LaunchConfiguration("polytope").perform(context)
    if polytope and os.path.isfile(polytope):
        files.append(polytope)
    start_enabled = LaunchConfiguration("start_enabled").perform(context).lower() == "true"
    return [Node(package="camslab", executable="controller", output="screen",
                 parameters=files + [{"start_enabled": start_enabled}])]


def estimator_kind(value):
    """"" (none), "path" or "fiducial" from an `adaptive` setting; true means path."""
    value = str(value).strip().lower()
    if value in ("", "false", "none", "off"):
        return ""
    if value == "true":
        return "path"
    if value not in ("path", "fiducial"):
        raise RuntimeError(f"adaptive must be false, path or fiducial, got {value!r}")
    return value


def estimator(context):
    """The estimator node when `adaptive` is path or fiducial. It reads the trajectory
    from the same parameter files as the controller, whose values are under
    /**/controller, so they are copied into its own parameters here. The fiducial
    estimator also loads the `fiducial` file."""
    kind = estimator_kind(LaunchConfiguration("adaptive").perform(context))
    if not kind:
        return []
    params = {}
    for path in controller_files(context):
        with open(path) as f:
            for key, value in (yaml.safe_load(f) or {}).items():
                if key.rstrip("/").endswith("controller"):
                    params.update(value.get("ros__parameters", {}))
    params["estimator"] = kind
    files = [params]
    if kind == "fiducial":
        files.insert(0, LaunchConfiguration("fiducial").perform(context))
    return [Node(package="camslab", executable="estimator", output="screen",
                 parameters=files)]


def generate_launch_description():
    share = get_package_share_directory("camslab")
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
        DeclareLaunchArgument("controller", default_value="",
                              description="controller parameter file loaded after `params`; "
                              "empty for none"),
        DeclareLaunchArgument("polytope", default_value=default_polytope_file(),
                              description="safety polytope file saved by the web UI; loaded "
                              "when it exists"),
        DeclareLaunchArgument("color", default_value="0 0.63 0.24 1",
                              description="RViz body colour \"r g b a\" in [0, 1]"),
        DeclareLaunchArgument("world", default_value="camslab", description="Gazebo world name"),
        DeclareLaunchArgument("start_enabled", default_value="true",
                              description="controller drives as soon as it has a pose"),
        DeclareLaunchArgument("adaptive", default_value="false",
                              description="run an adaptive estimator next to the "
                              "controller: false, path or fiducial"),
        DeclareLaunchArgument("fiducial",
                              default_value=os.path.join(share, "config", "fiducial.yaml"),
                              description="camera and marker file of the fiducial "
                              "estimator"),

        ign_partition(),
        GroupAction([
            PushRosNamespace(name),
            SetParameter("use_sim_time", True),
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-world", world, "-name", name,
                            "-file", os.path.join(share, "models", "r2", "model.sdf"),
                            "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                            "-z", "0.01", "-Y", LaunchConfiguration("yaw")]),
            Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
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
            OpaqueFunction(function=controller),
            OpaqueFunction(function=estimator),
            Node(package="robot_state_publisher", executable="robot_state_publisher",
                 parameters=[{"robot_description": robot_description,
                              "frame_prefix": [name, "/"], "publish_frequency": 50.0}]),
            Node(package="camslab", executable="pose_tf",
                 parameters=[{"child_frame": [name, "/base_footprint"]}]),
        ]),
    ])
