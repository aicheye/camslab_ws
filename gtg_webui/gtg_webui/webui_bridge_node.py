"""Web UI backend for the Gazebo fleet. Streams ROS 2 topics to the browser.

Subscribes: pose (geometry_msgs/PoseStamped), cmd_vel (geometry_msgs/Twist),
            goal (geometry_msgs/PoseStamped): goals set elsewhere, such as RViz's
            2D Goal Pose tool, are shown in the UI
Publishes:  goal (geometry_msgs/PoseStamped, heading 0)
Calls:      <controller_node>/enable, <controller_node>/set_parameters,
            <controller_node>/get_parameters, /<follower>/controller_node/enable

`followers` lists namespaces of other robots (gazebo_multi.launch.py) and `follows` the
namespace each one follows. /<follower>/pose, /<follower>/cmd_vel and the follower's goal
/<follows>/follow_goal are streamed as "fleet" messages, and Start and Stop also enable
and disable /<follower>/controller_node. Gains are set on <controller_node> only.

rclpy spins in a background thread. aiohttp runs in the main thread.
"""

import logging
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool

from .server import Backend, UiServer, fleet_msg, hello_msg, samples_msg, status_msg

# Controller parameters shown in the UI.
PARAM_NAMES = ["v_star", "d_switch", "k_v", "k_w", "epsilon", "v_max", "w_max"]
MAX_SAMPLES = 20000


class WebuiBridgeNode(Node, Backend):
    # The Backend method is on_message because rclpy.node.Node already defines "handle".
    def __init__(self):
        super().__init__("webui_bridge_node")
        self.label = self.declare_parameter("backend_label", "Gazebo").value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        controller = self.declare_parameter("controller_node", "controller_node").value
        self.robot_name = self.declare_parameter("robot_name", "").value
        self.followers = self.declare_parameter(
            "followers", rclpy.Parameter.Type.STRING_ARRAY).value or []
        follows = self.declare_parameter(
            "follows", rclpy.Parameter.Type.STRING_ARRAY).value or []
        if len(follows) != len(self.followers):
            raise ValueError("follows needs one entry per entry of followers")
        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", 8000).value
        sample_rate = self.declare_parameter("sample_rate", 30.0).value
        self.sample_period = 1.0 / sample_rate

        self.lock = threading.Lock()
        self.samples = []
        self.fleet = {name: [] for name in self.followers}
        self.fleet_last_sample = {name: -math.inf for name in self.followers}
        self.fleet_cmd = {name: (0.0, 0.0) for name in self.followers}
        self.fleet_goal = {name: None for name in self.followers}
        self.t0 = None
        self.last_sample_time = -math.inf
        self.last_pose_time = None
        self.cmd = (0.0, 0.0)
        self.goal = None
        self.params = {}
        # Must match the controllers' start_enabled, so the UI status is right before Start.
        self.enabled = self.declare_parameter("start_enabled", False).value
        self.status = status_msg("idle")

        self.server = UiServer(self, host, port)

        self.goal_pub = self.create_publisher(PoseStamped, "goal", 10)
        self.create_subscription(PoseStamped, "goal", self.on_goal, 10)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.enable_client = self.create_client(SetBool, f"{controller}/enable")
        self.extra_enable_clients = [
            self.create_client(SetBool, f"/{name}/controller_node/enable")
            for name in self.followers]
        for name, leader in zip(self.followers, follows):
            self.create_subscription(
                PoseStamped, f"/{name}/pose",
                lambda msg, name=name: self.on_follower_pose(name, msg), qos_profile_sensor_data)
            self.create_subscription(
                Twist, f"/{name}/cmd_vel",
                lambda msg, name=name: self.fleet_cmd.__setitem__(
                    name, (msg.linear.x, msg.angular.z)), 10)
            self.create_subscription(
                PoseStamped, f"/{leader}/follow_goal",
                lambda msg, name=name: self.fleet_goal.__setitem__(
                    name, [msg.pose.position.x, msg.pose.position.y]), 10)
        self.set_params_client = self.create_client(SetParameters, f"{controller}/set_parameters")
        self.get_params_client = self.create_client(GetParameters, f"{controller}/get_parameters")
        self.create_timer(0.2, self.on_status_timer)
        self.create_timer(1.0, self.fetch_params)

    # ------------------------------------------------------------ ROS thread

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_goal(self, msg):
        """Also receives this node's own goals; it only updates the UI, so there is no loop."""
        goal = [msg.pose.position.x, msg.pose.position.y]
        with self.lock:
            changed = goal != self.goal
            self.goal = goal
        if changed:
            self.server.broadcast_threadsafe(self.config_msg())

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def planar(self, msg):
        """(x, y, qx, qy) of a PoseStamped, or None when the body x axis is vertical."""
        o = msg.pose.orientation
        hx = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        hy = 2.0 * (o.x * o.y + o.w * o.z)
        norm = math.hypot(hx, hy)
        if norm < 1e-6:
            return None
        return msg.pose.position.x, msg.pose.position.y, hx / norm, hy / norm

    def on_follower_pose(self, name, msg):
        now = self.now()
        if now - self.fleet_last_sample[name] < self.sample_period:
            return
        pose = self.planar(msg)
        if pose is None:
            return
        x, y, _, _ = pose
        goal = self.fleet_goal[name]
        v, w = self.fleet_cmd[name]
        dist = math.hypot(goal[0] - x, goal[1] - y) if goal else 0.0
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            row = [now - self.t0, *pose, v, w, dist]
            self.fleet[name].append(row)
            del self.fleet[name][:-MAX_SAMPLES]
        self.fleet_last_sample[name] = now
        self.server.broadcast_threadsafe(fleet_msg({name: [row]}, {name: goal} if goal else {}))

    def on_pose(self, msg):
        now = self.now()
        self.last_pose_time = now
        if now - self.last_sample_time < self.sample_period:
            return
        pose = self.planar(msg)
        if pose is None:
            return
        x, y, qx, qy = pose
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            dist = math.hypot(self.goal[0] - x, self.goal[1] - y) if self.goal else 0.0
            row = [now - self.t0, x, y, qx, qy, self.cmd[0], self.cmd[1], dist]
            self.samples.append(row)
            del self.samples[:-MAX_SAMPLES]
        self.last_sample_time = now
        self.server.broadcast_threadsafe(samples_msg([row]))

    def on_status_timer(self):
        pose_topic = self.resolve_topic_name("pose")
        with self.lock:
            last = self.samples[-1] if self.samples else None
            epsilon = self.params.get("epsilon")
            if self.last_pose_time is None or self.now() - self.last_pose_time > 1.0:
                status = status_msg("running" if self.enabled else "idle",
                                    f"no pose on {pose_topic}")
            elif self.enabled and self.goal and epsilon and last and last[7] < epsilon:
                status = status_msg("reached")
            elif self.enabled and not self.goal:
                status = status_msg("running", "no goal set")
            else:
                status = status_msg("running" if self.enabled else "idle")
            changed = status != self.status
            self.status = status
        if changed:
            self.server.broadcast_threadsafe(status)

    def fetch_params(self):
        if self.params or not self.get_params_client.service_is_ready():
            return
        future = self.get_params_client.call_async(GetParameters.Request(names=PARAM_NAMES))
        future.add_done_callback(self.on_params_fetched)

    def on_params_fetched(self, future):
        values = future.result().values
        with self.lock:
            for name, value in zip(PARAM_NAMES, values):
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    self.params[name] = value.double_value
        self.server.broadcast_threadsafe(self.config_msg())

    # -------------------------------------------------------- asyncio thread

    def config_msg(self):
        with self.lock:
            return {"type": "config", "params": dict(self.params), "goal": self.goal,
                    "initial": None}

    def on_connect(self):
        with self.lock:
            samples = list(self.samples)
            fleet = {name: list(rows) for name, rows in self.fleet.items()}
            goals = {name: g for name, g in self.fleet_goal.items() if g}
            status = self.status
        # Robots spawn where the fleet file says, so the UI has no initial-pose controls.
        return [hello_msg(self.label, "live", False, self.robot_name), self.config_msg(),
                samples_msg(samples, reset=True), fleet_msg(fleet, goals, reset=True), status]

    async def on_message(self, msg):
        kind = msg.get("type")
        if kind == "set_goal":
            with self.lock:
                self.goal = [float(msg["x"]), float(msg["y"])]
            self.publish_goal()
            await self.server.broadcast(self.config_msg())
        elif kind == "set_params":
            self.set_params({k: float(v) for k, v in msg["params"].items() if k in PARAM_NAMES})
        elif kind == "start":
            self.publish_goal()
            self.set_enabled(True)
        elif kind == "stop":
            self.set_enabled(False)
        elif kind == "reset":
            self.set_enabled(False)
            with self.lock:
                self.samples = []
                self.fleet = {name: [] for name in self.followers}
                self.t0 = None
            await self.server.broadcast(samples_msg([], reset=True))
            await self.server.broadcast(fleet_msg({}, reset=True))

    def publish_goal(self):
        if self.goal is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x, msg.pose.position.y = self.goal
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def set_enabled(self, enabled):
        if not self.enable_client.service_is_ready():
            self.report_error(f"{self.enable_client.srv_name} is not available")
            return
        future = self.enable_client.call_async(SetBool.Request(data=enabled))

        def done(f):
            if f.result().success:
                self.enabled = enabled
        future.add_done_callback(done)
        for client in self.extra_enable_clients:
            if client.service_is_ready():
                client.call_async(SetBool.Request(data=enabled))
            else:
                self.report_error(f"{client.srv_name} is not available")

    def set_params(self, params):
        if not self.set_params_client.service_is_ready():
            self.report_error(f"{self.set_params_client.srv_name} is not available")
            return
        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            for name, value in params.items()])
        future = self.set_params_client.call_async(request)

        def done(f):
            with self.lock:
                for (name, value), result in zip(params.items(), f.result().results):
                    if result.successful:
                        self.params[name] = value
            self.server.broadcast_threadsafe(self.config_msg())
        future.add_done_callback(done)

    def report_error(self, message):
        self.get_logger().error(message)
        self.server.broadcast_threadsafe(status_msg("error", message))


def main(args=None):
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
    # aiohttp handles SIGINT and SIGTERM. rclpy is shut down after the server exits.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WebuiBridgeNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        node.server.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
