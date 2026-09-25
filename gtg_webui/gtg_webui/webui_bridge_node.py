"""Web UI backend for the Gazebo robots. Streams ROS 2 topics to the browser.

Per robot <name>, subscribes /<name>/pose (geometry_msgs/PoseStamped),
/<name>/cmd_vel (geometry_msgs/Twist), /<name>/reference (geometry_msgs/PoseStamped),
and for the first robot /<name>/reference_path (nav_msgs/Path), drawn on the map.
Calls:      /<name>/controller_node/enable, set_parameters, set_parameters_atomically,
            get_parameters

`robot_name` is the robot in "samples", which the plots show by default. `others` lists
the other robots, streamed as "fleet" messages. Start and Stop enable and disable every
controller, and parameters set in the UI go to every controller, since all of them track
the same trajectory. The Random button sends random trajectory parameters inside the
area (random_trajectory.py) to the first controller atomically, tries another set when the
controller rejects one for its kappa_max or w_max, and copies the accepted set to the others.

rclpy spins in a background thread. aiohttp runs in the main thread.
"""

import logging
import math
import threading

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool

from .random_trajectory import random_params
from .server import Backend, UiServer, fleet_msg, hello_msg, samples_msg, status_msg

# Controller parameters shown in the UI. shape is a string, the rest are doubles.
PARAM_NAMES = ["shape", "x0", "y0", "theta0", "circle_speed", "kappa", "a", "period",
               "k_par", "k_perp", "k_q", "kappa_max", "w_max", "v_max", "path_time"]
MAX_SAMPLES = 20000
RANDOM_TRIES = 50


def planar(msg):
    """(x, y, qx, qy) of a PoseStamped, or None when the body x axis is vertical."""
    o = msg.pose.orientation
    hx = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    hy = 2.0 * (o.x * o.y + o.w * o.z)
    norm = math.hypot(hx, hy)
    if norm < 1e-6:
        return None
    return msg.pose.position.x, msg.pose.position.y, hx / norm, hy / norm


class Robot:
    """Latest command and reference of one robot, and its sampled rows."""

    def __init__(self, name):
        self.name = name
        self.rows = []
        self.last_sample = -math.inf
        self.last_pose = None
        self.cmd = (0.0, 0.0)
        self.ref = None  # [x, y] of p*


class WebuiBridgeNode(Node, Backend):
    # The Backend method is on_message because rclpy.node.Node already defines "handle".
    def __init__(self):
        super().__init__("webui_bridge_node")
        self.label = self.declare_parameter("backend_label", "Gazebo").value
        self.robot_name = self.declare_parameter("robot_name", "angostura").value
        others = self.declare_parameter(
            "others", rclpy.Parameter.Type.STRING_ARRAY).value or []
        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", 8000).value
        sample_rate = self.declare_parameter("sample_rate", 30.0).value
        self.sample_period = 1.0 / sample_rate

        self.lock = threading.Lock()
        self.robots = {name: Robot(name) for name in [self.robot_name, *others]}
        self.main = self.robots[self.robot_name]
        self.t0 = None
        self.path = []
        self.params = {}
        # Must match the controllers' start_enabled, so the UI status is right before Start.
        self.enabled = self.declare_parameter("start_enabled", False).value
        self.status = status_msg("idle")

        self.server = UiServer(self, host, port)

        self.enable_clients = []
        for name, robot in self.robots.items():
            self.create_subscription(
                PoseStamped, f"/{name}/pose",
                lambda msg, robot=robot: self.on_pose(robot, msg), qos_profile_sensor_data)
            self.create_subscription(
                Twist, f"/{name}/cmd_vel",
                lambda msg, robot=robot: setattr(robot, "cmd", (msg.linear.x, msg.angular.z)),
                10)
            self.create_subscription(
                PoseStamped, f"/{name}/reference",
                lambda msg, robot=robot: setattr(
                    robot, "ref", [msg.pose.position.x, msg.pose.position.y]), 10)
            self.enable_clients.append(
                self.create_client(SetBool, f"/{name}/controller_node/enable"))
        self.create_subscription(
            Path, f"/{self.robot_name}/reference_path", self.on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.set_params_clients = [
            self.create_client(SetParameters, f"/{name}/controller_node/set_parameters")
            for name in self.robots]
        self.atomic_clients = [
            self.create_client(SetParametersAtomically,
                               f"/{name}/controller_node/set_parameters_atomically")
            for name in self.robots]
        self.get_params_client = self.create_client(
            GetParameters, f"/{self.robot_name}/controller_node/get_parameters")
        self.create_timer(0.2, self.on_status_timer)
        self.create_timer(1.0, self.fetch_params)

    # ------------------------------------------------------------ ROS thread

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, msg):
        with self.lock:
            self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.server.broadcast_threadsafe(self.config_msg())

    def on_pose(self, robot, msg):
        now = self.now()
        robot.last_pose = now
        if now - robot.last_sample < self.sample_period:
            return
        pose = planar(msg)
        if pose is None:
            return
        x, y = pose[0], pose[1]
        ref = robot.ref
        dist = math.hypot(ref[0] - x, ref[1] - y) if ref else 0.0
        xr, yr = ref if ref else (None, None)  # null in JSON
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            row = [now - self.t0, *pose, robot.cmd[0], robot.cmd[1], dist, xr, yr]
            robot.rows.append(row)
            del robot.rows[:-MAX_SAMPLES]
        robot.last_sample = now
        if robot is self.main:
            self.server.broadcast_threadsafe(samples_msg([row]))
        else:
            self.server.broadcast_threadsafe(fleet_msg({robot.name: [row]}))

    def on_status_timer(self):
        with self.lock:
            last_pose = self.main.last_pose
            if last_pose is None or self.now() - last_pose > 1.0:
                status = status_msg("running" if self.enabled else "idle",
                                    f"no pose on /{self.robot_name}/pose")
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
                elif value.type == ParameterType.PARAMETER_STRING:
                    self.params[name] = value.string_value
        self.server.broadcast_threadsafe(self.config_msg())

    # -------------------------------------------------------- asyncio thread

    def config_msg(self):
        with self.lock:
            return {"type": "config", "params": dict(self.params), "path": list(self.path),
                    "initial": None}

    def on_connect(self):
        with self.lock:
            samples = list(self.main.rows)
            fleet = {r.name: list(r.rows) for r in self.robots.values() if r is not self.main}
            status = self.status
        # Robots spawn where the fleet file says, so the UI has no initial-pose controls.
        return [hello_msg(self.label, "live", False, self.robot_name), self.config_msg(),
                samples_msg(samples, reset=True), fleet_msg(fleet, reset=True), status]

    async def on_message(self, msg):
        kind = msg.get("type")
        if kind == "set_params":
            self.set_params({k: v for k, v in msg["params"].items() if k in PARAM_NAMES})
        elif kind == "randomize_trajectory":
            self.randomize()
        elif kind == "start":
            self.set_enabled(True)
        elif kind == "stop":
            self.set_enabled(False)
        elif kind == "reset":
            self.set_enabled(False)
            with self.lock:
                for robot in self.robots.values():
                    robot.rows = []
                self.t0 = None
            await self.server.broadcast(samples_msg([], reset=True))
            await self.server.broadcast(fleet_msg({}, reset=True))

    def set_enabled(self, enabled):
        for i, client in enumerate(self.enable_clients):
            if not client.service_is_ready():
                self.report_error(f"{client.srv_name} is not available")
                continue
            future = client.call_async(SetBool.Request(data=enabled))

            def done(f, main=(i == 0)):
                result = f.result()
                if not result.success:
                    self.report_error(result.message or "controller refused to enable")
                elif main:
                    self.enabled = enabled
            future.add_done_callback(done)

    def set_params(self, params):
        parameters = []
        for name, value in params.items():
            if name == "shape":
                pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
            else:
                pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            parameters.append(Parameter(name=name, value=pv))
        for i, client in enumerate(self.set_params_clients):
            if not client.service_is_ready():
                self.report_error(f"{client.srv_name} is not available")
                continue
            future = client.call_async(SetParameters.Request(parameters=parameters))

            def done(f, main=(i == 0)):
                for (name, value), result in zip(params.items(), f.result().results):
                    if not result.successful:
                        self.report_error(f"{name} rejected: {result.reason}")
                    elif main:
                        with self.lock:
                            self.params[name] = value
                if main:
                    self.server.broadcast_threadsafe(self.config_msg())
            future.add_done_callback(done)

    def randomize(self, tries_left=RANDOM_TRIES):
        """Send random trajectory parameters to the first controller until it accepts a
        set, then send that set to the other controllers."""
        with self.lock:
            params = dict(self.params)
        needed = ("shape", "kappa_max", "w_max", "circle_speed")
        if any(name not in params for name in needed):
            self.report_error("controller parameters are not loaded yet")
            return
        try:
            candidate = random_params(params["shape"], params)
        except ValueError as exc:
            self.report_error(str(exc))
            return
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            for name, value in candidate.items()])
        main, others = self.atomic_clients[0], self.atomic_clients[1:]
        if not main.service_is_ready():
            self.report_error(f"{main.srv_name} is not available")
            return

        def done(f):
            result = f.result().result
            if not result.successful:
                if tries_left > 1:
                    self.randomize(tries_left - 1)
                else:
                    self.report_error(f"no random {params['shape']} accepted in "
                                      f"{RANDOM_TRIES} tries: {result.reason}")
                return
            for client in others:
                if client.service_is_ready():
                    client.call_async(request)
                else:
                    self.report_error(f"{client.srv_name} is not available")
            with self.lock:
                self.params.update(candidate)
            self.server.broadcast_threadsafe(self.config_msg())
        main.call_async(request).add_done_callback(done)

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
