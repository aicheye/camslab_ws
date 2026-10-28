"""Web UI backend for the Gazebo robots."""

import json
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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from .polytope import convex_hull, write_file
from .random_trajectory import random_params
from .server import (Backend, UiServer, adaptive_config, fleet_msg, hello_msg, history_msgs,
                     samples_msg, status_msg)

PARAM_NAMES = ["shape", "x0", "y0", "theta0", "circle_speed", "kappa", "a", "b", "period",
               "phi0", "control_points", "k_par", "k_perp", "k_q", "kappa_max", "w_max",
               "v_max", "t_max"]
POLYTOPE_NAMES = ["polytope", "polytope_margin"]
MAX_SAMPLES = 20000
SAMPLE_SLACK = 0.005
RANDOM_TRIES = 50


def parameter(name, value):
    """rcl_interfaces Parameter of a PARAM_NAMES value."""
    if name == "shape":
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value))
    elif name == "control_points":
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                            double_array_value=[float(v) for v in value])
    else:
        pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    return Parameter(name=name, value=pv)


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
        self.ref = None


class WebuiBridgeNode(Node, Backend):
    def __init__(self):
        super().__init__("webui_bridge")
        self.label = self.declare_parameter("backend_label", "Gazebo").value
        self.robot_name = self.declare_parameter("robot_name", "angostura").value
        others = self.declare_parameter(
            "others", rclpy.Parameter.Type.STRING_ARRAY).value or []
        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", 8000).value
        sample_rate = self.declare_parameter("sample_rate", 25.0).value
        self.polytope_file = self.declare_parameter("polytope_file", "").value
        self.adaptive = self.declare_parameter("adaptive", "").value or ""
        self.estimator = bool(self.adaptive)
        self.estimate_names = []
        self.plots = []
        self.estimate = None
        self.sample_period = 1.0 / sample_rate

        self.lock = threading.Lock()
        self.robots = {name: Robot(name) for name in [self.robot_name, *others]}
        self.main = self.robots[self.robot_name]
        self.t0 = None
        self.path = []
        self.params = {}
        self.polytope = {"points": [], "margin": 0.2}
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
                PoseStamped, f"/{name}/ref_pose",
                lambda msg, robot=robot: setattr(
                    robot, "ref", [msg.pose.position.x, msg.pose.position.y]), 10)
            self.enable_clients.append(
                self.create_client(SetBool, f"/{name}/controller/enable"))
        self.create_subscription(
            Path, f"/{self.robot_name}/ref_path", self.on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reset_client = None
        if self.estimator:
            self.create_subscription(
                Float64MultiArray, f"/{self.robot_name}/estimate", self.on_estimate, 10)
            self.reset_client = self.create_client(
                Trigger, f"/{self.robot_name}/estimator/reset")
        self.set_params_clients = [
            self.create_client(SetParameters, f"/{name}/controller/set_parameters")
            for name in self.robots]
        self.atomic_clients = [
            self.create_client(SetParametersAtomically,
                               f"/{name}/controller/set_parameters_atomically")
            for name in self.robots]
        self.get_params_client = self.create_client(
            GetParameters, f"/{self.robot_name}/controller/get_parameters")
        self.create_timer(0.2, self.on_status_timer)
        self.create_timer(1.0, self.fetch_params)


    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_path(self, msg):
        with self.lock:
            self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.server.broadcast_threadsafe(self.config_msg())

    @staticmethod
    def names_of(msg):
        """(column names, plots) from the layout labels, see
        camslab.estimator_node.array_msg()."""
        dims = msg.layout.dim
        names = dims[0].label.split(",") if dims else []
        plots = json.loads(dims[1].label) if len(dims) > 1 and dims[1].label else []
        return names, plots

    def on_estimate(self, msg):
        names, plots = self.names_of(msg)
        with self.lock:
            self.estimate = [x if math.isfinite(x) else None for x in msg.data]
            changed = names != self.estimate_names or plots != self.plots
            self.estimate_names = names
            self.plots = plots
        if changed:
            self.server.broadcast_threadsafe(self.config_msg())

    def on_pose(self, robot, msg):
        now = self.now()
        robot.last_pose = now
        if now - robot.last_sample < self.sample_period - SAMPLE_SLACK:
            return
        pose = planar(msg)
        if pose is None:
            return
        x, y = pose[0], pose[1]
        ref = robot.ref
        dist = math.hypot(ref[0] - x, ref[1] - y) if ref else 0.0
        xr, yr = ref if ref else (None, None)
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            row = [now - self.t0, *pose, robot.cmd[0], robot.cmd[1], dist, xr, yr]
            if robot is self.main and self.estimator:
                row += self.estimate or [None] * len(self.estimate_names)
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
        future = self.get_params_client.call_async(
            GetParameters.Request(names=PARAM_NAMES + POLYTOPE_NAMES))
        future.add_done_callback(self.on_params_fetched)

    def on_params_fetched(self, future):
        values = future.result().values
        with self.lock:
            for name, value in zip(PARAM_NAMES + POLYTOPE_NAMES, values):
                if name == "polytope":
                    xy = list(value.double_array_value)
                    self.polytope["points"] = [xy[i:i + 2] for i in range(0, len(xy) - 1, 2)]
                elif name == "polytope_margin":
                    if value.type == ParameterType.PARAMETER_DOUBLE:
                        self.polytope["margin"] = value.double_value
                elif value.type == ParameterType.PARAMETER_DOUBLE:
                    self.params[name] = value.double_value
                elif value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                    self.params[name] = list(value.double_array_value)
                elif value.type == ParameterType.PARAMETER_STRING:
                    self.params[name] = value.string_value
        self.server.broadcast_threadsafe(self.config_msg())


    def config_msg(self):
        with self.lock:
            adaptive = None
            if self.estimator:
                adaptive = adaptive_config(self.adaptive, self.estimate_names,
                                           editable=False, plots=self.plots)
            return {"type": "config", "params": dict(self.params), "path": list(self.path),
                    "initial": None,
                    "polytope": {**self.polytope, "file": self.polytope_file},
                    "adaptive": adaptive}

    def on_connect(self):
        with self.lock:
            samples = list(self.main.rows)
            fleet = {r.name: list(r.rows) for r in self.robots.values() if r is not self.main}
            status = self.status
        return [hello_msg(self.label, "live", False, self.robot_name, can_set_polytope=True),
                self.config_msg(),
                *history_msgs(samples), fleet_msg(fleet, reset=True), status]

    async def on_message(self, msg):
        kind = msg.get("type")
        if kind == "set_params":
            self.set_params({k: v for k, v in msg["params"].items() if k in PARAM_NAMES})
        elif kind == "randomize_trajectory":
            self.randomize()
        elif kind == "set_polytope":
            self.set_polytope(msg["points"], float(msg["margin"]))
        elif kind == "start":
            self.reset_estimator()
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

    def reset_estimator(self):
        if self.reset_client is None:
            return
        if not self.reset_client.service_is_ready():
            self.report_error(f"{self.reset_client.srv_name} is not available")
            return
        with self.lock:
            self.estimate = None
        self.reset_client.call_async(Trigger.Request())

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
        parameters = [parameter(name, value) for name, value in params.items()]
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
                        if result.reason:
                            self.report_warning(result.reason)
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
            parameter(name, value) for name, value in candidate.items()])
        main, others = self.atomic_clients[0], self.atomic_clients[1:]
        if not main.service_is_ready():
            self.report_error(f"{main.srv_name} is not available")
            return

        def done(f):
            result = f.result().result
            if (not result.successful or result.reason) and tries_left > 1:
                self.randomize(tries_left - 1)
                return
            if not result.successful:
                self.report_error(f"no random {params['shape']} accepted in "
                                  f"{RANDOM_TRIES} tries: {result.reason}")
                return
            if result.reason:
                self.report_warning(result.reason)
            for client in others:
                if client.service_is_ready():
                    client.call_async(request)
                else:
                    self.report_error(f"{client.srv_name} is not available")
            with self.lock:
                self.params.update(candidate)
            self.server.broadcast_threadsafe(self.config_msg())
        main.call_async(request).add_done_callback(done)

    def set_polytope(self, points, margin):
        """Set the convex hull of points as every controller's safety polytope, first
        controller first, and save it. Empty points removes the polytope."""
        corners = convex_hull(points)
        if 0 < len(corners) < 3:
            self.report_error("the safety polytope needs at least 3 corners that are not "
                              "on one line")
            return
        if margin < 0:
            self.report_error("the safety polytope margin must be 0 or more")
            return
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name="polytope", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[c for corner in corners for c in corner])),
            Parameter(name="polytope_margin", value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=margin))])
        main, others = self.atomic_clients[0], self.atomic_clients[1:]
        if not main.service_is_ready():
            self.report_error(f"{main.srv_name} is not available")
            return

        def done(f):
            result = f.result().result
            if not result.successful:
                self.report_error(f"safety polytope rejected: {result.reason}")
                return
            for client in others:
                if client.service_is_ready():
                    client.call_async(request)
                else:
                    self.report_error(f"{client.srv_name} is not available")
            with self.lock:
                self.polytope = {"points": corners, "margin": margin}
            self.server.broadcast_threadsafe(self.config_msg())
            if not self.polytope_file:
                self.report_error("safety polytope applied but not saved: the bridge has "
                                  "no polytope_file")
                return
            try:
                write_file(self.polytope_file, corners, margin)
            except OSError as exc:
                self.report_error(f"safety polytope applied but not saved: {exc}")
        main.call_async(request).add_done_callback(done)

    def report_warning(self, message):
        """Shows message in the UI header without changing the idle or running state."""
        self.get_logger().warning(message)
        with self.lock:
            state = self.status["state"]
        self.server.broadcast_threadsafe(
            status_msg(state, f"the car cannot track the whole trajectory: {message}"))

    def report_error(self, message):
        self.get_logger().error(message)
        self.server.broadcast_threadsafe(status_msg("error", message))


def main(args=None):
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s")
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
