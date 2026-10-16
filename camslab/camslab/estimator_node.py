"""Runs one adaptive estimator on a robot's pose and publishes the estimate."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from .adaptive.base import Measurement
from .adaptive.paths import distance_to_path
from .adaptive.registry import MODES, make_estimator

PATH_NAMES = ("shape", "x0", "y0", "theta0", "kappa", "circle_speed", "a", "b", "period",
              "phi0", "kappa_max", "w_max", "v_max")


def array_msg(names, values):
    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension(label=",".join(names), size=len(values),
                                          stride=len(values))]
    msg.data = [float(v) for v in values]
    return msg


def parameter_value(value):
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    return None


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator", automatically_declare_parameters_from_overrides=True)
        self.name = self.param("estimator", "")
        self.mode = self.param("adaptive_mode", "open")
        if self.mode not in MODES:
            raise ValueError(f"adaptive_mode must be one of {MODES}, got {self.mode!r}")
        rate = float(self.param("rate", 50.0))
        self.pose_timeout = float(self.param("pose_timeout", 0.5))
        self.path = {n: self.get_parameter(n).value for n in PATH_NAMES if self.has_parameter(n)}

        self.estimate_pub = self.create_publisher(Float64MultiArray, "estimate", 10)
        self.truth_pub = self.create_publisher(
            Float64MultiArray, "estimate_true",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reference_pub = self.create_publisher(Float64MultiArray, "reference", 10)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_subscription(ParameterEvent, "/parameter_events", self.on_parameter_event,
                                 10)
        self.create_service(Trigger, "~/reset", self.on_reset)

        self.p = None
        self.q = None
        self.pose_time = None
        self.cmd = (0.0, 0.0)
        self.t0 = None
        self.last_tick = None
        self.estimator = None
        self.rebuild()
        self.create_timer(1.0 / rate, self.on_timer)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rebuild(self):
        """New estimator from the current path, clock restarted."""
        self.t0 = None
        self.last_tick = None
        try:
            self.estimator = make_estimator(self.name, self.path)
        except ValueError as exc:
            self.estimator = None
            self.get_logger().error(str(exc))
            return
        truth = self.estimator.truth()
        if truth is not None:
            self.truth_pub.publish(array_msg(self.estimator.fields, truth))
        self.get_logger().info(f"estimator {self.name} ({self.mode} loop) on "
                               f"{self.path.get('shape')}")

    def on_pose(self, msg):
        o = msg.pose.orientation
        hx = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        hy = 2.0 * (o.x * o.y + o.w * o.z)
        norm = math.hypot(hx, hy)
        if norm < 1e-6:
            return
        self.p = np.array([msg.pose.position.x, msg.pose.position.y])
        self.q = np.array([hx / norm, hy / norm])
        self.pose_time = self.now()

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_parameter_event(self, event):
        """Follow trajectory changes made on this namespace's controller."""
        if event.node != self.get_namespace().rstrip("/") + "/controller":
            return
        changed = False
        for p in [*event.new_parameters, *event.changed_parameters]:
            if p.name in PATH_NAMES:
                value = parameter_value(p.value)
                if value is not None:
                    self.path[p.name] = value
                    changed = True
        if changed:
            self.rebuild()

    def on_reset(self, request, response):
        self.rebuild()
        response.success = self.estimator is not None
        response.message = "" if response.success else f"estimator {self.name!r} not running"
        return response

    def on_timer(self):
        now = self.now()
        if self.estimator is None or self.pose_time is None:
            return
        if now - self.pose_time > self.pose_timeout:
            return
        if self.t0 is None:
            self.t0 = now
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        m = Measurement(t=now - self.t0, p=self.p, q=self.q, v=self.cmd[0], w=self.cmd[1],
                        xi=distance_to_path(self.path, self.p))
        self.estimator.update(m, dt)
        self.estimate_pub.publish(array_msg(self.estimator.fields, self.estimator.estimate()))
        if self.mode == "closed":
            ref = self.estimator.reference(m, dt)
            if ref is not None:
                self.reference_pub.publish(array_msg(
                    ["p_x", "p_y", "q_x", "q_y", "v", "w", "kappa"],
                    [*ref.p, *ref.q, ref.v, ref.w, ref.kappa]))


def main(args=None):
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
