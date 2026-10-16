"""Runs the adaptive path estimator on a robot's pose and publishes the estimate."""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger

from .adaptive import COLUMNS, Estimator, Path
from .types import Params

PATH_NAMES = ("shape", "x0", "y0", "theta0", "kappa", "circle_speed", "a", "b", "period",
              "phi0", "control_points", "kappa_max", "w_max", "v_max")


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
    if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return tuple(value.double_array_value)
    return None


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator", automatically_declare_parameters_from_overrides=True)
        rate = float(self.param("rate", 50.0))
        self.pose_timeout = float(self.param("pose_timeout", 0.5))
        self.path = {n: self.get_parameter(n).value for n in PATH_NAMES if self.has_parameter(n)}

        self.estimate_pub = self.create_publisher(Float64MultiArray, "estimate", 10)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(ParameterEvent, "/parameter_events", self.on_parameter_event,
                                 10)
        self.create_service(Trigger, "~/reset", self.on_reset)

        self.p = None
        self.pose_time = None
        self.last_tick = None
        self.estimator = None
        self.true_path = None
        self.rebuild()
        self.create_timer(1.0 / rate, self.on_timer)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rebuild(self):
        """New estimator from the current path."""
        self.last_tick = None
        params = Params(**{k: v for k, v in self.path.items() if hasattr(Params, k)})
        try:
            self.true_path = Path(params)
        except ValueError as exc:
            self.estimator = None
            self.get_logger().error(str(exc))
            return
        self.estimator = Estimator(params)
        self.get_logger().info(f"estimator on {params.shape}")

    def on_pose(self, msg):
        self.p = np.array([msg.pose.position.x, msg.pose.position.y])
        self.pose_time = self.now()

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
        response.message = "" if response.success else "estimator not running"
        return response

    def on_timer(self):
        now = self.now()
        if self.estimator is None or self.pose_time is None:
            return
        if now - self.pose_time > self.pose_timeout:
            return
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        self.estimator.update(self.p, self.true_path.xi(self.p), dt)
        self.estimate_pub.publish(array_msg(
            COLUMNS, [*self.estimator.estimate(), *self.true_path.truth(self.p)]))


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
