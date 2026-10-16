"""Runs control() at a fixed rate on the latest pose and publishes cmd_vel."""

import dataclasses
import math

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, Pose, PoseStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_srvs.srv import SetBool

from .controller import control
from .safety import Polytope, check_inside
from .trajectory import check_limits, make_shape, reference
from .types import Params, State

TRAJECTORY_FIELDS = ("x0", "y0", "theta0", "circle_speed", "kappa", "a", "b", "period",
                     "phi0", "kappa_max", "w_max")
GAIN_FIELDS = ("k_par", "k_perp", "k_q", "v_max")
ZERO_SECONDS = 1.0
PATH_POINTS = 600


def state_from_pose(pose: Pose) -> State | None:
    o = pose.orientation
    hx = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
    hy = 2.0 * (o.x * o.y + o.w * o.z)
    norm = math.hypot(hx, hy)
    if norm < 1e-6:
        return None
    return State.from_pose(pose.position.x, pose.position.y, math.atan2(hy, hx))


def pose_from(p, q) -> Pose:
    half_yaw = 0.5 * math.atan2(q[1], q[0])
    pose = Pose()
    pose.position.x = float(p[0])
    pose.position.y = float(p[1])
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)
    return pose


def make_polytope(xy) -> Polytope | None:
    return Polytope(xy) if xy else None


def limit_warning(params: Params, t_max: float) -> str:
    try:
        check_limits(make_shape(params), params, t_max)
    except ValueError as exc:
        return str(exc)
    return ""


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        defaults = Params()
        self.params = Params(
            shape=self.declare_parameter("shape", defaults.shape).value,
            control_points=tuple(self.declare_parameter(
                "control_points", list(defaults.control_points)).value),
            **{f: self.declare_parameter(f, getattr(defaults, f)).value
               for f in TRAJECTORY_FIELDS + GAIN_FIELDS})
        self.t_max = self.declare_parameter("t_max", 60.0).value
        self.margin = self.declare_parameter("polytope_margin", 0.2).value
        polytope = self.declare_parameter("polytope", Parameter.Type.DOUBLE_ARRAY).value
        if polytope is None:
            self.set_parameters([Parameter("polytope", Parameter.Type.DOUBLE_ARRAY, [])])
        self.polytope = make_polytope(list(polytope or []))
        self.pose_timeout = self.declare_parameter("pose_timeout", 0.5).value
        self.frame_id = self.declare_parameter("frame_id", "map").value
        start_enabled = self.declare_parameter("start_enabled", False).value
        rate = self.declare_parameter("rate", 50.0).value

        self.enabled = False
        self.enable_error = ""
        self.trajectory_error = ""
        self.shape = None
        self.state = State.from_pose(0.0, 0.0, 0.0)
        self.pose_time = None
        self.t0 = None
        self.last_drive = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.ref_pose_pub = self.create_publisher(PoseStamped, "ref_pose", 10)
        self.ref_path_pub = self.create_publisher(Path, "ref_path", latched)
        self.polytope_pub = self.create_publisher(PolygonStamped, "safety_polytope", latched)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)
        self.create_service(SetBool, "~/enable", self.on_enable)

        self.publish_polytope()
        self.set_trajectory(self.params)
        self.set_enabled(start_enabled)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(1.0 / rate, self.on_timer)

    def seconds(self, since) -> float:
        return (self.get_clock().now() - since).nanoseconds * 1e-9

    def on_pose(self, msg):
        state = state_from_pose(msg.pose)
        if state is not None:
            self.state = state
            self.pose_time = self.get_clock().now()

    def on_enable(self, request, response):
        response.success = self.set_enabled(request.data)
        response.message = "" if response.success else self.enable_error
        return response

    def trajectory_problem(self, params: Params, t_max: float, polytope, margin) -> str:
        try:
            shape = make_shape(params)
        except ValueError as exc:
            return str(exc)
        return check_inside(shape, polytope, margin, t_max) if polytope else ""

    def set_trajectory(self, params: Params):
        self.params = params
        self.trajectory_error = self.trajectory_problem(params, self.t_max, self.polytope,
                                                        self.margin)
        warning = limit_warning(params, self.t_max)
        if warning:
            self.get_logger().warning(f"the car cannot track the whole trajectory: {warning}")
        self.t0 = None
        if self.trajectory_error:
            self.get_logger().error(f"trajectory rejected: {self.trajectory_error}")
            if self.enabled:
                self.set_enabled(False)
            return
        self.shape = make_shape(params)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for i in range(PATH_POINTS + 1):
            pose = PoseStamped()
            pose.header = path.header
            p = self.shape.position(self.t_max * i / PATH_POINTS)
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.ref_path_pub.publish(path)

    def publish_polytope(self):
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        if self.polytope is not None:
            msg.polygon.points = [Point32(x=float(x), y=float(y))
                                  for x, y in self.polytope.vertices]
        self.polytope_pub.publish(msg)

    def outside_polytope(self):
        if self.pose_time is not None and self.polytope is not None \
                and not self.polytope.contains(self.state.p):
            return self.state.p
        return None

    def set_enabled(self, enabled: bool) -> bool:
        if enabled and self.trajectory_error:
            self.enable_error = self.trajectory_error
            self.get_logger().error(f"not enabled: {self.enable_error}")
            return False
        p = self.outside_polytope()
        if enabled and p is not None:
            self.enable_error = (f"robot at ({p[0]:.2f}, {p[1]:.2f}) m is outside the safety "
                                 "polytope")
            self.get_logger().error(f"not enabled: {self.enable_error}")
            return False
        self.enabled = enabled
        self.t0 = None
        self.get_logger().info("enabled" if enabled else "disabled")
        return True

    def on_timer(self):
        pose_fresh = self.pose_time is not None and self.seconds(self.pose_time) < self.pose_timeout
        if self.enabled and self.pose_time is not None and not pose_fresh:
            self.get_logger().warning(f"no pose for {self.pose_timeout:.1f} s, commanding zero",
                                      throttle_duration_sec=1.0)
        p = self.outside_polytope()
        if self.enabled and pose_fresh and p is not None:
            self.get_logger().error(f"robot at ({p[0]:.2f}, {p[1]:.2f}) m left the safety "
                                    "polytope, disabling")
            self.set_enabled(False)

        cmd = None
        if self.enabled and pose_fresh and self.shape is not None:
            if self.t0 is None:
                self.t0 = self.get_clock().now()
            try:
                ref = reference(self.shape, self.seconds(self.t0))
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.pose = pose_from(ref.p, ref.q)
                self.ref_pose_pub.publish(msg)
                cmd = control(self.state, ref, self.params)
            except ValueError as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=2.0)

        twist = Twist()
        if cmd is not None and all(math.isfinite(c) for c in cmd):
            v = min(max(cmd[0], -self.params.v_max), self.params.v_max)
            w_limit = min(self.params.w_max, self.params.kappa_max * abs(v))
            twist.linear.x = float(v)
            twist.angular.z = float(min(max(cmd[1], -w_limit), w_limit))
            self.last_drive = self.get_clock().now()
            self.cmd_pub.publish(twist)
        elif self.last_drive is not None and self.seconds(self.last_drive) < ZERO_SECONDS:
            self.cmd_pub.publish(twist)

    def on_parameters(self, changes):
        result = SetParametersResult(successful=True)
        params = self.params
        t_max, pose_timeout = self.t_max, self.pose_timeout
        polytope, margin = self.polytope, self.margin
        trajectory_changed = polytope_changed = False
        for change in changes:
            name, kind = change.name, change.type_
            try:
                if name == "polytope" and kind == Parameter.Type.DOUBLE_ARRAY:
                    polytope = make_polytope(list(change.value))
                    trajectory_changed = polytope_changed = True
                elif name == "shape" and kind == Parameter.Type.STRING:
                    params = dataclasses.replace(params, shape=change.value)
                    trajectory_changed = True
                elif name == "control_points" and kind == Parameter.Type.DOUBLE_ARRAY:
                    if len(change.value) % 2 != 0:
                        raise ValueError(f"control_points needs x, y pairs, got "
                                         f"{len(change.value)} numbers")
                    params = dataclasses.replace(params, control_points=tuple(change.value))
                    trajectory_changed = True
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            if kind != Parameter.Type.DOUBLE:
                continue
            value = change.value
            if name == "t_max":
                t_max = value
                trajectory_changed = True
            elif name == "pose_timeout":
                pose_timeout = value
            elif name == "polytope_margin":
                margin = value
                trajectory_changed = True
            elif name in TRAJECTORY_FIELDS:
                params = dataclasses.replace(params, **{name: value})
                trajectory_changed = True
            elif name in GAIN_FIELDS:
                params = dataclasses.replace(params, **{name: value})
        if trajectory_changed:
            problem = self.trajectory_problem(params, t_max, polytope, margin)
            if problem:
                return SetParametersResult(successful=False, reason=problem)
            result.reason = limit_warning(params, t_max)
        self.t_max, self.pose_timeout = t_max, pose_timeout
        self.polytope, self.margin = polytope, margin
        if polytope_changed:
            self.publish_polytope()
        if trajectory_changed:
            self.set_trajectory(params)
        else:
            self.params = params
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
