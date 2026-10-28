"""Runs an adaptive estimator on a robot's pose and publishes the estimate.

`estimator` selects camslab.adaptive ("path": the circle that fits the path) or
camslab.fiducial ("fiducial": the camera extrinsic and the marker poses, with the pose
sensor simulated from the parameters in config/fiducial.yaml).
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import ColorRGBA, Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from . import adaptive, fiducial
from .types import Params

PATH_NAMES = ("shape", "x0", "y0", "theta0", "kappa", "circle_speed", "a", "b", "period",
              "phi0", "control_points", "kappa_max", "w_max", "v_max")
FIDUCIAL_GAINS = ("gamma_x", "gamma_t", "gamma_m", "gamma_j", "alpha_x", "alpha_t",
                  "psi_min", "straight_tol", "d_min", "d_step", "l_buffer")
MARKER_RATE = 10.0
TRUE_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.6)
ESTIMATE_COLOR = ColorRGBA(r=0.12, g=0.62, b=0.54, a=1.0)


def array_msg(names, values, plots=()):
    """Float64MultiArray labelled with the column names; a second dim carries the web UI
    plots of camslab_webui.server.adaptive_config() as JSON."""
    msg = Float64MultiArray()
    msg.layout.dim = [MultiArrayDimension(label=",".join(names), size=len(values),
                                          stride=len(values))]
    if plots:
        msg.layout.dim.append(MultiArrayDimension(label=json.dumps(list(plots)), size=0,
                                                  stride=0))
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


def pose_matrix(msg: PoseStamped) -> np.ndarray:
    """W_T_R from a PoseStamped, full orientation and height."""
    o = msg.pose.orientation
    x, y, z, w = o.x, o.y, o.z, o.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return fiducial.pose(R, [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


def quaternion_of(R: np.ndarray):
    """(x, y, z, w) of a rotation matrix."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s,
                0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return tuple(q)


def text_marker(ns, mid, position, text, color, frame, stamp, height=0.12):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, Marker.TEXT_VIEW_FACING, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
    m.pose.orientation.w = 1.0
    m.scale.z = height
    m.color = color
    m.text = text
    return m


def error_text(e) -> str:
    """The four errors of fiducial.ERROR_FIELDS as two lines for RViz."""
    return (f"|x~| {e[0]:.1e}   |t~X| {1e3 * e[1]:.1f} mm\n"
            f"max |m~j| {e[2]:.1e}   max |t~j| {1e3 * e[3]:.1f} mm")


def pose_marker(ns, mid, T, color, frame, stamp, kind=Marker.CUBE, scale=(0.2, 0.2, 0.02)):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = quaternion_of(T[:3, :3])
    m.pose.orientation.x, m.pose.orientation.y = qx, qy
    m.pose.orientation.z, m.pose.orientation.w = qz, qw
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color = color
    return m


class EstimatorNode(Node):
    def __init__(self):
        super().__init__("estimator", automatically_declare_parameters_from_overrides=True)
        rate = float(self.param("rate", 50.0))
        self.pose_timeout = float(self.param("pose_timeout", 0.5))
        self.kind = str(self.param("estimator", "path"))
        self.frame = str(self.param("frame", "map"))
        self.path = {n: self.get_parameter(n).value for n in PATH_NAMES if self.has_parameter(n)}

        self.estimate_pub = self.create_publisher(Float64MultiArray, "estimate", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "fiducials", 10)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(ParameterEvent, "/parameter_events", self.on_parameter_event,
                                 10)
        self.create_service(Trigger, "~/reset", self.on_reset)

        self.pose = None
        self.pose_time = None
        self.last_tick = None
        self.last_markers = -math.inf
        self.estimator = None
        self.true_path = None
        self.scene = None
        self.rebuild()
        self.create_timer(1.0 / rate, self.on_timer)

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rebuild(self):
        """New estimator from the current parameters."""
        self.last_tick = None
        if self.kind == "fiducial":
            self.rebuild_fiducial()
        elif self.kind == "path":
            self.rebuild_path()
        else:
            self.estimator = None
            self.get_logger().error(f"estimator must be path or fiducial, got {self.kind!r}")

    def rebuild_path(self):
        params = Params(**{k: v for k, v in self.path.items() if hasattr(Params, k)})
        try:
            self.true_path = adaptive.Path(params)
        except ValueError as exc:
            self.estimator = None
            self.get_logger().error(str(exc))
            return
        self.estimator = adaptive.Estimator(params)
        self.get_logger().info(f"path estimator on {params.shape}")

    def rebuild_fiducial(self):
        positions = list(self.param("marker_position", [4.5, 1.0, 0.0, 0.5, 3.0, 0.0,
                                                        3.0, 4.0, 0.4]))
        rpys = list(self.param("marker_rpy", [0.0, 0.0, 2.5, 0.3, 0.0, -0.5, 0.0, 0.4, -1.8]))
        known = list(self.param("marker_height_known", [True, True, False]))
        n = len(positions) // 3
        if len(positions) != 3 * n or len(rpys) != 3 * n or len(known) != n or n == 0:
            self.estimator = None
            self.get_logger().error("marker_position and marker_rpy need 3 values per "
                                    "marker and marker_height_known one")
            return
        markers = [fiducial.Marker(tuple(positions[3 * j:3 * j + 3]),
                                   tuple(rpys[3 * j:3 * j + 3]), bool(known[j]))
                   for j in range(n)]
        self.scene = fiducial.Scene(
            tuple(self.param("camera_cayley", [0.12, -0.08, 0.18])),
            tuple(self.param("camera_position", [0.18, -0.06, 0.32])), markers,
            fov=float(self.param("fov", 2 * math.pi)),
            range=float(self.param("range", 1e9)))
        gains = {name: float(self.get_parameter(name).value)
                 for name in FIDUCIAL_GAINS if self.has_parameter(name)}
        self.estimator = fiducial.Estimator(n, self.scene.known_heights(), **gains)
        self.truth = self.scene.truth()
        self.get_logger().info(f"fiducial estimator with {n} markers")

    def on_pose(self, msg):
        self.pose = msg
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
        if changed and self.kind == "path":
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
        try:
            self.step(now, dt)
        except NotImplementedError as exc:
            self.estimator = None
            self.get_logger().error(f"estimator stopped: {exc} is not implemented, see "
                                    "docs/fiducial-hints.md")

    def step(self, now, dt):
        if self.kind == "path":
            p = np.array([self.pose.pose.position.x, self.pose.pose.position.y])
            self.estimator.update(p, self.true_path.xi(p), dt)
            self.estimate_pub.publish(array_msg(
                adaptive.COLUMNS, [*self.estimator.estimate(), *self.true_path.truth(p)],
                plots=adaptive.PLOTS))
            return
        T_R = pose_matrix(self.pose)
        self.estimator.update(T_R, self.scene.observe(T_R), dt)
        estimate = self.estimator.estimate()
        e = fiducial.errors(estimate, self.truth, self.scene.n)
        self.estimate_pub.publish(array_msg(
            fiducial.columns(self.scene.n),
            [*estimate, *self.truth, *e, *self.estimator.excitation()],
            plots=fiducial.plots(self.estimator)))
        if now - self.last_markers >= 1.0 / MARKER_RATE:
            self.last_markers = now
            self.marker_pub.publish(self.marker_array(T_R, estimate, e))

    def marker_array(self, T_R, estimate, e):
        """RViz markers: the true fiducials and camera in grey, the estimates in green, and
        the four errors of fiducial.ERROR_FIELDS as text above the car."""
        n = self.scene.n
        stamp = self.get_clock().now().to_msg()
        msg = MarkerArray()
        for j, Y in enumerate(self.scene.marker_poses()):
            msg.markers.append(pose_marker("true", j, Y, TRUE_COLOR, self.frame, stamp))
            m_hat = estimate[6 + 3 * j:9 + 3 * j]
            t_hat = estimate[6 + 3 * n + 3 * j:9 + 3 * n + 3 * j]
            msg.markers.append(pose_marker(
                "estimate", j, fiducial.pose(fiducial.cayley_rotation(m_hat), t_hat),
                ESTIMATE_COLOR, self.frame, stamp))
        arrow = dict(kind=Marker.ARROW, scale=(0.25, 0.03, 0.03))
        msg.markers.append(pose_marker("true", n, T_R @ self.scene.camera(), TRUE_COLOR,
                                       self.frame, stamp, **arrow))
        X_hat = fiducial.pose(fiducial.cayley_rotation(estimate[0:3]), estimate[3:6])
        msg.markers.append(pose_marker("estimate", n, T_R @ X_hat, ESTIMATE_COLOR,
                                       self.frame, stamp, **arrow))
        msg.markers.append(text_marker("estimate", n + 1, T_R[:3, 3] + [0.0, 0.0, 0.6],
                                       error_text(e), ESTIMATE_COLOR, self.frame, stamp))
        return msg


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
