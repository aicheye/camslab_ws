"""Shows the fiducial observations in RViz: for every transform on `observations` (the
car node's marker poses in the camera frame) a flat square of the marker's size and a
label with its id and distance, as a MarkerArray on `observation_markers`. The markers
expire after `lifetime` seconds, so a fiducial that leaves the image disappears. Once a
second it logs what is seen, in the camera frame: x forward, y left, z up."""

import colorsys
import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

LOG_PERIOD = 1.0


def color_of(marker_id: int) -> ColorRGBA:
    r, g, b = colorsys.hsv_to_rgb((0.13 + 0.17 * marker_id) % 1.0, 0.75, 0.95)
    return ColorRGBA(r=r, g=g, b=b, a=0.9)


class ObservationMarkers(Node):
    def __init__(self):
        super().__init__("observation_markers")
        self.size = float(self.declare_parameter("marker_size", 0.14).value)
        self.prefix = str(self.declare_parameter("marker_prefix", "marker_").value)
        self.lifetime = float(self.declare_parameter("lifetime", 0.5).value)
        self.pub = self.create_publisher(MarkerArray, "observation_markers", 10)
        self.create_subscription(TFMessage, "observations", self.on_observations, 10)
        self.seen = {}                  # id -> (x, y, z) of the last second
        self.frames = 0
        self.create_timer(LOG_PERIOD, self.log)

    def on_observations(self, msg):
        self.frames += 1
        out = MarkerArray()
        lifetime = Duration(sec=int(self.lifetime), nanosec=int((self.lifetime % 1.0) * 1e9))
        for tf in msg.transforms:
            name = tf.child_frame_id
            if not name.startswith(self.prefix):
                continue
            try:
                marker_id = int(name[len(self.prefix):])
            except ValueError:
                continue
            t = tf.transform.translation
            self.seen[marker_id] = (t.x, t.y, t.z)
            square = Marker()
            square.header = tf.header
            square.ns, square.id = "fiducial", marker_id
            square.type, square.action = Marker.CUBE, Marker.ADD
            square.pose.position.x, square.pose.position.y, square.pose.position.z = t.x, t.y, t.z
            square.pose.orientation = tf.transform.rotation
            square.scale.x = square.scale.y = self.size
            square.scale.z = 0.004
            square.color = color_of(marker_id)
            square.lifetime = lifetime
            label = Marker()
            label.header = tf.header
            label.ns, label.id = "label", marker_id
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y = t.x, t.y
            label.pose.position.z = t.z + 0.6 * self.size + 0.05
            label.pose.orientation.w = 1.0
            label.scale.z = 0.08
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f"{marker_id}  {math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2):.2f} m"
            label.lifetime = lifetime
            out.markers += [square, label]
        if out.markers:
            self.pub.publish(out)

    def log(self):
        if not self.frames:
            self.get_logger().info("no observations: is the camera node running?")
            return
        if not self.seen:
            self.get_logger().info(f"{self.frames} frames, no marker in view")
        else:
            seen = "   ".join(f"{i}: x {p[0]:.2f} y {p[1]:.2f} z {p[2]:.2f}"
                              for i, p in sorted(self.seen.items()))
            self.get_logger().info(f"{self.frames} frames   {seen}")
        self.seen, self.frames = {}, 0


def main(args=None):
    rclpy.init(args=args)
    node = ObservationMarkers()
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
