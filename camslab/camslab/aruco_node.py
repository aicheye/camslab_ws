"""Detects ArUco markers in a camera image and publishes their poses in the camera frame,
the observations C_T_Mj of the fiducial estimator, as a tf2_msgs/TFMessage on
`observations` (one transform per visible marker, stamped with the image time) and on
/tf for RViz. Subscribes `image_raw` and `camera_info` of the usb_cam node."""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

from .aruco import Detector
from .estimator_node import quaternion_of


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco")
        marker_size = self.declare_parameter("marker_size", 0.14).value
        dictionary = self.declare_parameter("dictionary", "DICT_4X4_50").value
        self.camera_frame = self.declare_parameter("camera_frame", "camera").value
        self.prefix = self.declare_parameter("marker_prefix", "marker_").value
        self.debug = bool(self.declare_parameter("debug_image", False).value)
        self.detector = Detector(dictionary, marker_size)
        from cv_bridge import CvBridge

        self.bridge = CvBridge()
        self.broadcaster = TransformBroadcaster(self)
        self.observations_pub = self.create_publisher(TFMessage, "observations", 10)
        self.debug_pub = self.create_publisher(Image, "image_markers", 1) if self.debug else None
        self.create_subscription(CameraInfo, "camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, "image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"{dictionary}, markers of {marker_size} m")

    def on_info(self, msg):
        self.detector.set_intrinsics(msg.k, msg.d)

    def on_image(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        found = self.detector.detect(image)
        tfs = []
        for marker_id, T in found.items():
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self.camera_frame
            tf.child_frame_id = f"{self.prefix}{marker_id}"
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
                float(v) for v in T[:3, 3])
            qx, qy, qz, qw = quaternion_of(T[:3, :3])
            tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            tfs.append(tf)
        self.observations_pub.publish(TFMessage(transforms=tfs))
        if tfs:
            self.broadcaster.sendTransform(tfs)
        if self.debug_pub is not None:
            cv2 = self.detector.cv2
            corners, ids, _ = self.detector._detect(image)
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
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
