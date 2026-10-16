"""Broadcasts the robot pose as the TF transform parent_frame -> child_frame."""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class PoseTfNode(Node):
    def __init__(self):
        super().__init__("pose_tf")
        self.parent_frame = self.declare_parameter("parent_frame", "map").value
        self.child_frame = self.declare_parameter("child_frame", "base_footprint").value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, "pose", self.on_pose, qos_profile_sensor_data)

    def on_pose(self, msg):
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.parent_frame
        tf.child_frame_id = self.child_frame
        tf.transform.translation.x = msg.pose.position.x
        tf.transform.translation.y = msg.pose.position.y
        tf.transform.translation.z = msg.pose.position.z
        tf.transform.rotation = msg.pose.orientation
        self.broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTfNode()
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
