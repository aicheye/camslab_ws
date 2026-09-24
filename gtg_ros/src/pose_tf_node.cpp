// Broadcasts the robot pose as a transform, so TF has <parent_frame> -> <child_frame>.
//
// Subscribes: pose (geometry_msgs/PoseStamped)
// Publishes:  /tf  parent_frame -> child_frame
//
// The transform is stamped with this node's clock, not the pose's stamp, because
// Gazebo stamps poses with simulation time and RViz runs on wall time.

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace gtg_ros
{

class PoseTfNode : public rclcpp::Node
{
public:
  PoseTfNode()
  : Node("pose_tf_node")
  {
    parent_frame_ = declare_parameter("parent_frame", "map");
    child_frame_ = declare_parameter("child_frame", "base_footprint");
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now();
        tf.header.frame_id = parent_frame_;
        tf.child_frame_id = child_frame_;
        tf.transform.translation.x = msg->pose.position.x;
        tf.transform.translation.y = msg->pose.position.y;
        tf.transform.translation.z = msg->pose.position.z;
        tf.transform.rotation = msg->pose.orientation;
        broadcaster_->sendTransform(tf);
      });
  }

private:
  std::string parent_frame_;
  std::string child_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace gtg_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gtg_ros::PoseTfNode>());
  rclcpp::shutdown();
  return 0;
}
