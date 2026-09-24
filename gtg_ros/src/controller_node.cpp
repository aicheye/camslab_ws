// Runs gtg_ros::control() at a fixed rate on the latest pose and publishes cmd_vel.
//
// Subscribes: pose (geometry_msgs/PoseStamped),
//             goal (geometry_msgs/PoseStamped, the type of RViz's 2D Goal Pose; heading ignored)
// Publishes:  cmd_vel (geometry_msgs/Twist),
//             follow_goal (geometry_msgs/PoseStamped): followerGoal() of this robot's
//             pose, the goal for a robot that follows this one. Published every tick
//             while the pose is fresh, also while disabled.
//
// A follower is this node with its goal remapped to the leader's follow_goal.
// Services:   ~/enable (std_srvs/SetBool). The node starts disabled unless start_enabled.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gtg_ros/controller.hpp"
#include "gtg_ros/conversions.hpp"
#include "gtg_ros/formation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace gtg_ros
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node")
  {
    params_.v_star = declare_parameter("v_star", params_.v_star);
    params_.d_switch = declare_parameter("d_switch", params_.d_switch);
    params_.k_v = declare_parameter("k_v", params_.k_v);
    params_.k_w = declare_parameter("k_w", params_.k_w);
    params_.epsilon = declare_parameter("epsilon", params_.epsilon);
    params_.spacing = declare_parameter("spacing", params_.spacing);
    frame_id_ = declare_parameter("frame_id", "map");
    v_max_ = declare_parameter("v_max", 0.6);
    w_max_ = declare_parameter("w_max", 2.0);
    pose_timeout_ = declare_parameter("pose_timeout", 0.5);
    enabled_ = declare_parameter("start_enabled", false);
    const double rate = declare_parameter("rate", 50.0);

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) {return onParameters(changes);});

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    follow_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("follow_goal", 10);
    // Sensor-data QoS (best effort) also connects to reliable publishers such as ros_gz_bridge.
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (const auto state = stateFromPose(msg->pose)) {
          state_ = *state;
          pose_time_ = now();
        }
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        goal_ = Vec2{msg->pose.position.x, msg->pose.position.y};
      });
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      [this](
        std_srvs::srv::SetBool::Request::ConstSharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        enabled_ = request->data;
        response->success = true;
        RCLCPP_INFO(get_logger(), "%s", enabled_ ? "enabled" : "disabled");
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {onTimer();});

    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {cmd_pub_->publish(geometry_msgs::msg::Twist());});
  }

private:
  void onTimer()
  {
    const bool pose_fresh =
      pose_time_ && (now() - *pose_time_).seconds() < pose_timeout_;
    if (enabled_ && pose_time_ && !pose_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "no pose for %.1f s, commanding zero", pose_timeout_);
    }

    std::optional<Command> cmd;
    try {
      if (pose_fresh) {
        // Heading of this robot, so the pose shows which way the chain points.
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.pose = poseFromState(State{followerGoal(state_, params_), state_.q});
        follow_goal_pub_->publish(msg);
      }
      if (enabled_ && goal_ && pose_fresh) {
        cmd = control(state_, *goal_, params_);
      }
    } catch (const std::logic_error & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
    }

    geometry_msgs::msg::Twist twist;
    if (cmd && std::isfinite(cmd->v) && std::isfinite(cmd->w)) {
      twist.linear.x = std::clamp(cmd->v, -v_max_, v_max_);
      twist.angular.z = std::clamp(cmd->w, -w_max_, w_max_);
      zero_ticks_left_ = kZeroTicks;
      cmd_pub_->publish(twist);
    } else if (zero_ticks_left_ > 0) {
      // Zero is sent for a limited number of ticks after driving stops, so that
      // another cmd_vel source can drive while this node is idle.
      --zero_ticks_left_;
      cmd_pub_->publish(twist);
    }
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & changes)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & change : changes) {
      if (change.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        continue;
      }
      const double value = change.as_double();
      const auto & name = change.get_name();
      if (name == "v_star") {
        params_.v_star = value;
      } else if (name == "d_switch") {
        params_.d_switch = value;
      } else if (name == "k_v") {
        params_.k_v = value;
      } else if (name == "k_w") {
        params_.k_w = value;
      } else if (name == "epsilon") {
        params_.epsilon = value;
      } else if (name == "spacing") {
        params_.spacing = value;
      } else if (name == "v_max") {
        v_max_ = value;
      } else if (name == "w_max") {
        w_max_ = value;
      } else if (name == "pose_timeout") {
        pose_timeout_ = value;
      }
    }
    return result;
  }

  static constexpr int kZeroTicks = 50;

  Params params_;
  double v_max_;
  double w_max_;
  double pose_timeout_;
  std::string frame_id_;

  bool enabled_{false};
  State state_;
  std::optional<rclcpp::Time> pose_time_;
  std::optional<Vec2> goal_;
  int zero_ticks_left_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr follow_goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace gtg_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gtg_ros::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
