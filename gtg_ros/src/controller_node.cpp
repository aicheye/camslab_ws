// Runs gtg_ros::control() at a fixed rate on the latest pose and the reference at the
// time since the controller was enabled, and publishes cmd_vel.
//
// Subscribes: pose (geometry_msgs/PoseStamped)
// Publishes:  cmd_vel (geometry_msgs/Twist),
//             reference (geometry_msgs/PoseStamped): p* and q* now, while driving,
//             reference_path (nav_msgs/Path, transient local): p*(t) for t in
//             [0, path_time], on startup and whenever a trajectory parameter changes.
// Services:   ~/enable (std_srvs/SetBool). The node starts disabled unless start_enabled.
//
// The trajectory clock starts at the first tick that is enabled with a fresh pose, and
// restarts on every enable and every trajectory parameter change. The node will not
// enable while the trajectory breaks kappa_max or w_max over [0, path_time], and it
// rejects parameter changes that would break them.
//
// Every command is clamped to |v| <= v_max and |w| <= min(w_max, kappa_max |v|), the
// turning limit of the Ackermann car.

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
#include "gtg_ros/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
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
    params_.shape = declare_parameter("shape", params_.shape);
    params_.x0 = declare_parameter("x0", params_.x0);
    params_.y0 = declare_parameter("y0", params_.y0);
    params_.theta0 = declare_parameter("theta0", params_.theta0);
    params_.circle_speed = declare_parameter("circle_speed", params_.circle_speed);
    params_.kappa = declare_parameter("kappa", params_.kappa);
    params_.a = declare_parameter("a", params_.a);
    params_.period = declare_parameter("period", params_.period);
    params_.k_par = declare_parameter("k_par", params_.k_par);
    params_.k_perp = declare_parameter("k_perp", params_.k_perp);
    params_.k_q = declare_parameter("k_q", params_.k_q);
    params_.kappa_max = declare_parameter("kappa_max", params_.kappa_max);
    params_.w_max = declare_parameter("w_max", params_.w_max);
    path_time_ = declare_parameter("path_time", 60.0);
    frame_id_ = declare_parameter("frame_id", "map");
    v_max_ = declare_parameter("v_max", 0.6);
    pose_timeout_ = declare_parameter("pose_timeout", 0.5);
    const bool start_enabled = declare_parameter("start_enabled", false);
    const double rate = declare_parameter("rate", 50.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    reference_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("reference", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "reference_path", rclcpp::QoS(1).transient_local());
    // Sensor-data QoS (best effort) also connects to reliable publishers such as ros_gz_bridge.
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (const auto state = stateFromPose(msg->pose)) {
          state_ = *state;
          pose_time_ = now();
        }
      });
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable",
      [this](
        std_srvs::srv::SetBool::Request::ConstSharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
        response->success = setEnabled(request->data);
        response->message = response->success ? "" : trajectory_error_;
      });

    setTrajectory(params_);
    setEnabled(start_enabled);

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) {return onParameters(changes);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {onTimer();});

    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {cmd_pub_->publish(geometry_msgs::msg::Twist());});
  }

private:
  // Checks the trajectory of `params` against the limits. Returns the problem, or an
  // empty string. A trajectory whose functions are not written yet passes, so the node
  // still starts; onTimer() then reports the not-implemented error.
  std::string trajectoryProblem(const Params & params) const
  {
    try {
      const auto problem = checkLimits(*makeShape(params), params, path_time_);
      return problem.value_or("");
    } catch (const std::logic_error & e) {
      if (dynamic_cast<const std::invalid_argument *>(&e)) {
        return e.what();
      }
      return "";
    }
  }

  // Adopts `params`, restarts the trajectory clock, and republishes reference_path.
  void setTrajectory(const Params & params)
  {
    params_ = params;
    trajectory_error_ = trajectoryProblem(params_);
    t0_.reset();
    if (!trajectory_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "trajectory rejected: %s", trajectory_error_.c_str());
      if (enabled_) {
        setEnabled(false);
      }
      return;
    }
    shape_ = makeShape(params_);
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    try {
      constexpr int kPoints = 600;
      for (int i = 0; i <= kPoints; ++i) {
        const Vec2 p = shape_->position(path_time_ * i / kPoints);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    } catch (const std::logic_error & e) {
      RCLCPP_ERROR(get_logger(), "no reference_path: %s", e.what());
      path.poses.clear();
    }
    path_pub_->publish(path);
  }

  bool setEnabled(bool enabled)
  {
    if (enabled && !trajectory_error_.empty()) {
      RCLCPP_ERROR(get_logger(), "not enabled: %s", trajectory_error_.c_str());
      return false;
    }
    enabled_ = enabled;
    t0_.reset();
    RCLCPP_INFO(get_logger(), "%s", enabled_ ? "enabled" : "disabled");
    return true;
  }

  void onTimer()
  {
    const bool pose_fresh =
      pose_time_ && (now() - *pose_time_).seconds() < pose_timeout_;
    if (enabled_ && pose_time_ && !pose_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "no pose for %.1f s, commanding zero", pose_timeout_);
    }

    std::optional<Command> cmd;
    if (enabled_ && pose_fresh && shape_) {
      if (!t0_) {
        t0_ = now();
      }
      try {
        const Reference ref = reference(*shape_, (now() - *t0_).seconds());
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.pose = poseFromState(State{ref.p, ref.q});
        reference_pub_->publish(msg);
        cmd = control(state_, ref, params_);
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
      }
    }

    geometry_msgs::msg::Twist twist;
    if (cmd && std::isfinite(cmd->v) && std::isfinite(cmd->w)) {
      const double v = std::clamp(cmd->v, -v_max_, v_max_);
      const double w_limit = std::min(params_.w_max, params_.kappa_max * std::abs(v));
      twist.linear.x = v;
      twist.angular.z = std::clamp(cmd->w, -w_limit, w_limit);
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
    Params params = params_;
    double path_time = path_time_;
    bool trajectory_changed = false;
    for (const auto & change : changes) {
      const auto & name = change.get_name();
      if (name == "shape" && change.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        params.shape = change.as_string();
        trajectory_changed = true;
        continue;
      }
      if (change.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        continue;
      }
      const double value = change.as_double();
      // Trajectory and limit parameters: checked below before they are adopted.
      double * field = name == "x0" ? &params.x0 :
        name == "y0" ? &params.y0 :
        name == "theta0" ? &params.theta0 :
        name == "circle_speed" ? &params.circle_speed :
        name == "kappa" ? &params.kappa :
        name == "a" ? &params.a :
        name == "period" ? &params.period :
        name == "kappa_max" ? &params.kappa_max :
        name == "w_max" ? &params.w_max :
        name == "path_time" ? &path_time : nullptr;
      if (field) {
        *field = value;
        trajectory_changed = true;
      } else if (name == "k_par") {
        params.k_par = value;
      } else if (name == "k_perp") {
        params.k_perp = value;
      } else if (name == "k_q") {
        params.k_q = value;
      } else if (name == "v_max") {
        v_max_ = value;
      } else if (name == "pose_timeout") {
        pose_timeout_ = value;
      }
    }
    if (!trajectory_changed) {
      params_ = params;
      return result;
    }
    const double old_path_time = path_time_;
    path_time_ = path_time;
    const std::string problem = trajectoryProblem(params);
    if (!problem.empty()) {
      path_time_ = old_path_time;
      result.successful = false;
      result.reason = problem;
      return result;
    }
    setTrajectory(params);
    return result;
  }

  static constexpr int kZeroTicks = 50;

  Params params_;
  std::unique_ptr<Shape> shape_;
  std::string trajectory_error_;
  double path_time_;
  double v_max_;
  double pose_timeout_;
  std::string frame_id_;

  bool enabled_{false};
  State state_;
  std::optional<rclcpp::Time> pose_time_;
  std::optional<rclcpp::Time> t0_;  // trajectory time 0
  int zero_ticks_left_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
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
