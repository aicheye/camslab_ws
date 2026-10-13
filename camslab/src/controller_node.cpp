// Runs camslab::control() at a fixed rate on the latest pose and the reference at the
// time since the controller was enabled, and publishes cmd_vel.
//
// Subscribes: pose (geometry_msgs/PoseStamped)
// Publishes:  cmd_vel (geometry_msgs/Twist),
//             ref_pose (geometry_msgs/PoseStamped): p* and q* now, while driving,
//             ref_path (nav_msgs/Path, transient local): p*(t) for t in
//             [0, t_max], on startup and whenever a trajectory parameter changes.
//             safety_polytope (geometry_msgs/PolygonStamped, transient local): the
//             `polytope` parameter, empty when there is none.
// Services:   ~/enable (std_srvs/SetBool). The node starts disabled unless start_enabled.
//
// The trajectory clock starts at the first tick that is enabled with a fresh pose, and
// restarts on every enable and every trajectory parameter change. A trajectory that
// breaks kappa_max or w_max somewhere in [0, t_max] is accepted with a warning: the node
// logs it and returns it as the reason of the successful parameter change. The clamp
// below still holds every command to the car's limits, so the car lags p* there.
//
// Safety polytope: when the `polytope` parameter lists at least 3 corners (set in the web
// UI, which saves them to config/polytope.yaml), the node also refuses a trajectory whose p*(t) comes closer
// than polytope_margin to an edge over [0, t_max], refuses to enable while the robot is
// outside the polytope, and disables itself when the robot leaves it. An empty `polytope`
// turns these checks off.
//
// Every command is clamped to |v| <= v_max and |w| <= min(w_max, kappa_max |v|), the
// turning limit of the Ackermann car. camslab_sim/simulate.py applies the same clamp.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "camslab/controller.hpp"
#include "camslab/conversions.hpp"
#include "camslab/polytope.hpp"
#include "camslab/trajectory.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace camslab
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller")
  {
    params_.shape = declare_parameter("shape", params_.shape);
    std::vector<double> xy;
    for (const auto & p : params_.control_points) {
      xy.push_back(p.x);
      xy.push_back(p.y);
    }
    try {
      params_.control_points = toPoints(declare_parameter("control_points", xy));
    } catch (const std::invalid_argument & e) {
      RCLCPP_FATAL(get_logger(), "control_points parameter: %s", e.what());
      throw;
    }
    for (const auto & f : kParamFields) {
      params_.*f.field = declare_parameter(f.name, params_.*f.field);
    }
    t_max_ = declare_parameter("t_max", 60.0);
    safety_.margin = declare_parameter("polytope_margin", 0.2);
    // A YAML list of integers such as [0, 0, 6, 0, ...] is an integer array and fails here
    // with a type error; write every number with a decimal point.
    try {
      safety_.polytope = makePolytope(declare_parameter("polytope", std::vector<double>{}));
    } catch (const std::invalid_argument & e) {
      RCLCPP_FATAL(get_logger(), "polytope parameter: %s", e.what());
      throw;
    }
    pose_timeout_ = declare_parameter("pose_timeout", 0.5);
    frame_id_ = declare_parameter("frame_id", "map");
    const bool start_enabled = declare_parameter("start_enabled", false);
    const double rate = declare_parameter("rate", 50.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    ref_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("ref_pose", 10);
    ref_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("ref_path", rclcpp::QoS(1).transient_local());
    polytope_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      "safety_polytope", rclcpp::QoS(1).transient_local());
    // Sensor-data QoS (best effort) also connects to reliable publishers such as ros_gz_bridge.
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "pose", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
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
        response->message = response->success ? "" : enable_error_;
      });

    publishPolytope();
    setTrajectory(params_);
    setEnabled(start_enabled);

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changes) { return onParameters(changes); });

    // Ticks on the node clock, which is Gazebo time under use_sim_time.
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this]() { onTimer(); });

    get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() { cmd_pub_->publish(geometry_msgs::msg::Twist()); });
  }

private:
  // Double parameters that are fields of Params. `trajectory` is true when a change alters
  // the reference or its limit check, so the node rechecks it and restarts the clock.
  struct ParamField
  {
    const char * name;
    double Params::*field;
    bool trajectory;
  };
  static constexpr ParamField kParamFields[] = {
    {"x0", &Params::x0, true},
    {"y0", &Params::y0, true},
    {"theta0", &Params::theta0, true},
    {"circle_speed", &Params::circle_speed, true},
    {"kappa", &Params::kappa, true},
    {"a", &Params::a, true},
    {"b", &Params::b, true},
    {"period", &Params::period, true},
    {"phi0", &Params::phi0, true},
    {"kappa_max", &Params::kappa_max, true},
    {"w_max", &Params::w_max, true},
    {"k_par", &Params::k_par, false},
    {"k_perp", &Params::k_perp, false},
    {"k_q", &Params::k_q, false},
    {"v_max", &Params::v_max, false},
  };

  // Zero commands are sent for this long after driving stops [s].
  static constexpr double kZeroSeconds = 1.0;

  // The safety polytope, absent when the `polytope` parameter is empty.
  struct Safety
  {
    std::optional<Polytope> polytope;
    double margin{0.2};  // [m]
  };

  // Polytope from [x1, y1, x2, y2, ...], or nullopt for an empty list. Throws
  // std::invalid_argument for a list that is not a convex counter-clockwise polygon.
  static std::optional<Polytope> makePolytope(const std::vector<double> & xy)
  {
    if (xy.empty()) {
      return std::nullopt;
    }
    return Polytope(xy);
  }

  // Checks the trajectory of `params` against the safety polytope over [0, t_max].
  // Returns the problem, or an empty string. A trajectory whose functions are not written
  // yet passes, so the node still starts; onTimer() then reports the not-implemented error.
  std::string trajectoryProblem(const Params & params, double t_max, const Safety & safety) const
  {
    try {
      const auto shape = makeShape(params);
      std::optional<std::string> problem;
      if (safety.polytope) {
        problem = checkInside(*shape, *safety.polytope, safety.margin, t_max);
      }
      return problem.value_or("");
    } catch (const std::invalid_argument & e) {  // unknown shape
      return e.what();
    } catch (const std::logic_error &) {  // function not written yet
      return "";
    }
  }

  // The first place in [0, t_max] where the trajectory of `params` breaks kappa_max or
  // w_max, or an empty string. Not a reason to reject it: the car tracks it with an error.
  static std::string limitWarning(const Params & params, double t_max)
  {
    try {
      return checkLimits(*makeShape(params), params, t_max).value_or("");
    } catch (const std::logic_error &) {  // unknown shape or function not written yet
      return "";
    }
  }

  // Adopts `params`, restarts the trajectory clock, and republishes ref_path.
  void setTrajectory(const Params & params)
  {
    params_ = params;
    trajectory_error_ = trajectoryProblem(params_, t_max_, safety_);
    if (const std::string warning = limitWarning(params_, t_max_); !warning.empty()) {
      RCLCPP_WARN(get_logger(), "the car cannot track the whole trajectory: %s", warning.c_str());
    }
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
        const Vec2 p = shape_->position(t_max_ * i / kPoints);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    } catch (const std::logic_error & e) {
      RCLCPP_ERROR(get_logger(), "no ref_path: %s", e.what());
      path.poses.clear();
    }
    ref_path_pub_->publish(path);
  }

  void publishPolytope()
  {
    geometry_msgs::msg::PolygonStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    if (safety_.polytope) {
      for (const Vec2 & v : safety_.polytope->vertices()) {
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float>(v.x);
        point.y = static_cast<float>(v.y);
        msg.polygon.points.push_back(point);
      }
    }
    polytope_pub_->publish(msg);
  }

  // The robot's position when it is outside the safety polytope, otherwise nullopt.
  std::optional<Vec2> outsidePolytope() const
  {
    if (pose_time_ && safety_.polytope && !safety_.polytope->contains(state_.p)) {
      return state_.p;
    }
    return std::nullopt;
  }

  bool setEnabled(bool enabled)
  {
    if (enabled && !trajectory_error_.empty()) {
      enable_error_ = trajectory_error_;
      RCLCPP_ERROR(get_logger(), "not enabled: %s", enable_error_.c_str());
      return false;
    }
    if (const auto p = outsidePolytope(); enabled && p) {
      char message[120];
      std::snprintf(
        message, sizeof(message), "robot at (%.2f, %.2f) m is outside the safety polytope", p->x,
        p->y);
      enable_error_ = message;
      RCLCPP_ERROR(get_logger(), "not enabled: %s", enable_error_.c_str());
      return false;
    }
    enabled_ = enabled;
    t0_.reset();
    RCLCPP_INFO(get_logger(), "%s", enabled_ ? "enabled" : "disabled");
    return true;
  }

  void onTimer()
  {
    const bool pose_fresh = pose_time_ && (now() - *pose_time_).seconds() < pose_timeout_;
    if (enabled_ && pose_time_ && !pose_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "no pose for %.1f s, commanding zero", pose_timeout_);
    }

    if (const auto p = outsidePolytope(); enabled_ && pose_fresh && p) {
      RCLCPP_ERROR(
        get_logger(), "robot at (%.2f, %.2f) m left the safety polytope, disabling", p->x, p->y);
      setEnabled(false);
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
        ref_pose_pub_->publish(msg);
        cmd = control(state_, ref, params_);
      } catch (const std::logic_error & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", e.what());
      }
    }

    geometry_msgs::msg::Twist twist;
    if (cmd && std::isfinite(cmd->v) && std::isfinite(cmd->w)) {
      const double v = std::clamp(cmd->v, -params_.v_max, params_.v_max);
      const double w_limit = std::min(params_.w_max, params_.kappa_max * std::abs(v));
      twist.linear.x = v;
      twist.angular.z = std::clamp(cmd->w, -w_limit, w_limit);
      last_drive_ = now();
      cmd_pub_->publish(twist);
    } else if (last_drive_ && (now() - *last_drive_).seconds() < kZeroSeconds) {
      // Zero is sent for kZeroSeconds after driving stops, so that another cmd_vel
      // source can drive while this node is idle.
      cmd_pub_->publish(twist);
    }
  }

  // Adopts a set of changes only if the trajectory they give is inside the limits.
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & changes)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    Params params = params_;
    double t_max = t_max_;
    double pose_timeout = pose_timeout_;
    Safety safety = safety_;
    bool trajectory_changed = false;
    bool polytope_changed = false;
    for (const auto & change : changes) {
      const auto & name = change.get_name();
      if (
        name == "polytope" &&
        change.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        try {
          safety.polytope = makePolytope(change.as_double_array());
        } catch (const std::invalid_argument & e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
        trajectory_changed = true;
        polytope_changed = true;
      }
      if (name == "shape" && change.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        params.shape = change.as_string();
        trajectory_changed = true;
      }
      if (
        name == "control_points" &&
        change.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        try {
          params.control_points = toPoints(change.as_double_array());
        } catch (const std::invalid_argument & e) {
          result.successful = false;
          result.reason = e.what();
          return result;
        }
        trajectory_changed = true;
      }
      if (change.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        continue;
      }
      const double value = change.as_double();
      if (name == "t_max") {
        t_max = value;
        trajectory_changed = true;
      } else if (name == "pose_timeout") {
        pose_timeout = value;
      } else if (name == "polytope_margin") {
        safety.margin = value;
        trajectory_changed = true;
      }
      for (const auto & f : kParamFields) {
        if (name == f.name) {
          params.*f.field = value;
          trajectory_changed = trajectory_changed || f.trajectory;
        }
      }
    }
    if (trajectory_changed) {
      const std::string problem = trajectoryProblem(params, t_max, safety);
      if (!problem.empty()) {
        result.successful = false;
        result.reason = problem;
        return result;
      }
      result.reason = limitWarning(params, t_max);
    }
    t_max_ = t_max;
    pose_timeout_ = pose_timeout;
    safety_ = safety;
    if (polytope_changed) {
      publishPolytope();
    }
    if (trajectory_changed) {
      setTrajectory(params);
    } else {
      params_ = params;
    }
    return result;
  }

  Params params_;
  std::unique_ptr<Shape> shape_;
  std::string trajectory_error_;
  std::string enable_error_;  // why the last enable failed
  Safety safety_;
  double t_max_;
  double pose_timeout_;
  std::string frame_id_;

  bool enabled_{false};
  State state_;
  std::optional<rclcpp::Time> pose_time_;
  std::optional<rclcpp::Time> t0_;  // trajectory time 0
  std::optional<rclcpp::Time> last_drive_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr polytope_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

}  // namespace camslab

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camslab::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
