#pragma once

#include <cmath>
#include <optional>

#include "camslab/types.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace camslab
{

// Planar state from a 3D pose. q is the body x axis projected on the global xy plane.
// Returns nullopt when the body x axis is vertical.
inline std::optional<State> stateFromPose(const geometry_msgs::msg::Pose & pose)
{
  const auto & o = pose.orientation;
  const double hx = 1.0 - 2.0 * (o.y * o.y + o.z * o.z);
  const double hy = 2.0 * (o.x * o.y + o.w * o.z);
  const double norm = std::hypot(hx, hy);
  if (norm < 1e-6) {
    return std::nullopt;
  }
  State state;
  state.p = {pose.position.x, pose.position.y};
  state.q = {hx / norm, hy / norm};
  return state;
}

inline geometry_msgs::msg::Pose poseFromState(const State & state)
{
  const double half_yaw = 0.5 * std::atan2(state.q.y, state.q.x);
  geometry_msgs::msg::Pose pose;
  pose.position.x = state.p.x;
  pose.position.y = state.p.y;
  pose.orientation.z = std::sin(half_yaw);
  pose.orientation.w = std::cos(half_yaw);
  return pose;
}

}  // namespace camslab
