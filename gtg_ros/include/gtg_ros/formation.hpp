#pragma once

#include "gtg_ros/types.hpp"

namespace gtg_ros
{

// Goal p* for a robot that follows `leader`. Each controller_node publishes this for its
// own pose on follow_goal, and a follower's controller_node uses it as its goal.
Vec2 followerGoal(const State & leader, const Params & params);

}  // namespace gtg_ros
