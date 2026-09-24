#pragma once

#include "gtg_ros/types.hpp"

namespace gtg_ros
{

// Go-to-goal control law for the goal p* = goal. Same law as gtg_sim/controller.py.
Command control(const State & state, const Vec2 & goal, const Params & params);

}  // namespace gtg_ros
