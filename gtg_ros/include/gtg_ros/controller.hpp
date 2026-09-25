#pragma once

#include "gtg_ros/types.hpp"

namespace gtg_ros
{

// Trajectory tracking control law for the reference ref. Same law as gtg_sim/controller.py.
Command control(const State & state, const Reference & ref, const Params & params);

}  // namespace gtg_ros
