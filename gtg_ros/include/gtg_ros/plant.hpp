#pragma once

#include "gtg_ros/types.hpp"

namespace gtg_ros
{

// State dt seconds later with cmd held constant over dt. Same model as gtg_sim/plant.py.
State step(const State & state, const Command & cmd, double dt);

}  // namespace gtg_ros
