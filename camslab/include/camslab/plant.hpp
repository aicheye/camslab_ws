#pragma once

#include "camslab/types.hpp"

namespace camslab
{

// State dt seconds later with cmd held constant over dt. Same model as camslab_sim/plant.py.
State step(const State & state, const Command & cmd, double dt);

}  // namespace camslab
