#pragma once

#include "camslab/types.hpp"

namespace camslab
{

// Trajectory tracking control law for the reference ref. Same law as camslab_sim/controller.py.
Command control(const State & state, const Reference & ref, const Params & params);

}  // namespace camslab
