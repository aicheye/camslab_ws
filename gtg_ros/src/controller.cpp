/*Trajectory tracking control law.

With the reference p*, q*, v*, w* from reference() and e = p* - p:

    v = < v* q* + k_par e, q >
    w = w* + < k_perp v* e + k_q q*, S q >

<a, b> is the dot product, and S = [[0, -1], [1, 0]] rotates a vector by +90 deg.
k_par, k_perp, k_q > 0.
*/

#include "gtg_ros/controller.hpp"

#include <cmath>
#include <stdexcept>

namespace gtg_ros {

// Return the command (v [m/s], w [rad/s]) that tracks ref.
Command control(const State & /*state*/, const Reference & /*ref*/,
                const Params & /*params*/) {
  throw std::logic_error("control() in controller.cpp is not implemented");
}

} // namespace gtg_ros
