/*Go-to-goal control law.

With a = p* - p:

    v = v*              if ||a|| > D
        Kv ||a||        otherwise
    w = Kw atan2(sin(e), cos(e))
    e = (bearing of a) - (heading of q)
*/

#include "gtg_ros/controller.hpp"

#include <cmath>

namespace gtg_ros {

// Return the command (v [m/s], w [rad/s]) for the goal p* = goal.
Command control(const State &state, const Vec2 &goal, const Params &params) {
  Vec2 a = goal - state.p;
  float d = a.norm();
  double e = std::atan2(a.y, a.x) - std::atan2(state.q.y, state.q.x);

  double v = d > params.d_switch ? params.v_star : params.k_v * d;
  double w = params.k_w * std::atan2(std::sin(e), std::cos(e));

  return {v, w};
}

} // namespace gtg_ros
