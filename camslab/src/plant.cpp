/* Unicycle plant.

    dp/dt = v q
    dq/dt = w S q,    S = [[0, -1], [1, 0]]
*/

#include "camslab/plant.hpp"

#include <cmath>

#include "camslab/types.hpp"

namespace camslab
{

// Return the state dt seconds later, with v and w held constant over dt.
State step(const State & state, const Command & cmd, double dt)
{
  double a{std::cos(cmd.w * dt)};
  double b{std::sin(cmd.w * dt)};
  Mat2 R{a, -b, b, a};

  Vec2 p;
  if (std::abs(cmd.w) > 1e-9) {
    Mat2 M{b, a - 1, -a + 1, b};
    p = state.p + (cmd.v / cmd.w) * Mat2::dot(M, state.q);
  } else {
    p = state.p + cmd.v * dt * state.q;
  }

  Vec2 q = Mat2::dot(R, state.q);

  return {p, q};
}

}  // namespace camslab
