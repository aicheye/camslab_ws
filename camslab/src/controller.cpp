/* Trajectory tracking control law.

With the reference p*, q*, v*, w* from reference() and e = p* - p:

    v = < v* q* + k_par e, q >
    w = w* + < k_perp v* e + k_q q*, S q >

<a, b> is the dot product, and S = [[0, -1], [1, 0]] rotates a vector by +90
deg. k_par, k_perp, k_q > 0.
*/

#include "camslab/controller.hpp"

#include <cmath>
#include <stdexcept>

namespace camslab
{

// Return the command (v [m/s], w [rad/s]) that tracks ref.
Command control(const State & state, const Reference & ref, const Params & params)
{
  Vec2 e{ref.p - state.p};
  Mat2 S{0, -1, 1, 0};

  double v{Vec2::dot(ref.v * ref.q + params.k_par * e, state.q)};
  double w{
    ref.w + Vec2::dot(params.k_perp * ref.v * e + params.k_q * ref.q, Mat2::dot(S, state.q))};

  return {v, w};
}

}  // namespace camslab
