/* Reference trajectories p*(t). Same shapes as gtg_sim/trajectory.py.

Each shape gives p*(t) and its first two time derivatives, differentiated by hand.
reference() turns them into p*, q*, v*, w* and kappa*.
*/

#include "gtg_ros/trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace gtg_ros {

Circle::Circle(const Params &params)
    : p0_{params.x0, params.y0}, theta0_{params.theta0}, v_{params.circle_speed},
      kappa_{params.kappa} {}

Vec2 Circle::position(double /*t*/) const {
  throw std::logic_error("Circle::position() in trajectory.cpp is not implemented");
}

Vec2 Circle::velocity(double /*t*/) const {
  throw std::logic_error("Circle::velocity() in trajectory.cpp is not implemented");
}

Vec2 Circle::acceleration(double /*t*/) const {
  throw std::logic_error("Circle::acceleration() in trajectory.cpp is not implemented");
}

Gerono::Gerono(const Params &params)
    : center_{params.x0, params.y0}, theta0_{params.theta0}, a_{params.a},
      period_{params.period} {}

Vec2 Gerono::position(double /*t*/) const {
  throw std::logic_error("Gerono::position() in trajectory.cpp is not implemented");
}

Vec2 Gerono::velocity(double /*t*/) const {
  throw std::logic_error("Gerono::velocity() in trajectory.cpp is not implemented");
}

Vec2 Gerono::acceleration(double /*t*/) const {
  throw std::logic_error("Gerono::acceleration() in trajectory.cpp is not implemented");
}

// Return p*, q*, v*, w* and kappa* at time t.
Reference reference(const Shape & /*shape*/, double /*t*/) {
  throw std::logic_error("reference() in trajectory.cpp is not implemented");
}

std::unique_ptr<Shape> makeShape(const Params &params) {
  if (params.shape == "circle") {
    return std::make_unique<Circle>(params);
  }
  if (params.shape == "gerono") {
    return std::make_unique<Gerono>(params);
  }
  throw std::invalid_argument("unknown shape '" + params.shape +
                              "', expected circle or gerono");
}

std::optional<std::string> checkLimits(const Shape &shape, const Params &params,
                                       double t_end) {
  constexpr int kSamples = 2000;
  char message[160];
  for (int i = 0; i <= kSamples; ++i) {
    const double t = t_end * i / kSamples;
    const Reference ref = reference(shape, t);
    if (std::abs(ref.kappa) > params.kappa_max) {
      std::snprintf(message, sizeof(message),
                    "|kappa*| = %.3f 1/m at t = %.2f s is above kappa_max = %.3f 1/m",
                    std::abs(ref.kappa), t, params.kappa_max);
      return std::string(message);
    }
    if (std::abs(ref.w) > params.w_max) {
      std::snprintf(message, sizeof(message),
                    "|w*| = %.3f rad/s at t = %.2f s is above w_max = %.3f rad/s",
                    std::abs(ref.w), t, params.w_max);
      return std::string(message);
    }
  }
  return std::nullopt;
}

} // namespace gtg_ros
