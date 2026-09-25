/* Reference trajectories p*(t). Same shapes as camslab_sim/trajectory.py.

Each shape gives p*(t) and its first two time derivatives, differentiated by
hand. reference() turns them into p*, q*, v*, w* and kappa*.
*/

#include "camslab/trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace camslab
{

Circle::Circle(const Params & params)
: p0_{params.x0, params.y0}, theta0_{params.theta0}, v_{params.circle_speed}, kappa_{params.kappa}
{
}

Vec2 Circle::heading(double t) const
{
  double a{std::cos(theta0_)};
  double b{std::sin(theta0_)};
  double c{std::cos(kappa_ * v_ * t)};
  double d{std::sin(kappa_ * v_ * t)};
  return {a * c - b * d, b * c + a * d};
}

Vec2 Circle::position(double t) const
{
  if (std::abs(kappa_) < 1e-9) return p0_ + velocity(t) * t;

  Vec2 C{p0_.x - std::sin(theta0_) / kappa_, p0_.y + std::cos(theta0_) / kappa_};
  Mat2 S{0, 1, -1, 0};
  return (1 / kappa_) * Mat2::dot(S, heading(t)) + C;
}

Vec2 Circle::velocity(double t) const { return v_ * heading(t); }

Vec2 Circle::acceleration(double t) const
{
  Mat2 R{0, -1, 1, 0};
  return kappa_ * v_ * v_ * Mat2::dot(R, heading(t));
}

Gerono::Gerono(const Params & params)
: center_{params.x0, params.y0}, theta0_{params.theta0}, a_{params.a}, period_{params.period}
{
}

Vec2 Gerono::position(double /*t*/) const
{
  throw std::logic_error("Gerono::position() in trajectory.cpp is not implemented");
}

Vec2 Gerono::velocity(double /*t*/) const
{
  throw std::logic_error("Gerono::velocity() in trajectory.cpp is not implemented");
}

Vec2 Gerono::acceleration(double /*t*/) const
{
  throw std::logic_error("Gerono::acceleration() in trajectory.cpp is not implemented");
}

// Return p*, q*, v*, w* and kappa* at time t.
Reference reference(const Shape & shape, double t)
{
  double speed{shape.velocity(t).norm()};
  Mat2 S{0, -1, 1, 0};
  Vec2 heading{shape.velocity(t) / speed};
  double angular{Vec2::dot(shape.acceleration(t), Mat2::dot(S, heading)) / speed};

  return {shape.position(t), heading, speed, angular, angular / speed};
}

std::unique_ptr<Shape> makeShape(const Params & params)
{
  if (params.shape == "circle") {
    return std::make_unique<Circle>(params);
  }
  if (params.shape == "gerono") {
    return std::make_unique<Gerono>(params);
  }
  throw std::invalid_argument("unknown shape '" + params.shape + "', expected circle or gerono");
}

std::optional<std::string> checkLimits(const Shape & shape, const Params & params, double t_end)
{
  constexpr int kSamples = 2000;
  double kappa_worst{0.0};
  double t_kappa{0.0};
  double w_worst{0.0};
  double t_w{0.0};
  for (int i = 0; i <= kSamples; ++i) {
    const double t = t_end * i / kSamples;
    const Reference ref = reference(shape, t);
    if (std::abs(ref.kappa) > kappa_worst) {
      kappa_worst = std::abs(ref.kappa);
      t_kappa = t;
    }
    if (std::abs(ref.w) > w_worst) {
      w_worst = std::abs(ref.w);
      t_w = t;
    }
  }
  char message[160];
  if (kappa_worst > params.kappa_max) {
    std::snprintf(
      message,
      sizeof(message),
      "|kappa*| = %.3f 1/m at t = %.2f s is above kappa_max = %.3f 1/m",
      kappa_worst,
      t_kappa,
      params.kappa_max);
    return std::string(message);
  }
  if (w_worst > params.w_max) {
    std::snprintf(
      message,
      sizeof(message),
      "|w*| = %.3f rad/s at t = %.2f s is above w_max = %.3f rad/s",
      w_worst,
      t_w,
      params.w_max);
    return std::string(message);
  }
  return std::nullopt;
}

}  // namespace camslab
