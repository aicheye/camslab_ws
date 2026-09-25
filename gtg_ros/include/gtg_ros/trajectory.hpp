#pragma once

#include <memory>
#include <optional>
#include <string>

#include "gtg_ros/types.hpp"

namespace gtg_ros
{

// A reference trajectory p*(t) and its first two time derivatives. Same shapes as
// gtg_sim/trajectory.py.
class Shape
{
public:
  virtual ~Shape() = default;
  virtual Vec2 position(double t) const = 0;      // p*(t) [m]
  virtual Vec2 velocity(double t) const = 0;      // dp*/dt [m/s]
  virtual Vec2 acceleration(double t) const = 0;  // d^2 p*/dt^2 [m/s^2]
};

// Circle of curvature kappa driven at constant speed circle_speed, starting at (x0, y0) with
// heading theta0. kappa = 0 is a straight line; kappa > 0 turns left.
class Circle : public Shape
{
public:
  explicit Circle(const Params & params);
  Vec2 position(double t) const override;
  Vec2 velocity(double t) const override;
  Vec2 acceleration(double t) const override;

private:
  Vec2 p0_;
  double theta0_;
  double v_;
  double kappa_;
};

// Lemniscate of Gerono centred on (x0, y0), long axis along theta0, half-length a, one
// lap every `period` seconds. p*(0) = (x0, y0).
class Gerono : public Shape
{
public:
  explicit Gerono(const Params & params);
  Vec2 position(double t) const override;
  Vec2 velocity(double t) const override;
  Vec2 acceleration(double t) const override;

private:
  Vec2 center_;
  double theta0_;
  double a_;
  double period_;
};

// p*, q*, v*, w* and kappa* at time t, from shape.position/velocity/acceleration(t).
Reference reference(const Shape & shape, double t);

// Circle or Gerono from params.shape. Throws std::invalid_argument for another name.
std::unique_ptr<Shape> makeShape(const Params & params);

// Samples reference() on [0, t_end]. Returns a message if |kappa*| > kappa_max or
// |w*| > w_max anywhere, nullopt if the trajectory is inside both limits.
std::optional<std::string> checkLimits(const Shape & shape, const Params & params, double t_end);

}  // namespace gtg_ros
