#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "camslab/types.hpp"

namespace camslab
{

// A reference trajectory p*(t) and its first two time derivatives. Same shapes
// as camslab_sim/trajectory.py.
class Shape
{
public:
  virtual ~Shape() = default;
  virtual Vec2 position(double t) const = 0;      // p*(t) [m]
  virtual Vec2 velocity(double t) const = 0;      // dp*/dt [m/s]
  virtual Vec2 acceleration(double t) const = 0;  // d^2 p*/dt^2 [m/s^2]
};

// Circle of curvature kappa driven at constant speed circle_speed, starting at
// (x0, y0) with heading theta0. kappa = 0 is a straight line; kappa > 0 turns
// left.
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
  Vec2 heading(double t) const;
};

// Lemniscate of Gerono centred on (x0, y0), long axis along theta0, half-length
// a, one lap every `period` seconds, at lap angle phi0 when t = 0. The crossing
// at (x0, y0) is lap angle 0, so p*(0) = (x0, y0) when phi0 = 0.
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
  double phi0_;
};

// Ellipse centred on (x0, y0), half-axis a along theta0 and half-axis b
// perpendicular to it, one counter-clockwise lap every `period` seconds, at lap
// angle phi0 when t = 0. Lap angle 0 is the end of the a axis, (x0, y0) + a
// (cos theta0, sin theta0).
class Ellipse : public Shape
{
public:
  explicit Ellipse(const Params & params);
  Vec2 position(double t) const override;
  Vec2 velocity(double t) const override;
  Vec2 acceleration(double t) const override;

private:
  Vec2 center_;
  double theta0_;
  double a_;
  double b_;
  double period_;
  double phi0_;
};

// Rectangle centred on (x0, y0) with side 2a along theta0 and side 2b perpendicular to
// it, one counter-clockwise lap every `period` seconds at constant speed
// 4 (a + b) / period. p*(0) is the middle of the side at (x0, y0) + a (cos theta0,
// sin theta0), heading theta0 + pi/2. p*(t + period) = p*(t).
// Same as Rectangle in camslab_sim/trajectory.py.
class Rectangle : public Shape
{
public:
  explicit Rectangle(const Params & params);
  Vec2 position(double t) const override;
  Vec2 velocity(double t) const override;
  Vec2 acceleration(double t) const override;

private:
  Vec2 center_;
  double theta0_;
  double a_;
  double b_;
  double period_;
};

// Closed uniform cubic B-spline through the control points P_0, ..., P_{N-1} of
// control_points, one lap every `period` seconds, N >= 4. Same as Spline in
// camslab_sim/trajectory.py, whose docstring describes the segments. The curve stays
// inside the convex hull of the control points.
class Spline : public Shape
{
public:
  // Throws std::invalid_argument unless control_points has at least 4 points.
  explicit Spline(const Params & params);
  Vec2 position(double t) const override;
  Vec2 velocity(double t) const override;
  Vec2 acceleration(double t) const override;

private:
  std::vector<Vec2> points_;  // P_k [m]
  double period_;

  struct Vals
  {
    int N, i;
    double u;
  };

  Vals calc(double t) const;
};

// Points {x0, y0}, {x1, y1}, ... from the flat array [x0, y0, x1, y1, ...] of the
// control_points parameter. Throws std::invalid_argument if xy has an odd length.
std::vector<Vec2> toPoints(const std::vector<double> & xy);

// p*, q*, v*, w* and kappa* at time t, from
// shape.position/velocity/acceleration(t).
Reference reference(const Shape & shape, double t);

// Circle, Gerono, Ellipse, Rectangle or Spline from params.shape. Throws std::invalid_argument for another
// name.
std::unique_ptr<Shape> makeShape(const Params & params);

// Samples reference() on [0, t_end]. Returns a message naming the largest |kappa*|
// or |w*| if it is above kappa_max or w_max, nullopt if the trajectory is inside both.
std::optional<std::string> checkLimits(const Shape & shape, const Params & params, double t_end);

}  // namespace camslab
