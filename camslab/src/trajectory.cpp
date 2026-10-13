/* Reference trajectories p*(t). Same shapes as camslab_sim/trajectory.py.

Each shape gives p*(t) and its first two time derivatives, differentiated by
hand. reference() turns them into p*, q*, v*, w* and kappa*.
*/

#include "camslab/trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <string>

namespace camslab
{

Circle::Circle(const Params & params)
: p0_{params.x0, params.y0}, theta0_{params.theta0}, v_{params.circle_speed}, kappa_{params.kappa}
{
}

Vec2 Circle::heading(double t) const
{
  return {std::cos(theta0_ + kappa_ * v_ * t), std::sin(theta0_ + kappa_ * v_ * t)};
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
: center_{params.x0, params.y0},
  theta0_{params.theta0},
  a_{params.a},
  period_{params.period},
  phi0_{params.phi0}
{
}

Vec2 Gerono::position(double t) const
{
  Mat2 R{std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double arg{phi0_ + (2 * std::numbers::pi / period_) * t};
  Vec2 i{a_ * std::sin(arg), (a_ / 2) * std::sin(2 * arg)};

  return Mat2::dot(R, i) + center_;
}

Vec2 Gerono::velocity(double t) const
{
  Mat2 R{std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double h{2 * std::numbers::pi / period_};
  double arg{phi0_ + h * t};
  Vec2 i{a_ * h * std::cos(arg), a_ * h * std::cos(2 * arg)};

  return Mat2::dot(R, i);
}

Vec2 Gerono::acceleration(double t) const
{
  Mat2 R = {std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double h{2 * std::numbers::pi / period_};
  double arg{phi0_ + h * t};
  Vec2 i{-a_ * h * h * std::sin(arg), -2 * a_ * h * h * std::sin(2 * arg)};

  return Mat2::dot(R, i);
}

Ellipse::Ellipse(const Params & params)
: center_{params.x0, params.y0},
  theta0_{params.theta0},
  a_{params.a},
  b_{params.b},
  period_{params.period},
  phi0_{params.phi0}
{
}

Vec2 Ellipse::position(double t) const
{
  Mat2 R{std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double arg{phi0_ + (2 * std::numbers::pi / period_) * t};
  Vec2 i{a_ * std::cos(arg), b_ * sin(arg)};

  return Mat2::dot(R, i) + center_;
}

Vec2 Ellipse::velocity(double t) const
{
  Mat2 R = {std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double h{2 * std::numbers::pi / period_};
  double arg{phi0_ + h * t};
  Vec2 i{-a_ * h * std::sin(arg), b_ * h * std::cos(arg)};

  return Mat2::dot(R, i);
}

Vec2 Ellipse::acceleration(double t) const
{
  Mat2 R = {std::cos(theta0_), -std::sin(theta0_), std::sin(theta0_), std::cos(theta0_)};

  double h{2 * std::numbers::pi / period_};
  double arg{phi0_ + h * t};
  Vec2 i{-a_ * h * h * std::cos(arg), -b_ * h * h * std::sin(arg)};

  return Mat2::dot(R, i);
}

Rectangle::Rectangle(const Params & params)
: center_{params.x0, params.y0},
  theta0_{params.theta0},
  a_{params.a},
  b_{params.b},
  period_{params.period}
{
}

Vec2 Rectangle::position(double t) const
{
  double l{4 * a_ + 4 * b_};
  double t1 = (2 * a_ / l) * period_;
  double t2 = t1 + (2 * b_ / l) * period_;
  double t3 = t2 + (2 * a_ / l) * period_;
  double tmod = (t + t1 / 2) - std::floor((t + t1 / 2) / period_) * period_;
  double v{l / period_};
  Vec2 p;

  if (tmod <= t1) {
    p = {-a_, -b_};
    p += tmod * Vec2{v, 0};
  } else if (tmod <= t2) {
    p = {a_, -b_};
    p += (tmod - t1) * Vec2{0, v};
  } else if (tmod <= t3) {
    p = {a_, b_};
    p += (tmod - t2) * Vec2{-v, 0};
  } else {
    p = {-a_, b_};
    p += (tmod - t3) * Vec2{0, -v};
  }

  Mat2 R{cos(theta0_), -sin(theta0_), sin(theta0_), cos(theta0_)};

  return Mat2::dot(R, p) + center_;
}

Vec2 Rectangle::velocity(double t) const
{
  double l{4 * a_ + 4 * b_};
  double t1 = (2 * a_ / l) * period_;
  double t2 = t1 + (2 * b_ / l) * period_;
  double t3 = t2 + (2 * a_ / l) * period_;
  double tmod = (t + t1 / 2) - std::floor((t + t1 / 2) / period_) * period_;
  double v{l / period_};
  Vec2 q;

  if (tmod <= t1) {
    q = {v, 0};
  } else if (tmod <= t2) {
    q = {0, v};
  } else if (tmod <= t3) {
    q = {-v, 0};
  } else {
    q = {0, -v};
  }

  Mat2 R{cos(theta0_), -sin(theta0_), sin(theta0_), cos(theta0_)};

  return Mat2::dot(R, q);
}

Vec2 Rectangle::acceleration(double) const { return {0, 0}; }

Spline::Spline(const Params & params) : points_{params.control_points}, period_{params.period}
{
  if (points_.size() < 4) {
    throw std::invalid_argument(
      "control_points needs at least 4 points, got " + std::to_string(points_.size()));
  }
}

std::vector<Vec2> toPoints(const std::vector<double> & xy)
{
  if (xy.size() % 2 != 0) {
    throw std::invalid_argument(
      "control_points needs x, y pairs, got " + std::to_string(xy.size()) + " numbers");
  }
  std::vector<Vec2> points;
  for (std::size_t i = 0; i < xy.size(); i += 2) {
    points.push_back({xy[i], xy[i + 1]});
  }
  return points;
}

Spline::Vals Spline::calc(double t) const
{
  int N{(int)points_.size()};
  double s{t / (period_ / N)};
  int i{((int)std::floor(s) % N + N) % N};

  double u{s - std::floor(s)};

  return {N, i, u};
}

Vec2 Spline::position(double t) const
{
  auto [N, i, u] = calc(t);
  double u2 = u * u;
  double u3 = u2 * u;

  double b0{(-u3 + 3 * u2 - 3 * u + 1) / 6};
  double b1{(3 * u3 - 6 * u2 + 4) / 6};
  double b2{(-3 * u3 + 3 * u2 + 3 * u + 1) / 6};
  double b3{u3 / 6};

  return b0 * points_[i] + b1 * points_[(i + 1) % N] + b2 * points_[(i + 2) % N] +
         b3 * points_[(i + 3) % N];
}

Vec2 Spline::velocity(double t) const
{
  auto [N, i, u] = calc(t);
  double u2 = u * u;

  double c0 = (-u2 + 2 * u - 1) / 2;
  double c1 = (3 * u2 - 4 * u) / 2;
  double c2 = (-3 * u2 + 2 * u + 1) / 2;
  double c3 = u2 / 2;

  Vec2 c = c0 * points_[i] + c1 * points_[(i + 1) % N] + c2 * points_[(i + 2) % N] +
           c3 * points_[(i + 3) % N];

  return (N / period_) * c;
}

Vec2 Spline::acceleration(double t) const
{
  auto [N, i, u] = calc(t);

  double d0 = -u + 1;
  double d1 = 3 * u - 2;
  double d2 = -3 * u + 1;
  double d3 = u;

  Vec2 d = d0 * points_[i] + d1 * points_[(i + 1) % N] + d2 * points_[(i + 2) % N] +
           d3 * points_[(i + 3) % N];

  return ((N * N) / (period_ * period_)) * d;
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
  if (params.shape == "ellipse") {
    return std::make_unique<Ellipse>(params);
  }
  if (params.shape == "rectangle") {
    return std::make_unique<Rectangle>(params);
  }
  if (params.shape == "spline") {
    return std::make_unique<Spline>(params);
  }
  throw std::invalid_argument(
    "unknown shape '" + params.shape + "', expected circle, gerono, ellipse, rectangle or spline");
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
