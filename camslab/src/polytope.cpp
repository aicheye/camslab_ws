/* Safety polytope: a convex polygon given by its vertices, counter-clockwise.

Edge i runs from v_i to v_{i+1}. For a counter-clockwise polygon the inside is on the left
of every edge, so the distance of p from the line of edge i, positive on the left, is

    d_i(p) = cross(v_{i+1} - v_i, p - v_i) / ||v_{i+1} - v_i||

and p is inside when every d_i(p) >= 0. signedDistance() returns min_i d_i(p).
*/

#include "camslab/polytope.hpp"

#include <algorithm>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace camslab
{

namespace
{

double cross(const Vec2 & u, const Vec2 & v) { return u.x * v.y - u.y * v.x; }

}  // namespace

Polytope::Polytope(const std::vector<double> & xy)
{
  if (xy.size() % 2 != 0) {
    throw std::invalid_argument(
      "polytope has " + std::to_string(xy.size()) + " numbers, expected x, y pairs");
  }
  for (std::size_t i = 0; i < xy.size(); i += 2) {
    vertices_.push_back({xy[i], xy[i + 1]});
  }
  const std::size_t n = vertices_.size();
  if (n < 3) {
    throw std::invalid_argument(
      "polytope has " + std::to_string(n) + " vertices, expected at least 3");
  }
  // Convex and counter-clockwise: every corner turns left.
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 & a = vertices_[i];
    const Vec2 & b = vertices_[(i + 1) % n];
    const Vec2 & c = vertices_[(i + 2) % n];
    if (cross(b - a, c - b) <= 0.0) {
      throw std::invalid_argument(
        "polytope turns right or goes straight at vertex " + std::to_string((i + 1) % n + 1) +
        "; the vertices must go counter-clockwise around a convex polygon (the web UI "
        "writes them in this order)");
    }
  }
}

double Polytope::signedDistance(const Vec2 & p) const
{
  double d = std::numeric_limits<double>::infinity();
  const std::size_t n = vertices_.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 & a = vertices_[i];
    const Vec2 edge = vertices_[(i + 1) % n] - a;
    d = std::min(d, cross(edge, p - a) / edge.norm());
  }
  return d;
}

std::optional<std::string> checkInside(
  const Shape & shape, const Polytope & polytope, double margin, double t_end)
{
  constexpr int kSamples = 2000;
  double d_worst = std::numeric_limits<double>::infinity();
  double t_worst = 0.0;
  Vec2 p_worst;
  for (int i = 0; i <= kSamples; ++i) {
    const double t = t_end * i / kSamples;
    const Vec2 p = shape.position(t);
    const double d = polytope.signedDistance(p);
    if (d < d_worst) {
      d_worst = d;
      t_worst = t;
      p_worst = p;
    }
  }
  if (d_worst >= margin) {
    return std::nullopt;
  }
  char message[200];
  if (d_worst < 0.0) {
    std::snprintf(
      message, sizeof(message),
      "p*(t) = (%.2f, %.2f) m at t = %.2f s is %.3f m outside the safety polytope", p_worst.x,
      p_worst.y, t_worst, -d_worst);
  } else {
    std::snprintf(
      message, sizeof(message),
      "p*(t) = (%.2f, %.2f) m at t = %.2f s is %.3f m from the safety polytope edge, less "
      "than polytope_margin = %.3f m",
      p_worst.x, p_worst.y, t_worst, d_worst, margin);
  }
  return std::string(message);
}

}  // namespace camslab
