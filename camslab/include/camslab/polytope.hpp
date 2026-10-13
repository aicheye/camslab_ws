#pragma once

#include <optional>
#include <string>
#include <vector>

#include "camslab/trajectory.hpp"
#include "camslab/types.hpp"

namespace camslab
{

// Convex polygon on the floor that the robot must stay inside, in the map (mocap) frame.
// The web UI sets the vertices; the controller node reads them from the `polytope`
// parameter as [x1, y1, x2, y2, ...].
class Polytope
{
public:
  // Throws std::invalid_argument unless `xy` has an even length, at least 3 vertices,
  // and the vertices go counter-clockwise around a convex polygon.
  explicit Polytope(const std::vector<double> & xy);

  // Distance from p to the nearest edge line [m], positive inside and negative outside.
  // A point inside is at least `margin` from every edge when this is >= margin.
  double signedDistance(const Vec2 & p) const;

  bool contains(const Vec2 & p) const { return signedDistance(p) >= 0.0; }

  const std::vector<Vec2> & vertices() const { return vertices_; }

private:
  std::vector<Vec2> vertices_;
};

// Samples shape.position() on [0, t_end]. Returns a message naming the point of p*(t)
// nearest the edge if it is less than `margin` inside the polytope, nullopt otherwise.
std::optional<std::string> checkInside(
  const Shape & shape, const Polytope & polytope, double margin, double t_end);

}  // namespace camslab
