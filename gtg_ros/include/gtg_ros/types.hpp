#pragma once

#include <cmath>

namespace gtg_ros {

struct Vec2 {
  double x{0.0};
  double y{0.0};
  inline Vec2 operator-() const { return {-x, -y}; }
  double norm() const { return std::sqrt(x * x + y * y); }
};

inline Vec2 operator*(const double s, const Vec2 &u) {
  return {s * u.x, s * u.y};
}

inline Vec2 operator+(const Vec2 &u, const Vec2 &v) {
  return {u.x + v.x, u.y + v.y};
}

inline Vec2 operator-(const Vec2 &u, const Vec2 &v) { return u + (-v); }

// 2x2 matrix [[a, b], [c, d]], stored row by row. Mat2{a, b, c, d}; the default
// is identity.
struct Mat2 {
  double a{1.0};
  double b{0.0};
  double c{0.0};
  double d{1.0};

  inline static Vec2 dot(const Mat2 &m, const Vec2 &u) {
    return {m.a * u.x + m.b * u.y, m.c * u.x + m.d * u.y};
  }
};

// p = [x, y] in the global frame. q = [cos(theta), sin(theta)], ||q|| = 1.
struct State {
  Vec2 p;
  Vec2 q{1.0, 0.0};
};

struct Command {
  double v{0.0}; // linear velocity [m/s]
  double w{0.0}; // angular velocity [rad/s]
};

struct Params {
  double v_star{0.5};   // cruise speed used when ||p* - p|| > D [m/s]
  double d_switch{0.5}; // D, the switching distance [m]
  double k_v{1.0};      // Kv [1/s]
  double k_w{2.0};      // Kw [1/s]
  double epsilon{0.05}; // goal tolerance [m]
  double spacing{0.5};  // follower: distance from the leader to its goal [m]
};

} // namespace gtg_ros
