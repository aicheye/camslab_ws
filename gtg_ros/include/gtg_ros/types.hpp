#pragma once

#include <cmath>
#include <string>

namespace gtg_ros {

struct Vec2 {
  double x{0.0};
  double y{0.0};
  inline Vec2 operator-() const { return {-x, -y}; }
  double norm() const { return std::sqrt(x * x + y * y); }

  inline static double dot(const Vec2 &u, const Vec2 &v) {
    return u.x * v.x + u.y * v.y;
  }
};

inline Vec2 operator*(const double s, const Vec2 &u) {
  return {s * u.x, s * u.y};
}

inline Vec2 operator*(const Vec2 &u, const double s) { return s * u; }

inline Vec2 operator/(const Vec2 &u, const double s) { return (1 / s) * u; }

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

// Reference at one time t, from reference() in trajectory.hpp.
struct Reference {
  Vec2 p;            // p*(t) [m]
  Vec2 q{1.0, 0.0};  // q*(t), unit tangent of the path
  double v{0.0};     // v*(t) = ||dp*/dt|| [m/s]
  double w{0.0};     // w*(t), from dq*/dt = w* S q* [rad/s]
  double kappa{0.0}; // kappa*(t), curvature, w* = kappa* v* [1/m]
};

struct Params {
  // Trajectory. (x0, y0, theta0) is the pose of the trajectory frame: the
  // circle starts at (x0, y0) heading theta0, the Gerono lemniscate is centred
  // on (x0, y0) with its long axis along theta0.
  std::string shape{"circle"}; // circle or gerono
  double x0{2.0};              // [m]
  double y0{1.0};              // [m]
  double theta0{0.0};          // [rad]
  double circle_speed{0.3};    // circle: constant speed [m/s]
  double kappa{1.0};   // circle: curvature 1/r [1/m], 0 is a straight line
  double a{2.0};       // gerono: half the length of the long axis [m]
  double period{40.0}; // gerono: time for one lap [s]

  // Control law gains, all > 0.
  double k_par{1.0};  // k_parallel [1/s]
  double k_perp{4.0}; // k_perp [1/m^2]
  double k_q{2.0};    // k_q [1/s]

  // Limits the reference must stay inside (checkLimits in trajectory.hpp).
  double kappa_max{
      2.9};          // |kappa*| [1/m]; the R2 turns at most tan(0.6) / 0.235
  double w_max{2.0}; // |w*| [rad/s]
};

} // namespace gtg_ros
