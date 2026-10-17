#pragma once

#include "go2_mpc/types.hpp"

namespace go2 {

// Foot swing from lift-off p0 to touch-down pf (world frame) over duration T.
//
// Horizontal (x, y): cubic Bezier with control points [p0, p0, pf, pf]
//   -> p0 + (pf - p0)(3s^2 - 2s^3): zero velocity at both ends.
// Vertical (z): two cubic Bezier halves, each with doubled end points,
//   s in [0, 1/2]: z0 -> z_top,   s in [1/2, 1]: z_top -> zf,
//   z_top = max(z0, zf) + height: apex at mid-swing, zero vertical velocity at
//   lift-off, apex and touch-down.
// s = t / T. Velocity and acceleration are exact derivatives w.r.t. time.
class SwingTrajectory {
 public:
  struct Point {
    Vec3 pos = Vec3::Zero();
    Vec3 vel = Vec3::Zero();
    Vec3 acc = Vec3::Zero();
  };

  SwingTrajectory() = default;
  SwingTrajectory(const Vec3& p0, const Vec3& pf, double height, double duration)
      : p0_(p0), pf_(pf), height_(height), duration_(duration) {}

  // Moves the touch-down target (e.g. the foothold is re-planned during swing).
  void setTarget(const Vec3& pf) { pf_ = pf; }

  Point evaluate(double t) const;  // t is clamped to [0, duration]

  const Vec3& start() const { return p0_; }
  const Vec3& target() const { return pf_; }
  double duration() const { return duration_; }
  double apexHeight() const { return std::max(p0_.z(), pf_.z()) + height_; }

 private:
  Vec3 p0_ = Vec3::Zero();
  Vec3 pf_ = Vec3::Zero();
  double height_ = 0.0;
  double duration_ = 1.0;
};

// Cubic Bezier with control points [a, a, b, b] at parameter u in [0, 1]:
// value, d/du and d2/du2.
struct CubicBlend {
  double value, d1, d2;
};
CubicBlend cubicBlend(double a, double b, double u);

}  // namespace go2
