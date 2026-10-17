#include "go2_mpc/swing_trajectory.hpp"

#include <algorithm>

namespace go2 {

// Bernstein form of [a, a, b, b]: B(u) = a (1-u)^3 + 3a u(1-u)^2 + 3b u^2(1-u) + b u^3
//                                       = a + (b - a)(3u^2 - 2u^3)
CubicBlend cubicBlend(double a, double b, double u) {
  const double d = b - a;
  return {a + d * u * u * (3.0 - 2.0 * u), d * 6.0 * u * (1.0 - u), d * (6.0 - 12.0 * u)};
}

SwingTrajectory::Point SwingTrajectory::evaluate(double t) const {
  const double T = duration_;
  const double s = std::clamp(t / T, 0.0, 1.0);
  Point p;

  for (int i = 0; i < 2; ++i) {
    const CubicBlend b = cubicBlend(p0_[i], pf_[i], s);
    p.pos[i] = b.value;
    p.vel[i] = b.d1 / T;
    p.acc[i] = b.d2 / (T * T);
  }

  // Each z half spans T/2, so d/dt = (2/T) d/du.
  const double z_top = apexHeight();
  const bool rising = s < 0.5;
  const double u = rising ? 2.0 * s : 2.0 * s - 1.0;
  const CubicBlend bz = rising ? cubicBlend(p0_.z(), z_top, u) : cubicBlend(z_top, pf_.z(), u);
  p.pos.z() = bz.value;
  p.vel.z() = bz.d1 * 2.0 / T;
  p.acc.z() = bz.d2 * 4.0 / (T * T);
  return p;
}

}  // namespace go2
