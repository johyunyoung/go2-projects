#include "go2_mpc/gait_scheduler.hpp"

#include <cmath>

namespace go2 {

double GaitScheduler::cycle(int leg, double t) const {
  const double c = (t - t0_) / params_.period + params_.offsets[leg];
  return c - std::floor(c);
}

GaitScheduler::LegPhase GaitScheduler::leg(int leg, double t) const {
  const double c = cycle(leg, t);
  const double d = params_.duty;
  LegPhase p;
  p.stance = c < d;
  if (p.stance) {
    p.progress = c / d;
    p.remaining = (d - c) * params_.period;
  } else {
    p.progress = (c - d) / (1.0 - d);
    p.remaining = (1.0 - c) * params_.period;
  }
  return p;
}

}  // namespace go2
