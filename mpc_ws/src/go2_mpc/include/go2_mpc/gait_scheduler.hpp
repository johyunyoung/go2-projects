#pragma once

#include <array>

#include "go2_mpc/types.hpp"

namespace go2 {

// Periodic gait: each leg has a phase offset; within one period the leg is in stance
// for the first `duty` fraction of its own cycle and in swing for the rest.
//
//   leg cycle c_i(t) = frac((t - t0) / period + offset_i)
//   stance  while c_i <  duty,  progress = c_i / duty
//   swing   while c_i >= duty,  progress = (c_i - duty) / (1 - duty)
class GaitScheduler {
 public:
  struct Params {
    double period = 0.3;
    double duty = 0.5;
    std::array<double, kNumLegs> offsets{0.0, 0.5, 0.5, 0.0};  // FR, FL, RR, RL (trot)
  };

  struct LegPhase {
    bool stance = true;
    double progress = 0.0;   // 0..1 within the current stance or swing
    double remaining = 0.0;  // time left in the current stance or swing [s]
  };

  explicit GaitScheduler(const Params& params) : params_(params) {}

  void start(double t) { t0_ = t; }
  LegPhase leg(int leg, double t) const;
  bool inContact(int leg, double t) const { return cycle(leg, t) < params_.duty; }

  double stanceTime() const { return params_.duty * params_.period; }
  double swingTime() const { return (1.0 - params_.duty) * params_.period; }
  const Params& params() const { return params_; }

 private:
  double cycle(int leg, double t) const;

  Params params_;
  double t0_ = 0.0;
};

}  // namespace go2
