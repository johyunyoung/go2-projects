#pragma once

#include <array>

#include "go2_mpc/config.hpp"
#include "go2_mpc/foot_plan.hpp"
#include "go2_mpc/gait_scheduler.hpp"
#include "go2_mpc/state_estimator.hpp"
#include "go2_mpc/swing_trajectory.hpp"

namespace go2 {

// M4 bench test with the base welded in the air.
//
// A virtual body moves forward at swing_test.speed. Each leg follows exactly what it
// would on the ground: in stance the foot stays at its touch-down point in the
// virtual world; in swing it follows a Bezier from lift-off to the Raibert target
//   p_td = nominal + x_body(t_touchdown) + v T_stance / 2
// (re-planned until swing.freeze_after). Everything is converted to the base frame
// and tracked with the swing-leg controller, so tracking error is measurable on
// every leg all the time.
class SwingTestController {
 public:
  explicit SwingTestController(const Config& config);

  void start(const RobotState& s, double t);
  JointCommand step(double t, const RobotState& s);
  const FootPlan& plan() const { return plan_; }

 private:
  Vec3 touchdownTarget(int leg, double time_to_touchdown) const;
  void report(double t);

  const Config& config_;
  GaitScheduler gait_;
  std::array<Vec3, kNumLegs> nominal_{};  // foot at the stand pose, base frame
  std::array<SwingTrajectory, kNumLegs> swing_{};
  std::array<double, kNumLegs> swing_t0_{};
  std::array<bool, kNumLegs> was_stance_{};
  std::array<Vec3, kNumLegs> stance_point_{};  // virtual world
  double x_body_ = 0.0, speed_ = 0.0;          // virtual body position / speed (x)
  double t_start_ = 0.0, t_last_ = 0.0;
  FootPlan plan_;

  // Tracking error statistics since the last report (after the speed ramp).
  struct Stats {
    double max = 0.0, sum_sq = 0.0, max_swing = 0.0;
    long n = 0;
  };
  std::array<Stats, kNumLegs> stats_{};
  double last_report_ = 0.0;
};

}  // namespace go2
