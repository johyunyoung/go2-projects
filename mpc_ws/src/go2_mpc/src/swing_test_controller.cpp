#include "go2_mpc/swing_test_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "go2_mpc/kinematics.hpp"
#include "go2_mpc/leg_controller.hpp"

namespace go2 {
namespace {

GaitScheduler::Params gaitParams(const Config& c) {
  GaitScheduler::Params p;
  p.period = c.gait.period;
  p.duty = c.gait.duty;
  p.offsets = c.gait.offsets;
  return p;
}

}  // namespace

SwingTestController::SwingTestController(const Config& config)
    : config_(config), gait_(gaitParams(config)) {
  const Vec12& q_stand = config.stand.waypoints.back();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    nominal_[leg] = kin::footPosition(leg, kin::legJoints(q_stand, leg));
  }
}

// Raibert target in the virtual world: the nominal foot under the hip where the body
// will be at touch-down, plus half a stance length ahead.
Vec3 SwingTestController::touchdownTarget(int leg, double time_to_touchdown) const {
  const double x_td = x_body_ + speed_ * time_to_touchdown;
  return nominal_[leg] + Vec3(x_td + 0.5 * speed_ * gait_.stanceTime(), 0.0, 0.0);
}

void SwingTestController::start(const RobotState& s, double t) {
  (void)s;
  gait_.start(t);
  t_start_ = t_last_ = last_report_ = t;
  x_body_ = speed_ = 0.0;
  stats_ = {};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto ph = gait_.leg(leg, t);
    was_stance_[leg] = ph.stance;
    stance_point_[leg] = nominal_[leg];
    // Legs starting in swing step in place (speed is still 0).
    swing_[leg] = SwingTrajectory(nominal_[leg], nominal_[leg], config_.swing.height, gait_.swingTime());
    swing_t0_[leg] = t - ph.progress * gait_.swingTime();
  }
  std::printf("[swing-test] start: virtual speed %.2f m/s (ramp %.1f s), step length %.3f m, "
              "swing %.3f s, height %.3f m\n",
              config_.swing_test.speed, config_.swing_test.ramp,
              config_.swing_test.speed * gait_.stanceTime(), gait_.swingTime(), config_.swing.height);
}

JointCommand SwingTestController::step(double t, const RobotState& s) {
  const double dt = t - t_last_;
  t_last_ = t;
  const double ramp = config_.swing_test.ramp;
  speed_ = config_.swing_test.speed * std::clamp((t - t_start_) / std::max(ramp, 1e-6), 0.0, 1.0);
  x_body_ += speed_ * dt;
  const Vec3 body(x_body_, 0.0, 0.0), body_vel(speed_, 0.0, 0.0);
  const Vec3 g_body = s.R.transpose() * Vec3(0.0, 0.0, -config_.model.gravity);

  JointCommand cmd;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto ph = gait_.leg(leg, t);
    if (was_stance_[leg] && !ph.stance) {  // lift-off
      swing_[leg] = SwingTrajectory(stance_point_[leg], touchdownTarget(leg, ph.remaining),
                                    config_.swing.height, gait_.swingTime());
      swing_t0_[leg] = t - ph.progress * gait_.swingTime();
    } else if (!was_stance_[leg] && ph.stance) {  // touch-down
      stance_point_[leg] = swing_[leg].target();
    }
    was_stance_[leg] = ph.stance;

    Vec3 p_world, v_world = Vec3::Zero(), a_world = Vec3::Zero();
    if (ph.stance) {
      p_world = stance_point_[leg];
    } else {
      if (ph.progress < config_.swing.freeze_after) {
        swing_[leg].setTarget(touchdownTarget(leg, ph.remaining));
      }
      const auto pt = swing_[leg].evaluate(t - swing_t0_[leg]);
      p_world = pt.pos;
      v_world = pt.vel;
      a_world = pt.acc;
    }

    // Virtual world -> base frame (the virtual body only translates along x).
    // (The virtual body's acceleration during the speed ramp is ignored.)
    leg::FootTarget target;
    target.pos = p_world - body;
    target.vel = v_world - body_vel;
    target.acc = a_world;
    const Vec3& p_body = target.pos;
    const Vec3 tau_g = config_.swing.gravity_compensation
                           ? kin::legGravityTorque(leg, kin::legJoints(s.q, leg), g_body)
                           : Vec3::Zero();
    leg::swing(cmd, leg, target, config_.swing.kp.segment<3>(3 * leg),
               config_.swing.kd.segment<3>(3 * leg), config_.swing.inertia_feedforward, tau_g);

    // Debug: desired point and swing curve in the real world frame.
    plan_.stance[leg] = ph.stance;
    plan_.des_world[leg] = s.pos + s.R * p_body;
    for (int i = 0; i < FootPlan::kCurvePoints; ++i) {
      Vec3 p = p_body;
      if (!ph.stance) {
        // Sample the whole swing, shifted by where the virtual body is now.
        const double ti = swing_[leg].duration() * i / (FootPlan::kCurvePoints - 1);
        p = swing_[leg].evaluate(ti).pos - body;
      }
      plan_.curve[leg][i] = s.pos + s.R * p;
    }

    // Tracking error after the speed ramp.
    if (t - t_start_ > ramp + 0.5) {
      const double e = (s.foot_body[leg] - p_body).norm();
      auto& st = stats_[leg];
      st.max = std::max(st.max, e);
      if (!ph.stance) st.max_swing = std::max(st.max_swing, e);
      st.sum_sq += e * e;
      ++st.n;
    }
  }

  if (config_.swing_test.report_period > 0 && t - last_report_ >= config_.swing_test.report_period) {
    report(t);
  }
  return cmd;
}

void SwingTestController::report(double t) {
  last_report_ = t;
  if (stats_[0].n == 0) return;
  std::printf("[swing-test] foot tracking error [mm] (max / rms / max in swing):");
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto& st = stats_[leg];
    std::printf("  %s %.1f/%.1f/%.1f", kLegNames[leg], 1e3 * st.max, 1e3 * std::sqrt(st.sum_sq / st.n),
                1e3 * st.max_swing);
  }
  std::printf("\n");
  stats_ = {};
}

}  // namespace go2
