#include "go2_mpc/stand_controller.hpp"

#include <numeric>

#include "go2_mpc/kinematics.hpp"

namespace go2 {
namespace {

double smoothstep(double u) { return u * u * (3.0 - 2.0 * u); }

}  // namespace

void StandController::start(const Vec12& q_now, double t, bool hold) {
  q_start_ = q_now;
  t0_ = t;
  first_ = hold ? stand_.waypoints.size() - 1 : 0;
  total_duration_ = std::accumulate(stand_.durations.begin() + first_, stand_.durations.end(), 0.0);
  reported_ = false;
}

JointCommand StandController::step(double t, const Vec12& q, const Mat3& R) const {
  JointCommand cmd;
  cmd.kp = stand_.kp;
  cmd.kd = stand_.kd;
  cmd.q = stand_.waypoints.back();  // hold once all segments are done

  double elapsed = t - t0_;
  const Vec12* from = &q_start_;
  for (size_t i = first_; i < stand_.waypoints.size(); ++i) {
    const Vec12& to = stand_.waypoints[i];
    const double T = stand_.durations[i];
    if (elapsed < T) {
      const double u = elapsed / T;
      cmd.q = *from + smoothstep(u) * (to - *from);
      cmd.dq = (6.0 * u * (1.0 - u) / T) * (to - *from);
      break;
    }
    elapsed -= T;
    from = &to;
  }

  if (stand_.gravity_compensation) {
    const double w = loadFraction(t);
    // Ground reaction force on each foot, expressed in the body frame.
    const Vec3 f_body = R.transpose() * Vec3(0.0, 0.0, w * config_.model.mass * config_.model.gravity / kNumLegs);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const Mat3 J = kin::footJacobian(leg, kin::legJoints(q, leg));
      cmd.tau.segment<3>(3 * leg) = -J.transpose() * f_body;
    }
  }
  return cmd;
}

double StandController::loadFraction(double t) const {
  if (first_ > 0) return 1.0;  // hold: already standing on the feet
  const double last = stand_.durations.back();
  const double u = (t - t0_ - (total_duration_ - last)) / last;
  if (u <= 0.0) return 0.0;
  if (u >= 1.0) return 1.0;
  return smoothstep(u);
}

bool StandController::reportDue(double t) {
  if (reported_ || t - t0_ < total_duration_ + stand_.report_delay) return false;
  reported_ = true;
  return true;
}

}  // namespace go2
