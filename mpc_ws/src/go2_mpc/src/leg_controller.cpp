#include "go2_mpc/leg_controller.hpp"

#include <cmath>

#include "go2_mpc/kinematics.hpp"

namespace go2::leg {

void stance(JointCommand& cmd, int leg, const Vec12& q, const Mat3& R, const Vec3& f_world,
            const Vec3& kd, const Vec3& tau_ff) {
  const Mat3 J = kin::footJacobian(leg, kin::legJoints(q, leg));
  cmd.q.segment<3>(3 * leg).setZero();
  cmd.dq.segment<3>(3 * leg).setZero();
  cmd.kp.segment<3>(3 * leg).setZero();
  cmd.kd.segment<3>(3 * leg) = kd;
  cmd.tau.segment<3>(3 * leg) = -J.transpose() * (R.transpose() * f_world) + tau_ff;
}

bool swing(JointCommand& cmd, int leg, const FootTarget& target, const Vec3& kp, const Vec3& kd,
           bool inertia_ff, const Vec3& tau_ff) {
  Vec3 q_des;
  const bool reachable = kin::inverse(leg, target.pos, q_des);
  Vec3 dq_des = Vec3::Zero();
  Vec3 tau = tau_ff;

  // J^-1 blows up near stretched (singular) postures; skip the feed-forward there.
  const Mat3 J = kin::footJacobian(leg, q_des);
  if (std::abs(J.determinant()) > 1e-6) {
    const auto lu = J.partialPivLu();
    dq_des = lu.solve(target.vel);
    if (inertia_ff) {
      // Jdot dq by a central difference of J along dq.
      constexpr double h = 1e-4;
      const Vec3 jdot_dq = (kin::footJacobian(leg, q_des + h * dq_des) -
                            kin::footJacobian(leg, q_des - h * dq_des)) / (2.0 * h) * dq_des;
      const Vec3 ddq_des = lu.solve(target.acc - jdot_dq);
      tau += kin::legMassMatrix(leg, q_des) * ddq_des;
    }
  }

  cmd.q.segment<3>(3 * leg) = q_des;
  cmd.dq.segment<3>(3 * leg) = dq_des;
  cmd.kp.segment<3>(3 * leg) = kp;
  cmd.kd.segment<3>(3 * leg) = kd;
  cmd.tau.segment<3>(3 * leg) = tau;
  return reachable;
}

Vec3 worldToBodyVelocity(const Vec3& v_world, const Vec3& v_base, const Vec3& omega_body,
                         const Mat3& R, const Vec3& p_body) {
  return R.transpose() * (v_world - v_base) - omega_body.cross(p_body);
}

}  // namespace go2::leg
