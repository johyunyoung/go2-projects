#pragma once

#include "go2_mpc/types.hpp"

namespace go2::leg {

// Stance: joint torques that make the foot exert -f on the ground, where f is the
// ground reaction force on the robot in the world frame:  tau = -J^T R^T f + tau_ff
// (tau_ff: the leg's own gravity torque, so the force f carries the body only).
// Writes the leg's slots of cmd (q/dq/kp = 0, kd = damping).
void stance(JointCommand& cmd, int leg, const Vec12& q, const Mat3& R, const Vec3& f_world,
            const Vec3& kd, const Vec3& tau_ff = Vec3::Zero());

// Desired foot motion in the base frame (relative to the base origin, i.e. already
// compensated for the body's own motion).
struct FootTarget {
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  Vec3 acc = Vec3::Zero();
};

// Swing: the motor PD tracks q_des = IK(p) with dq_des = J^-1 v as velocity
// feed-forward. Feed-forward torque on top:
//   inertia:  M(q_des) qdd_des,  qdd_des = J^-1 (a - Jdot dq_des)   (if inertia_ff)
//   tau_ff:   e.g. the leg's own gravity torque
// Returns false if the target was out of reach (the command then uses the projected IK).
bool swing(JointCommand& cmd, int leg, const FootTarget& target, const Vec3& kp, const Vec3& kd,
           bool inertia_ff, const Vec3& tau_ff);

// Foot velocity relative to the base, expressed in the base frame, for a desired
// world-frame foot velocity:  v_body = R^T (v_world - v_base) - omega_body x p_body.
Vec3 worldToBodyVelocity(const Vec3& v_world, const Vec3& v_base, const Vec3& omega_body,
                         const Mat3& R, const Vec3& p_body);

}  // namespace go2::leg
