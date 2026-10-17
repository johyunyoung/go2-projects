#include "go2_mpc/state_estimator.hpp"

#include <algorithm>
#include <cmath>

#include "go2_mpc/kinematics.hpp"

namespace go2 {
namespace {

// Everything that does not depend on the base position/velocity.
RobotState fromImuAndJoints(const LowStateData& low) {
  RobotState s;
  s.R = low.quat.normalized().toRotationMatrix();
  s.rpy = rotationToRpy(s.R);
  s.omega_body = low.gyro;
  s.omega_world = s.R * low.gyro;
  s.q = low.q;
  s.dq = low.dq;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    s.foot_body[leg] = kin::footPosition(leg, kin::legJoints(s.q, leg));
  }
  return s;
}

void setFeetWorld(RobotState& s) {
  for (int leg = 0; leg < kNumLegs; ++leg) s.foot_world[leg] = s.pos + s.R * s.foot_body[leg];
}

// Velocity of the ball foot's centre when it rolls without slipping on flat ground:
// the touching surface point is still, so v_centre = r (w_calf x z).
// Calf angular velocity (base frame): w_body + x dq_hip + Rx(q_hip) y (dq_thigh + dq_calf).
Vec3 rollingVelocity(const RobotState& s, int leg) {
  const Vec3 q = kin::legJoints(s.q, leg), dq = kin::legJoints(s.dq, leg);
  const Vec3 w_calf_body = s.omega_body + Vec3::UnitX() * dq[0] +
                           Eigen::AngleAxisd(q[0], Vec3::UnitX()) * Vec3::UnitY() * (dq[1] + dq[2]);
  return kin::kFootRadius * (s.R * w_calf_body).cross(Vec3::UnitZ());
}

}  // namespace

Vec3 rotationToRpy(const Mat3& R) {
  return {std::atan2(R(2, 1), R(2, 2)), std::asin(std::clamp(-R(2, 0), -1.0, 1.0)),
          std::atan2(R(1, 0), R(0, 0))};
}

RobotState GroundTruthEstimator::update(const LowStateData& low,
                                        const std::optional<BaseGroundTruth>& gt,
                                        const ContactEstimate& /*contact*/) {
  RobotState s = fromImuAndJoints(low);
  if (gt) {
    s.pos = gt->pos;
    s.vel = gt->vel;
  }
  setFeetWorld(s);
  s.valid = gt.has_value();
  return s;
}

KalmanEstimator::KalmanEstimator(const Config& config) : config_(config) {}

// Start with the feet on the ground under the legs and the body at rest. In the
// simulator the horizontal position is taken from ground truth so that the estimate
// and the truth share an origin; on the robot it starts at (0, 0).
void KalmanEstimator::initialize(const RobotState& s, const std::optional<BaseGroundTruth>& gt) {
  double lowest = 0.0;
  for (int leg = 0; leg < kNumLegs; ++leg) lowest = std::min(lowest, (s.R * s.foot_body[leg]).z());
  Vec3 p(0.0, 0.0, kin::kFootRadius - lowest);
  if (gt) p.head<2>() = gt->pos.head<2>();

  x_.setZero();
  x_.segment<3>(0) = p;
  for (int leg = 0; leg < kNumLegs; ++leg) x_.segment<3>(6 + 3 * leg) = p + s.R * s.foot_body[leg];
  P_.setIdentity();
  P_ *= 0.01;
  initialized_ = true;
}

RobotState KalmanEstimator::update(const LowStateData& low,
                                   const std::optional<BaseGroundTruth>& gt,
                                   const ContactEstimate& contact) {
  RobotState s = fromImuAndJoints(low);
  if (!initialized_) {
    // In the simulator, wait for ground truth so both start from the same origin.
    if (config_.robot.use_ground_truth && !gt) return s;  // valid = false
    initialize(s, gt);
  }

  const auto& e = config_.estimator;
  const double dt = config_.control.dt;
  const Mat3 I3 = Mat3::Identity();

  // ---- predict ----
  // The accelerometer measures specific force in the body frame: a = R f + g.
  const Vec3 accel = s.R * low.acc + Vec3(0.0, 0.0, -config_.model.gravity);
  StateMat A = StateMat::Identity();
  A.block<3, 3>(0, 3) = dt * I3;
  x_.segment<3>(0) += dt * x_.segment<3>(3);
  x_.segment<3>(3) += dt * accel;
  std::array<Vec3, kNumLegs> v_roll;  // Eigen vectors are not zeroed by {}
  v_roll.fill(Vec3::Zero());
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (e.foot_rolling) v_roll[leg] = rollingVelocity(s, leg);
    x_.segment<3>(6 + 3 * leg) += dt * std::clamp(contact.trust[leg], 0.0, 1.0) * v_roll[leg];
  }

  StateMat Q = StateMat::Zero();
  Q.block<3, 3>(0, 0) = (dt / 20.0) * e.process_p * I3;
  Q.block<3, 3>(3, 3) = (dt * 9.8 / 20.0) * e.process_v * I3;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double scale = 1.0 + (1.0 - contact.trust[leg]) * e.untrusted_scale;
    Q.block<3, 3>(6 + 3 * leg, 6 + 3 * leg) = scale * dt * e.process_foot * I3;
  }
  P_ = A * P_ * A.transpose() + Q;

  // ---- measure ----
  Eigen::Matrix<double, kM, 1> y;
  Eigen::Matrix<double, kM, kN> H = Eigen::Matrix<double, kM, kN>::Zero();
  Eigen::Matrix<double, kM, 1> r;  // diagonal measurement noise
  const Vec3 v_hat = x_.segment<3>(3);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const double trust = std::clamp(contact.trust[leg], 0.0, 1.0);
    const double scale = 1.0 + (1.0 - trust) * e.untrusted_scale;
    const Vec3 p_rel = s.R * s.foot_body[leg];

    // Leg kinematics: foot relative to the base.
    y.segment<3>(3 * leg) = p_rel;
    H.block<3, 3>(3 * leg, 0) = -I3;
    H.block<3, 3>(3 * leg, 6 + 3 * leg) = I3;
    r.segment<3>(3 * leg).setConstant(scale * e.meas_p_rel);

    // Leg odometry: a planted foot does not move, so the body moves opposite to it.
    const Mat3 J = kin::footJacobian(leg, kin::legJoints(s.q, leg));
    const Vec3 foot_vel_rel = s.R * (s.omega_body.cross(s.foot_body[leg]) + J * kin::legJoints(s.dq, leg));
    y.segment<3>(12 + 3 * leg) = (1.0 - trust) * v_hat + trust * (v_roll[leg] - foot_vel_rel);
    H.block<3, 3>(12 + 3 * leg, 3) = I3;
    r.segment<3>(12 + 3 * leg).setConstant(scale * e.meas_v);

    // Flat ground: a planted foot point sits one foot radius above z = 0.
    const double z_free = x_[2] + p_rel.z();
    y[24 + leg] = (1.0 - trust) * z_free + trust * kin::kFootRadius;
    H(24 + leg, 6 + 3 * leg + 2) = 1.0;
    r[24 + leg] = scale * e.meas_z;
  }

  const Eigen::Matrix<double, kM, kM> S = H * P_ * H.transpose() + Eigen::Matrix<double, kM, kM>(r.asDiagonal());
  const Eigen::Matrix<double, kN, kM> K = P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix<double, kM, kM>::Identity());
  x_ += K * (y - H * x_);
  P_ = (StateMat::Identity() - K * H) * P_;
  P_ = 0.5 * (P_ + P_.transpose());

  s.pos = x_.segment<3>(0);
  s.vel = x_.segment<3>(3);
  setFeetWorld(s);
  s.valid = true;
  return s;
}

}  // namespace go2
