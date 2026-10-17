#pragma once

#include <array>
#include <optional>

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Floating-base state used by the controllers. World frame is z-up, gravity -z.
struct RobotState {
  // Base origin (the frame the URDF/MJCF legs attach to).
  Vec3 pos = Vec3::Zero();       // world
  Vec3 vel = Vec3::Zero();       // world
  Mat3 R = Mat3::Identity();     // body -> world
  Vec3 rpy = Vec3::Zero();       // roll, pitch, yaw (ZYX)
  Vec3 omega_body = Vec3::Zero();
  Vec3 omega_world = Vec3::Zero();

  Vec12 q = Vec12::Zero();
  Vec12 dq = Vec12::Zero();

  // Foot points (kin::footPosition), per leg.
  std::array<Vec3, kNumLegs> foot_body{};   // base frame, relative to base origin
  std::array<Vec3, kNumLegs> foot_world{};  // world

  bool valid = false;
};

// How much the estimator may trust each foot to be planted: 1 = firmly in stance,
// 0 = in the air. Comes from the controller's gait (ramped at touch-down/lift-off).
struct ContactEstimate {
  std::array<double, kNumLegs> trust{1.0, 1.0, 1.0, 1.0};
};

// Turns raw sensor data into RobotState.
class StateEstimator {
 public:
  virtual ~StateEstimator() = default;
  virtual RobotState update(const LowStateData& low, const std::optional<BaseGroundTruth>& gt,
                            const ContactEstimate& contact) = 0;
};

// Simulation only: base position/velocity straight from the simulator.
class GroundTruthEstimator : public StateEstimator {
 public:
  RobotState update(const LowStateData& low, const std::optional<BaseGroundTruth>& gt,
                    const ContactEstimate& contact) override;
};

// IMU + leg kinematics linear Kalman filter (Bledt et al., IROS 2018, after Bloesch
// et al., RSS 2012). Orientation and angular velocity come from the IMU as they are;
// the filter estimates base position/velocity and the four foot positions:
//
//   x = [p, v, p_foot_FR, p_foot_FL, p_foot_RR, p_foot_RL]   (18, world frame)
//   predict:  p += v dt,  v += (R a_imu + g) dt,  feet move only by rolling
//   measure, per leg (weighted by contact trust):
//     p_foot - p     = R FK(q)                         (leg kinematics)
//     v              = -R (w x FK(q) + J(q) dq) + v_roll (foot not slipping)
//     p_foot.z       = foot radius                      (flat ground)
// The foot point is the centre of the ball foot. Rolling without slipping, it moves
// with v_roll = r (w_calf x z) (w_calf: calf angular velocity, world frame).
//
// A foot with low trust gets large process and measurement noise, so it follows its
// leg freely and does not drag the body estimate. Velocity and height are observable;
// horizontal position drifts like any odometry (the MPC only uses it relatively).
class KalmanEstimator : public StateEstimator {
 public:
  explicit KalmanEstimator(const Config& config);

  RobotState update(const LowStateData& low, const std::optional<BaseGroundTruth>& gt,
                    const ContactEstimate& contact) override;
  void reset() { initialized_ = false; }

 private:
  static constexpr int kN = 18, kM = 28;
  using StateVec = Eigen::Matrix<double, kN, 1>;
  using StateMat = Eigen::Matrix<double, kN, kN>;

  void initialize(const RobotState& s, const std::optional<BaseGroundTruth>& gt);

  const Config& config_;
  bool initialized_ = false;
  StateVec x_ = StateVec::Zero();
  StateMat P_ = StateMat::Identity();
};

// ZYX roll-pitch-yaw of a rotation matrix.
Vec3 rotationToRpy(const Mat3& R);

}  // namespace go2
