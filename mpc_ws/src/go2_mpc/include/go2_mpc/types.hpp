#pragma once

#include <array>
#include <cstdint>

#include <Eigen/Dense>

namespace go2 {

// Leg and joint order follow the SDK (rt/lowcmd, rt/lowstate) everywhere in this
// code base, so there is no index remapping anywhere: FR, FL, RR, RL, and within a
// leg hip (abduction), thigh, calf. Joint j of leg l is at index 3 * l + j.
enum Leg : int { FR = 0, FL = 1, RR = 2, RL = 3 };

constexpr int kNumLegs = 4;
constexpr int kNumJoints = 12;
constexpr int kNumMotorSlots = 20;  // LowCmd_/LowState_ carry 20 motor slots; Go2 uses 12

constexpr std::array<const char*, kNumLegs> kLegNames = {"FR", "FL", "RR", "RL"};

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Vec12 = Eigen::Matrix<double, kNumJoints, 1>;
using Mat3 = Eigen::Matrix3d;

// IMU position in the base frame (go2.xml site "imu", go2_description imu_joint).
// The simulator's ground-truth position/velocity are taken at this point.
inline const Vec3 kImuOffset{-0.02557, 0.0, 0.04232};

// Everything the controller reads from rt/lowstate in one tick.
struct LowStateData {
  Vec12 q = Vec12::Zero();
  Vec12 dq = Vec12::Zero();
  Vec12 tau_est = Vec12::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();  // body -> world
  Vec3 rpy = Vec3::Zero();
  Vec3 gyro = Vec3::Zero();  // body frame
  Vec3 acc = Vec3::Zero();   // body frame
  std::array<uint8_t, 40> remote{};  // raw wireless remote bytes
  uint32_t tick = 0;
  double stamp = 0.0;  // steady clock [s] when received
};

// Base position/velocity from rt/sportmodestate. unitree_mujoco fills it with the
// simulator's ground truth (velocity in world frame). On the real robot it only
// exists while sport mode runs, which low-level control has to switch off, so the
// controller must never require it outside simulation.
//
// RobotInterface converts the simulator's IMU-site values to the base origin, so
// pos/vel here are the base frame origin's position and velocity in world frame.
struct BaseGroundTruth {
  Vec3 pos = Vec3::Zero();
  Vec3 vel = Vec3::Zero();
  double stamp = 0.0;
};

// What the controller sends to rt/lowcmd. The motor driver (or the simulator)
// applies tau + kp * (q - q_meas) + kd * (dq - dq_meas).
struct JointCommand {
  Vec12 q = Vec12::Zero();
  Vec12 dq = Vec12::Zero();
  Vec12 kp = Vec12::Zero();
  Vec12 kd = Vec12::Zero();
  Vec12 tau = Vec12::Zero();
};

// Body-frame velocity command from the remote.
struct VelocityCommand {
  double vx = 0.0;  // forward [m/s]
  double vy = 0.0;  // left [m/s]
  double wz = 0.0;  // counter-clockwise yaw rate [rad/s]
};

}  // namespace go2
