#include "go2_mpc/robot_interface.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/robot/channel/channel_factory.hpp>

namespace go2 {
namespace {

using LowCmdMsg = unitree_go::msg::dds_::LowCmd_;
using LowStateMsg = unitree_go::msg::dds_::LowState_;
using SportStateMsg = unitree_go::msg::dds_::SportModeState_;

// Values the Go2 motor driver reads as "no position / velocity target".
constexpr float kPosStop = 2.146e9f;
constexpr float kVelStop = 16000.0f;

double steadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

RobotInterface::RobotInterface(const Config& config) : config_(config) {
  unitree::robot::ChannelFactory::Instance()->Init(config_.robot.domain_id,
                                                   config_.robot.network);

  // Header, level flag and per-motor mode as in unitree_sdk2's go2 low-level example.
  lowcmd_msg_.head()[0] = 0xFE;
  lowcmd_msg_.head()[1] = 0xEF;
  lowcmd_msg_.level_flag() = 0xFF;
  lowcmd_msg_.gpio() = 0;
  for (int i = 0; i < kNumMotorSlots; ++i) {
    auto& m = lowcmd_msg_.motor_cmd()[i];
    m.mode() = 0x01;  // servo (FOC) mode
    m.q() = kPosStop;
    m.dq() = kVelStop;
    m.kp() = 0.0f;
    m.kd() = 0.0f;
    m.tau() = 0.0f;
  }
}

void RobotInterface::start() {
  // Listen on rt/lowcmd briefly: if anything arrives, another controller owns it.
  {
    bool other_writer = false;
    auto probe = std::make_shared<unitree::robot::ChannelSubscriber<LowCmdMsg>>("rt/lowcmd");
    probe->InitChannel([&other_writer](const void*) { other_writer = true; });
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    probe->CloseChannel();
    if (other_writer) {
      throw std::runtime_error(
          "another process is publishing rt/lowcmd (go2_ctrl? robot sport mode?); stop it first");
    }
  }

  lowcmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<LowCmdMsg>>("rt/lowcmd");
  lowcmd_pub_->InitChannel();

  lowstate_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<LowStateMsg>>("rt/lowstate");
  lowstate_sub_->InitChannel([this](const void* msg) { onLowState(msg); }, 1);

  if (config_.robot.use_ground_truth) {
    sport_sub_ =
        std::make_shared<unitree::robot::ChannelSubscriber<SportStateMsg>>("rt/sportmodestate");
    sport_sub_->InitChannel([this](const void* msg) { onSportState(msg); }, 1);
  }
}

bool RobotInterface::waitForLowState(double timeout_s) const {
  const double t0 = steadyNow();
  while (steadyNow() - t0 < timeout_s) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (lowstate_received_) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

LowStateData RobotInterface::lowState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return lowstate_;
}

std::optional<BaseGroundTruth> RobotInterface::groundTruth() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!ground_truth_) return std::nullopt;
  // rt/sportmodestate reports the IMU site; move it to the base origin:
  //   p_base = p_imu - R r,   v_base = v_imu - R (w_body x r)
  BaseGroundTruth gt = *ground_truth_;
  const Mat3 R = lowstate_.quat.toRotationMatrix();
  gt.pos -= R * kImuOffset;
  gt.vel -= R * lowstate_.gyro.cross(kImuOffset);
  return gt;
}

double RobotInterface::lowStateAge() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!lowstate_received_) return 1e9;
  return steadyNow() - lowstate_.stamp;
}

JointCommand RobotInterface::send(const JointCommand& cmd) {
  JointCommand sent = cmd;
  sent.tau = cmd.tau.cwiseMax(-config_.safety.tau_max).cwiseMin(config_.safety.tau_max);

  for (int i = 0; i < kNumJoints; ++i) {
    auto& m = lowcmd_msg_.motor_cmd()[i];
    m.mode() = 0x01;
    m.q() = static_cast<float>(sent.q[i]);
    m.dq() = static_cast<float>(sent.dq[i]);
    m.kp() = static_cast<float>(sent.kp[i]);
    m.kd() = static_cast<float>(sent.kd[i]);
    m.tau() = static_cast<float>(sent.tau[i]);
  }
  // The real robot drops a LowCmd whose CRC is wrong; the simulator ignores it.
  lowcmd_msg_.crc() =
      crc32_core(reinterpret_cast<uint32_t*>(&lowcmd_msg_), (sizeof(LowCmdMsg) >> 2) - 1);
  lowcmd_pub_->Write(lowcmd_msg_);
  return sent;
}

void RobotInterface::onLowState(const void* msg) {
  const auto& s = *static_cast<const LowStateMsg*>(msg);
  LowStateData d;
  for (int i = 0; i < kNumJoints; ++i) {
    const auto& m = s.motor_state()[i];
    d.q[i] = m.q();
    d.dq[i] = m.dq();
    d.tau_est[i] = m.tau_est();
  }
  const auto& imu = s.imu_state();
  d.quat = Eigen::Quaterniond(imu.quaternion()[0], imu.quaternion()[1], imu.quaternion()[2],
                              imu.quaternion()[3]);
  for (int i = 0; i < 3; ++i) {
    d.rpy[i] = imu.rpy()[i];
    d.gyro[i] = imu.gyroscope()[i];
    d.acc[i] = imu.accelerometer()[i];
  }
  std::copy(s.wireless_remote().begin(), s.wireless_remote().end(), d.remote.begin());
  d.tick = s.tick();
  d.stamp = steadyNow();

  std::lock_guard<std::mutex> lock(state_mutex_);
  lowstate_ = d;
  lowstate_received_ = true;
}

void RobotInterface::onSportState(const void* msg) {
  const auto& s = *static_cast<const SportStateMsg*>(msg);
  BaseGroundTruth gt;
  for (int i = 0; i < 3; ++i) {
    gt.pos[i] = s.position()[i];
    gt.vel[i] = s.velocity()[i];
  }
  gt.stamp = steadyNow();

  std::lock_guard<std::mutex> lock(state_mutex_);
  ground_truth_ = gt;
}

}  // namespace go2
