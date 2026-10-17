#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// The only place that talks to unitree_sdk2. Switching between unitree_mujoco and the
// real robot is a matter of Config::robot.network ("lo" vs the robot NIC).
class RobotInterface {
 public:
  explicit RobotInterface(const Config& config);

  // Fails if another process already publishes rt/lowcmd (e.g. go2_ctrl, or the
  // robot's own sport-mode service), since two writers would fight over the motors.
  void start();

  // Blocks until rt/lowstate arrives or the timeout passes. Returns false on timeout.
  bool waitForLowState(double timeout_s) const;

  LowStateData lowState() const;
  std::optional<BaseGroundTruth> groundTruth() const;  // empty if never received
  double lowStateAge() const;  // seconds since the last rt/lowstate

  // Clamps torque to the configured limits, fills rt/lowcmd, computes the CRC and
  // writes it. Returns the command actually sent (after clamping).
  JointCommand send(const JointCommand& cmd);

 private:
  void onLowState(const void* msg);
  void onSportState(const void* msg);

  const Config& config_;

  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_pub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_sub_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_sub_;

  unitree_go::msg::dds_::LowCmd_ lowcmd_msg_{};

  mutable std::mutex state_mutex_;
  LowStateData lowstate_;
  bool lowstate_received_ = false;
  std::optional<BaseGroundTruth> ground_truth_;
};

}  // namespace go2
