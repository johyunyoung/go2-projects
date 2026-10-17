#pragma once

#include <cstdint>
#include <optional>

#include "go2_mpc/config.hpp"
#include "go2_mpc/mpc_controller.hpp"
#include "go2_mpc/remote.hpp"
#include "go2_mpc/stand_controller.hpp"
#include "go2_mpc/swing_test_controller.hpp"
#include "go2_mpc/state_estimator.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

enum class Mode : uint8_t { Passive = 0, Stand = 1, Mpc = 2, SwingTest = 3 };

const char* modeName(Mode mode);

// Everything a mode needs to compute one command.
struct ControlInput {
  double t = 0.0;  // controller time since start [s]
  const LowStateData* low = nullptr;
  const RobotState* state = nullptr;
  const RemoteEvents* events = nullptr;
  const PoseCommand* pose = nullptr;
  const VelocityCommand* velocity = nullptr;
  bool lowstate_stale = false;
};

// Mode switching and per-mode command generation.
//
//   Passive --(LT+A)--> Stand --(start, once standing)--> Mpc
//   Mpc --(LT+A, or MPC watchdog)--> Stand (hold the pose, no tuck)
//   any --(LT+B, rt/lowstate stale, or tilt > max_tilt in Mpc)--> Passive
//   With swing_test.enable, start goes to SwingTest instead of Mpc (base welded in the
//   air, M4); LT+A leaves it for Stand.
class Fsm {
 public:
  explicit Fsm(const Config& config)
      : config_(config), stand_(config), mpc_(config), swing_test_(config) {}

  JointCommand step(const ControlInput& in);
  Mode mode() const { return mode_; }
  MpcStatus mpcStatus(double t) const { return mpc_.status(t); }
  // Contact trust for the state estimator (from the gait in Mpc mode).
  ContactEstimate contact(double t) const {
    if (mode_ == Mode::Mpc) return mpc_.contact(t);
    if (mode_ == Mode::SwingTest) return ContactEstimate{{0.0, 0.0, 0.0, 0.0}};  // feet in the air
    return ContactEstimate{};  // Passive / Stand: feet on the ground
  }
  const FootPlan* footPlan() const {
    if (mode_ == Mode::SwingTest) return &swing_test_.plan();
    if (mode_ == Mode::Mpc) return &mpc_.plan();
    return nullptr;
  }

 private:
  void enter(Mode next, const ControlInput& in, const char* reason = nullptr);
  JointCommand passiveCommand() const;
  void reportStand(const ControlInput& in) const;

  const Config& config_;
  Mode mode_ = Mode::Passive;
  StandController stand_;
  MpcController mpc_;
  SwingTestController swing_test_;
};

}  // namespace go2
