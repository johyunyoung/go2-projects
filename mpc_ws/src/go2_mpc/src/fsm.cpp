#include "go2_mpc/fsm.hpp"

#include <cmath>
#include <cstdio>

namespace go2 {

const char* modeName(Mode mode) {
  switch (mode) {
    case Mode::Passive: return "Passive";
    case Mode::Stand: return "Stand";
    case Mode::Mpc: return "Mpc";
    case Mode::SwingTest: return "SwingTest";
  }
  return "?";
}

JointCommand Fsm::step(const ControlInput& in) {
  const RemoteEvents& ev = *in.events;
  const RobotState& s = *in.state;

  // --- transitions ---
  if (in.lowstate_stale && mode_ != Mode::Passive) {
    enter(Mode::Passive, in, "rt/lowstate stale");
  } else if (ev.passive && mode_ != Mode::Passive) {
    enter(Mode::Passive, in, "LT+B");
  } else if (ev.stand && mode_ == Mode::Passive) {
    enter(Mode::Stand, in, "LT+A");
  } else if (ev.stand && (mode_ == Mode::Mpc || mode_ == Mode::SwingTest)) {
    enter(Mode::Stand, in, "LT+A");
  } else if (ev.run && mode_ == Mode::Stand) {
    if (!stand_.finished(in.t)) {
      std::printf("[fsm] start ignored: still standing up\n");
    } else if (!s.valid) {
      std::printf("[fsm] start ignored: no base state (ground truth missing)\n");
    } else {
      enter(config_.swing_test.enable ? Mode::SwingTest : Mode::Mpc, in, "start");
    }
  } else if (mode_ == Mode::Mpc) {
    if (std::abs(s.rpy.x()) > config_.safety.max_tilt || std::abs(s.rpy.y()) > config_.safety.max_tilt) {
      enter(Mode::Passive, in, "tilt limit");
    } else if (mpc_.timedOut(in.t)) {
      enter(Mode::Stand, in, "MPC watchdog");
    }
  }

  // --- command ---
  switch (mode_) {
    case Mode::Stand:
      if (stand_.reportDue(in.t)) reportStand(in);
      return stand_.step(in.t, s.q, s.R);
    case Mode::Mpc:
      return mpc_.step(in.t, s, *in.pose, *in.velocity);
    case Mode::SwingTest:
      return swing_test_.step(in.t, s);
    case Mode::Passive:
      break;
  }
  return passiveCommand();
}

void Fsm::enter(Mode next, const ControlInput& in, const char* reason) {
  std::printf("[fsm] t=%.3f  %s -> %s%s%s%s\n", in.t, modeName(mode_), modeName(next),
              reason ? " (" : "", reason ? reason : "", reason ? ")" : "");
  if (mode_ == Mode::Mpc) {
    const MpcStatus st = mpc_.status(in.t);
    std::printf("[mpc] stop: %lu solves, %lu failures\n", static_cast<unsigned long>(st.solves),
                static_cast<unsigned long>(st.failures));
    mpc_.stop();
  }
  const bool standing = mode_ == Mode::Mpc || mode_ == Mode::SwingTest;
  if (next == Mode::Stand) stand_.start(in.state->q, in.t, /*hold=*/standing);
  if (next == Mode::Mpc) mpc_.start(*in.state, in.t);
  if (next == Mode::SwingTest) swing_test_.start(*in.state, in.t);
  mode_ = next;
}

// Zero torque with joint damping only: the robot sinks down slowly instead of falling.
JointCommand Fsm::passiveCommand() const {
  JointCommand cmd;
  cmd.kd = config_.passive.kd;
  return cmd;
}

void Fsm::reportStand(const ControlInput& in) const {
  const Vec12 err = in.state->q - stand_.target();
  int worst = 0;
  err.cwiseAbs().maxCoeff(&worst);
  std::printf("[stand] holding: max |q - q_target| = %.4f rad (joint %d, %s), rms %.4f rad",
              std::abs(err[worst]), worst, kLegNames[worst / 3], std::sqrt(err.squaredNorm() / 12));
  if (in.state->valid) std::printf(", base height %.3f m", in.state->pos.z());
  std::printf("\n");
}

}  // namespace go2
