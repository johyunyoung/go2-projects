#pragma once

#include <array>
#include <cstdint>

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Edge-triggered requests decoded from the remote in one tick.
struct RemoteEvents {
  bool stand = false;    // LT + A
  bool run = false;      // start
  bool passive = false;  // LT + B
};

// Stick positions in [-1, 1] (after the SDK's smoothing and dead zone).
struct RemoteAxes {
  double lx = 0.0, ly = 0.0, rx = 0.0, ry = 0.0;
};

// Decodes the 40 wireless-remote bytes of rt/lowstate. The real Go2 remote and the
// simulator's keyboard patch fill the same bytes, so this works unchanged on both:
//   keyboard 1 -> LT + A,  2 -> start,  0 -> LT + B
//   W/S -> ly,  A/D -> lx,  Q/E -> rx   (keyboard steps each axis by 0.1 per press)
class Remote {
 public:
  explicit Remote(const Config::Command& config) : config_(config) {}

  // Call exactly once per control tick: button edges are computed between calls.
  void update(const std::array<uint8_t, 40>& raw);

  const RemoteEvents& events() const { return events_; }
  const VelocityCommand& velocity() const { return velocity_; }
  const RemoteAxes& axes() const { return axes_; }

 private:
  const Config::Command& config_;
  unitree::common::UnitreeJoystick joystick_;
  RemoteEvents events_;
  VelocityCommand velocity_;
  RemoteAxes axes_;
};

}  // namespace go2
