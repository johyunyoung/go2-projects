#include "go2_mpc/remote.hpp"

#include <algorithm>
#include <cstring>

namespace go2 {

void Remote::update(const std::array<uint8_t, 40>& raw) {
  unitree::common::REMOTE_DATA_RX key;
  static_assert(sizeof(key) == 40, "REMOTE_DATA_RX layout changed");
  std::memcpy(&key, raw.data(), sizeof(key));
  joystick_.extract(key);

  const bool lt = joystick_.LT.pressed;
  events_.stand = lt && joystick_.A.on_pressed;
  events_.passive = lt && joystick_.B.on_pressed;
  events_.run = joystick_.start.on_pressed;

  axes_.lx = joystick_.lx();
  axes_.ly = joystick_.ly();
  axes_.rx = joystick_.rx();
  axes_.ry = joystick_.ry();

  // Same convention as go2_ctrl: forward = ly, left = -lx, counter-clockwise = -rx.
  const double s = config_.scale;
  velocity_.vx = std::clamp(s * joystick_.ly(), -config_.vx_max, config_.vx_max);
  velocity_.vy = std::clamp(-s * joystick_.lx(), -config_.vy_max, config_.vy_max);
  velocity_.wz = std::clamp(-s * joystick_.rx(), -config_.wz_max, config_.wz_max);
}

}  // namespace go2
