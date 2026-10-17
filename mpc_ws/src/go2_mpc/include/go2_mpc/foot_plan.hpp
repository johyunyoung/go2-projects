#pragma once

#include <array>

#include "go2_mpc/types.hpp"

namespace go2 {

// What the debug stream shows about the feet (swing test, walking).
struct FootPlan {
  static constexpr int kCurvePoints = 10;
  std::array<bool, kNumLegs> stance{};
  std::array<Vec3, kNumLegs> des_world{};  // desired foot position, world frame
  // Sampled swing curve per leg (world frame); a stance leg repeats its point.
  std::array<std::array<Vec3, kCurvePoints>, kNumLegs> curve{};
};

}  // namespace go2
