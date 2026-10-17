#pragma once

#include <random>

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Simulation only: makes the simulator's perfect sensors look like real ones before
// the estimators see them, so the Kalman filter is tested under realistic conditions.
//
//   gyro  += bias + N(0, gyro_std)            acc += bias + N(0, acc_std)
//   orientation *= small rotation (constant roll/pitch offset + N(0, attitude_std))
//   dq    += N(0, dq_std)
//
// All standard deviations and biases are multiplied by `scale` (0 = off).
class SimNoise {
 public:
  SimNoise(const Config::SimNoise& config, double scale);

  bool enabled() const { return scale_ > 0.0; }
  void apply(LowStateData& low);

 private:
  const Config::SimNoise& config_;
  double scale_;
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
};

}  // namespace go2
