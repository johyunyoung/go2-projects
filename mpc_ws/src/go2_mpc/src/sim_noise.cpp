#include "go2_mpc/sim_noise.hpp"

namespace go2 {

SimNoise::SimNoise(const Config::SimNoise& config, double scale)
    : config_(config), scale_(scale), rng_(config.seed) {}

void SimNoise::apply(LowStateData& low) {
  if (!enabled()) return;
  const auto& c = config_;
  const double k = scale_;
  // Explicit Vec3 return: with `auto` the lambda would return an Eigen expression that
  // refers to a destroyed temporary.
  auto n3 = [&](double std) -> Vec3 { return Vec3(normal_(rng_), normal_(rng_), normal_(rng_)) * (k * std); };

  low.gyro += k * c.gyro_bias + n3(c.gyro_std);
  low.acc += k * c.acc_bias + n3(c.acc_std);

  // Attitude error: a constant roll/pitch offset (the IMU's own attitude estimate is
  // never exact) plus noise, applied in the body frame.
  const Vec3 err = k * Vec3(c.attitude_bias[0], c.attitude_bias[1], 0.0) + n3(c.attitude_std);
  const double angle = err.norm();
  if (angle > 0.0) low.quat = (low.quat * Eigen::Quaterniond(Eigen::AngleAxisd(angle, err / angle))).normalized();

  for (int i = 0; i < kNumJoints; ++i) low.dq[i] += k * c.dq_std * normal_(rng_);
}

}  // namespace go2
