#include <gtest/gtest.h>

#include "go2_mpc/config.hpp"
#include "go2_mpc/kinematics.hpp"
#include "go2_mpc/sim_noise.hpp"
#include "go2_mpc/state_estimator.hpp"

using namespace go2;

namespace {

// Kinematic robot: the base follows p(t) = p0 + v t (yaw only), the feet stay where
// they were planted. Produces the sensor data the robot would report.
class FakeRobot {
 public:
  FakeRobot(const Vec3& p0, const Vec3& v, double yaw) : p0_(p0), v_(v), yaw_(yaw) {
    const Vec3 nominal_q(0.0, 0.8, -1.5);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      feet_[leg] = p0 + R() * kin::footPosition(leg, nominal_q);
      feet_[leg].z() = kin::kFootRadius;
    }
  }

  Mat3 R() const { return Eigen::AngleAxisd(yaw_, Vec3::UnitZ()).toRotationMatrix(); }
  Vec3 pos(double t) const { return p0_ + v_ * t; }

  // Joint angles placing the planted feet, joint velocities by finite difference.
  LowStateData sense(double t, double dt, const Vec3& acc_bias = Vec3::Zero()) const {
    LowStateData d;
    const Vec12 q = joints(t), q_prev = joints(t - dt);
    d.q = q;
    d.dq = (q - q_prev) / dt;
    d.quat = Eigen::Quaterniond(R());
    d.gyro.setZero();
    d.acc = R().transpose() * Vec3(0.0, 0.0, 9.81) + acc_bias;  // at rest / constant velocity
    return d;
  }

  Vec3 foot(int leg) const { return feet_[leg]; }
  void moveFoot(int leg, const Vec3& p) { feet_[leg] = p; }

 private:
  Vec12 joints(double t) const {
    Vec12 q;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      Vec3 ql;
      kin::inverse(leg, R().transpose() * (feet_[leg] - pos(t)), ql);
      q.segment<3>(3 * leg) = ql;
    }
    return q;
  }

  Vec3 p0_, v_;
  double yaw_;
  std::array<Vec3, kNumLegs> feet_;
};

// Real-robot setting (no ground truth). The fake robot's feet are fixed points (no
// rolling ball), so rolling is off here; the rolling model is checked in the simulator.
Config testConfig() {
  Config c;
  c.control.dt = 0.002;
  c.robot.use_ground_truth = false;
  c.robot.estimator = "kalman";
  c.estimator.foot_rolling = false;
  return c;
}

}  // namespace

TEST(KalmanEstimator, StandingStillConverges) {
  const Config c = testConfig();
  KalmanEstimator kf(c);
  FakeRobot robot(Vec3(0.0, 0.0, 0.30), Vec3::Zero(), 0.4);
  RobotState s;
  for (int i = 0; i < 1000; ++i) s = kf.update(robot.sense(i * 0.002, 0.002), std::nullopt, {});
  EXPECT_LT(s.vel.norm(), 1e-3);
  EXPECT_NEAR(s.pos.z(), 0.30, 1e-3);
  for (int leg = 0; leg < kNumLegs; ++leg) EXPECT_NEAR(s.foot_world[leg].z(), kin::kFootRadius, 1e-3);
}

TEST(KalmanEstimator, ConvergesToTheWalkingSpeed) {
  const Config c = testConfig();
  KalmanEstimator kf(c);
  const Vec3 v(0.5, -0.2, 0.0);
  FakeRobot robot(Vec3(0.0, 0.0, 0.30), v, 0.0);
  RobotState s;
  // Starts believing the robot is at rest; feet stay reachable for ~0.25 s.
  double t_converged = -1.0;
  for (int i = 1; i <= 125; ++i) {
    const double t = i * 0.002;
    s = kf.update(robot.sense(t, 0.002), std::nullopt, {});
    if (t_converged < 0 && (s.vel - v).norm() < 0.02) t_converged = t;
  }
  std::printf("[ info ] velocity within 0.02 m/s after %.3f s\n", t_converged);
  EXPECT_GT(t_converged, 0.0);
  EXPECT_LT(t_converged, 0.1);
  EXPECT_LT((s.vel - v).norm(), 0.01);
  EXPECT_NEAR(s.pos.z(), 0.30, 2e-3);
}

TEST(KalmanEstimator, LegOdometryBoundsAccelerometerBias) {
  const Config c = testConfig();
  KalmanEstimator kf(c);
  FakeRobot robot(Vec3(0.0, 0.0, 0.30), Vec3::Zero(), 0.0);
  RobotState s;
  // 0.3 m/s^2 bias: integrated alone it would be 3 m/s of error after 10 s.
  double worst = 0.0;
  for (int i = 1; i <= 5000; ++i) {
    s = kf.update(robot.sense(i * 0.002, 0.002, Vec3(0.3, 0.0, 0.3)), std::nullopt, {});
    worst = std::max(worst, s.vel.norm());
  }
  std::printf("[ info ] 10 s with a 0.3 m/s^2 bias: worst |v| %.4f m/s, z error %.4f m\n", worst,
              s.pos.z() - 0.30);
  EXPECT_LT(worst, 0.05);
  EXPECT_NEAR(s.pos.z(), 0.30, 0.01);
}

TEST(KalmanEstimator, UntrustedFootDoesNotDragTheBody) {
  const Config c = testConfig();
  KalmanEstimator kf(c);
  FakeRobot robot(Vec3(0.0, 0.0, 0.30), Vec3::Zero(), 0.0);
  ContactEstimate contact;
  contact.trust[FL] = 0.0;  // FL in the air, swinging around
  RobotState s;
  for (int i = 1; i <= 1000; ++i) {
    const double t = i * 0.002;
    const Vec3 base_fl = kin::footPosition(FL, Vec3(0.0, 0.8, -1.5)) + Vec3(0, 0, 0.30);
    robot.moveFoot(FL, base_fl + Vec3(0.05 * std::sin(20 * t), 0.0, 0.06 + 0.04 * std::sin(20 * t)));
    s = kf.update(robot.sense(t, 0.002), std::nullopt, contact);
  }
  EXPECT_LT(s.vel.norm(), 0.01);
  EXPECT_NEAR(s.pos.z(), 0.30, 2e-3);
}

// Typical MEMS IMU noise and biases (SimNoise at scale 1): the estimate must stay
// finite and close to the truth.
TEST(KalmanEstimator, StaysAccurateWithSensorNoise) {
  const Config c = testConfig();
  KalmanEstimator kf(c);
  SimNoise noise(c.sim_noise, 1.0);
  FakeRobot robot(Vec3(0.0, 0.0, 0.30), Vec3::Zero(), 0.0);
  RobotState s;
  double worst = 0.0;
  for (int i = 1; i <= 2500; ++i) {
    LowStateData low = robot.sense(i * 0.002, 0.002);
    noise.apply(low);
    s = kf.update(low, std::nullopt, {});
    ASSERT_TRUE(s.vel.allFinite() && s.pos.allFinite()) << "diverged at step " << i;
    if (i > 250) worst = std::max(worst, s.vel.norm());
  }
  std::printf("[ info ] 5 s standing with sensor noise: worst |v| %.4f m/s, z %.4f m\n", worst, s.pos.z());
  EXPECT_LT(worst, 0.05);
  EXPECT_NEAR(s.pos.z(), 0.30, 0.01);
}
