#include <gtest/gtest.h>

#include "go2_mpc/gait_scheduler.hpp"
#include "go2_mpc/swing_trajectory.hpp"

using namespace go2;

namespace {
const Vec3 kP0(0.10, -0.15, 0.00);
const Vec3 kPf(0.18, -0.13, 0.02);
constexpr double kHeight = 0.08, kT = 0.15;
}  // namespace

TEST(SwingTrajectory, EndPointsAndApex) {
  const SwingTrajectory s(kP0, kPf, kHeight, kT);
  EXPECT_LT((s.evaluate(0.0).pos - kP0).norm(), 1e-12);
  EXPECT_LT((s.evaluate(kT).pos - kPf).norm(), 1e-12);
  EXPECT_NEAR(s.evaluate(kT / 2).pos.z(), 0.02 + kHeight, 1e-12);  // apex above the higher end
  EXPECT_LT(s.evaluate(0.0).vel.norm(), 1e-12);                   // lift-off at rest
  EXPECT_LT(s.evaluate(kT).vel.norm(), 1e-12);                    // touch-down at rest
  EXPECT_NEAR(s.evaluate(kT / 2).vel.z(), 0.0, 1e-12);            // apex: no vertical speed
  // Clamped outside [0, T].
  EXPECT_LT((s.evaluate(-1.0).pos - kP0).norm(), 1e-12);
  EXPECT_LT((s.evaluate(2.0).pos - kPf).norm(), 1e-12);
}

TEST(SwingTrajectory, DerivativesMatchFiniteDifferences) {
  const SwingTrajectory s(kP0, kPf, kHeight, kT);
  constexpr double h = 1e-6;
  for (double t = 0.005; t < kT - 0.005; t += 0.005) {
    if (std::abs(t - kT / 2) < 2 * h) continue;  // acceleration jumps at the apex
    const auto p = s.evaluate(t);
    const Vec3 v_fd = (s.evaluate(t + h).pos - s.evaluate(t - h).pos) / (2 * h);
    const Vec3 a_fd = (s.evaluate(t + h).vel - s.evaluate(t - h).vel) / (2 * h);
    EXPECT_LT((p.vel - v_fd).norm(), 1e-6) << "t " << t;
    EXPECT_LT((p.acc - a_fd).norm(), 1e-3) << "t " << t;
  }
}

TEST(SwingTrajectory, ContinuousAcrossTheApex) {
  const SwingTrajectory s(kP0, kPf, kHeight, kT);
  const double e = 1e-9;
  const auto a = s.evaluate(kT / 2 - e), b = s.evaluate(kT / 2 + e);
  EXPECT_LT((a.pos - b.pos).norm(), 1e-8);
  EXPECT_LT((a.vel - b.vel).norm(), 1e-6);
}

TEST(SwingTrajectory, RetargetKeepsTheStart) {
  SwingTrajectory s(kP0, kPf, kHeight, kT);
  s.setTarget(kPf + Vec3(0.05, 0, 0));
  EXPECT_LT((s.evaluate(0.0).pos - kP0).norm(), 1e-12);
  EXPECT_NEAR(s.evaluate(kT).pos.x(), kPf.x() + 0.05, 1e-12);
}

TEST(GaitScheduler, TrotPairsAlternate) {
  GaitScheduler g({0.3, 0.5, {0.0, 0.5, 0.5, 0.0}});
  g.start(1.0);
  for (double t = 1.0; t < 2.0; t += 0.01) {
    // Diagonal pairs move together and opposite to the other pair.
    EXPECT_EQ(g.inContact(FR, t), g.inContact(RL, t));
    EXPECT_EQ(g.inContact(FL, t), g.inContact(RR, t));
    EXPECT_NE(g.inContact(FR, t), g.inContact(FL, t));
  }
  const auto fr = g.leg(FR, 1.0 + 0.075);  // half-way through FR's stance
  EXPECT_TRUE(fr.stance);
  EXPECT_NEAR(fr.progress, 0.5, 1e-9);
  EXPECT_NEAR(fr.remaining, 0.075, 1e-9);
  const auto fl = g.leg(FL, 1.0 + 0.03);  // FL starts in swing
  EXPECT_FALSE(fl.stance);
  EXPECT_NEAR(fl.progress, 0.2, 1e-9);
  EXPECT_NEAR(fl.remaining, 0.12, 1e-9);
  EXPECT_DOUBLE_EQ(g.stanceTime(), 0.15);
  EXPECT_DOUBLE_EQ(g.swingTime(), 0.15);
}
