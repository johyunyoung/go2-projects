#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "go2_mpc/kinematics.hpp"
#include "go2_mpc/srb_mpc.hpp"

using namespace go2;
using StateVec = SrbMpc::StateVec;

namespace {

SrbMpc::Params params() {
  SrbMpc::Params p;
  p.inertia << 0.188, 0.0, -0.020, 0.0, 0.499, 0.0, -0.020, 0.0, 0.532;
  p.q_weights << 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1;
  return p;
}

// Standing problem: CoM at (0, 0, h), all feet under the hips on the ground.
SrbMpc::Problem standing(const SrbMpc::Params& p, double h) {
  SrbMpc::Problem pb;
  StateVec ref = StateVec::Zero();
  ref[5] = h;
  pb.x0 = ref;
  pb.x_ref.assign(p.horizon + 1, ref);
  std::array<Vec3, kNumLegs> feet;
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 hip = kin::hipPosition(leg);
    feet[leg] = Vec3(hip.x(), hip.y() + kin::sideSign(leg) * kin::kHipLength, 0.0);
  }
  pb.foot_world.assign(p.horizon, feet);
  pb.contact.assign(p.horizon, {true, true, true, true});
  return pb;
}

double sumFz(const Vec12& f) { return f[2] + f[5] + f[8] + f[11]; }

void expectFrictionCone(const Vec12& f, double mu) {
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const Vec3 fi = f.segment<3>(3 * leg);
    if (fi.isZero()) continue;
    EXPECT_LE(std::abs(fi.x()), mu * fi.z() + 1e-6);
    EXPECT_LE(std::abs(fi.y()), mu * fi.z() + 1e-6);
  }
}

}  // namespace

TEST(SrbMpc, EquilibriumCarriesTheWeight) {
  const auto p = params();
  SrbMpc mpc(p);
  const auto sol = mpc.solve(standing(p, 0.30));
  ASSERT_TRUE(sol.ok) << "status " << sol.status;
  EXPECT_NEAR(sumFz(sol.f0), p.mass * p.gravity, 1.0);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_NEAR(sol.f0[3 * leg + 2], p.mass * p.gravity / 4, 2.0) << kLegNames[leg];
    EXPECT_NEAR(sol.f0[3 * leg + 0], 0.0, 0.5);
  }
  // Holding the reference means the prediction barely moves.
  EXPECT_NEAR(sol.x_pred.back()[5], 0.30, 2e-3);
}

TEST(SrbMpc, LowBodyIsPushedUp) {
  const auto p = params();
  SrbMpc mpc(p);
  auto pb = standing(p, 0.30);
  pb.x0[5] = 0.27;
  const auto sol = mpc.solve(pb);
  ASSERT_TRUE(sol.ok);
  EXPECT_GT(sumFz(sol.f0), p.mass * p.gravity + 5.0);
  EXPECT_GT(sol.x_pred.back()[5], 0.27);
  expectFrictionCone(sol.f0, p.mu);
}

TEST(SrbMpc, RollIsCorrectedByTheLowSide) {
  const auto p = params();
  SrbMpc mpc(p);
  auto pb = standing(p, 0.30);
  pb.x0[0] = 0.1;  // positive roll: left side up, right side down
  const auto sol = mpc.solve(pb);
  ASSERT_TRUE(sol.ok);
  // Negative roll torque needs more force on the right (y < 0) feet.
  EXPECT_GT(sol.f0[3 * FR + 2], sol.f0[3 * FL + 2]);
  EXPECT_GT(sol.f0[3 * RR + 2], sol.f0[3 * RL + 2]);
  EXPECT_LT(sol.x_pred.back()[0], 0.1);
}

TEST(SrbMpc, SwingLegsGetNoForce) {
  const auto p = params();
  SrbMpc mpc(p);
  auto pb = standing(p, 0.30);
  for (auto& c : pb.contact) c = {true, false, false, true};  // trot pair FR + RL
  const auto sol = mpc.solve(pb);
  ASSERT_TRUE(sol.ok);
  EXPECT_TRUE(sol.f0.segment<3>(3 * FL).isZero());
  EXPECT_TRUE(sol.f0.segment<3>(3 * RR).isZero());
  EXPECT_NEAR(sumFz(sol.f0), p.mass * p.gravity, 15.0);
  expectFrictionCone(sol.f0, p.mu);
}

TEST(SrbMpc, SolveTime) {
  const auto p = params();
  SrbMpc mpc(p);
  auto pb = standing(p, 0.30);
  pb.x0[5] = 0.28;
  std::vector<double> t;
  for (int i = 0; i < 500; ++i) t.push_back(mpc.solve(pb).solve_ms);
  std::sort(t.begin(), t.end());
  std::printf("[ info ] build + solve [ms]: median %.3f  p99 %.3f  max %.3f\n", t[250], t[495],
              t.back());
  EXPECT_LT(t[495], 2.0);  // plan criterion (p99 < 2 ms)
}

// Same, but paced like the controller: one solve every 20 ms.
TEST(SrbMpc, SolveTimePaced) {
  const auto p = params();
  SrbMpc mpc(p);
  auto pb = standing(p, 0.30);
  pb.x0[5] = 0.28;
  std::vector<double> t;
  for (int i = 0; i < 100; ++i) {
    t.push_back(mpc.solve(pb).solve_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::sort(t.begin(), t.end());
  std::printf("[ info ] paced 50 Hz, build + solve [ms]: median %.3f  p99 %.3f  max %.3f\n", t[50],
              t[98], t.back());
}
