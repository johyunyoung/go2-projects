#include <array>
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "go2_mpc/kinematics.hpp"

#ifdef GO2_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif

using namespace go2;

namespace {

// Joint angles inside the Go2 working range (thigh limits differ front/back, so stay
// within the intersection), with the foot at least 5 cm below the hip in the leg
// plane: the branch kin::inverse assumes. (A leg swung forward with the knee fully
// folded can bring the foot up to hip height, where that branch is ambiguous; the
// controller never commands such postures. The tucked lying pose is ~10 cm below.)
Vec3 randomJoints(std::mt19937& rng) {
  std::uniform_real_distribution<double> hip(-0.8, 0.8), thigh(-0.4, 1.8), calf(-2.6, -0.9);
  while (true) {
    const Vec3 q(hip(rng), thigh(rng), calf(rng));
    const double z_plane = -kin::kThighLength * std::cos(q[1]) - kin::kCalfLength * std::cos(q[1] + q[2]);
    if (z_plane < -0.05) return q;
  }
}

}  // namespace

TEST(Kinematics, NominalStandIsSymmetric) {
  const Vec3 q(0.0, 0.8, -1.5);
  const Vec3 fr = kin::footPosition(FR, q), fl = kin::footPosition(FL, q);
  const Vec3 rr = kin::footPosition(RR, q), rl = kin::footPosition(RL, q);
  EXPECT_NEAR(fr.y(), -fl.y(), 1e-12);
  EXPECT_NEAR(fr.x(), rr.x() + 2 * kin::kHipX, 1e-12);
  EXPECT_NEAR(rr.y(), -rl.y(), 1e-12);
  // Foot below the hip by l1 cos(q2) + l2 cos(q2 + q3).
  EXPECT_NEAR(fr.z(), -(0.213 * std::cos(0.8) + 0.213 * std::cos(-0.7)), 1e-12);
}

TEST(Kinematics, JacobianMatchesFiniteDifference) {
  std::mt19937 rng(1);
  constexpr double h = 1e-6;
  for (int trial = 0; trial < 200; ++trial) {
    const int leg = trial % kNumLegs;
    const Vec3 q = randomJoints(rng);
    const Mat3 J = kin::footJacobian(leg, q);
    for (int j = 0; j < 3; ++j) {
      Vec3 dq = Vec3::Zero();
      dq[j] = h;
      const Vec3 fd = (kin::footPosition(leg, q + dq) - kin::footPosition(leg, q - dq)) / (2 * h);
      EXPECT_LT((J.col(j) - fd).norm(), 1e-8) << "leg " << leg << " joint " << j;
    }
  }
}

TEST(Kinematics, InverseRoundTrip) {
  std::mt19937 rng(2);
  for (int trial = 0; trial < 1000; ++trial) {
    const int leg = trial % kNumLegs;
    const Vec3 q = randomJoints(rng);
    Vec3 q_ik;
    ASSERT_TRUE(kin::inverse(leg, kin::footPosition(leg, q), q_ik));
    EXPECT_LT((q_ik - q).norm(), 1e-9) << "leg " << leg << " q " << q.transpose();
  }
}

TEST(Kinematics, InverseProjectsUnreachableTargets) {
  Vec3 q;
  // 1 m below the hip is out of reach (legs are 0.426 m): fully stretched leg.
  EXPECT_FALSE(kin::inverse(FL, kin::hipPosition(FL) + Vec3(0, 0.0955, -1.0), q));
  EXPECT_NEAR(q[2], 0.0, 1e-9);
  EXPECT_TRUE(q.allFinite());
}

#ifdef GO2_HAVE_MUJOCO
// Compares against MuJoCo's forward kinematics of the simulator's own model.
TEST(Kinematics, FootPositionMatchesMujoco) {
  char err[1000] = "";
  mjModel* m = mj_loadXML(GO2_MUJOCO_MODEL, nullptr, err, sizeof(err));
  ASSERT_NE(m, nullptr) << err;
  mjData* d = mj_makeData(m);

  const int base = mj_name2id(m, mjOBJ_BODY, "base_link");
  ASSERT_GE(base, 0);
  std::mt19937 rng(3);
  double worst = 0.0;
  for (int trial = 0; trial < 200; ++trial) {
    // Base at the origin, identity orientation: world frame == base frame.
    mj_resetData(m, d);
    for (int i = 0; i < 7; ++i) d->qpos[i] = (i == 3) ? 1.0 : 0.0;
    Vec12 q;
    for (int leg = 0; leg < kNumLegs; ++leg) q.segment<3>(3 * leg) = randomJoints(rng);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      for (int j = 0; j < 3; ++j) {
        static const char* names[] = {"hip", "thigh", "calf"};
        const std::string joint = std::string(kLegNames[leg]) + "_" + names[j] + "_joint";
        const int id = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
        ASSERT_GE(id, 0) << joint;
        d->qpos[m->jnt_qposadr[id]] = q[3 * leg + j];
      }
    }
    mj_kinematics(m, d);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const std::string body = std::string(kLegNames[leg]) + "_foot";
      const int id = mj_name2id(m, mjOBJ_BODY, body.c_str());
      ASSERT_GE(id, 0) << body;
      const Vec3 p_mj(d->xpos[3 * id], d->xpos[3 * id + 1], d->xpos[3 * id + 2]);
      const Vec3 p = kin::footPosition(leg, kin::legJoints(q, leg));
      worst = std::max(worst, (p - p_mj).norm());
      EXPECT_LT((p - p_mj).norm(), 1e-3) << body;  // plan criterion: 1 mm
    }
  }
  std::printf("[ info ] worst FK deviation from MuJoCo: %.2e m\n", worst);

  // Leg gravity torque vs MuJoCo's bias force at zero velocity, with a tilted base.
  double worst_tau = 0.0;
  for (int trial = 0; trial < 100; ++trial) {
    mj_resetData(m, d);
    const Eigen::Quaterniond qb = Eigen::Quaterniond(Eigen::AngleAxisd(0.3 * std::sin(trial), Vec3::UnitX()) *
                                                     Eigen::AngleAxisd(0.2 * std::cos(trial), Vec3::UnitY()));
    d->qpos[3] = qb.w(); d->qpos[4] = qb.x(); d->qpos[5] = qb.y(); d->qpos[6] = qb.z();
    Vec12 q;
    for (int leg = 0; leg < kNumLegs; ++leg) q.segment<3>(3 * leg) = randomJoints(rng);
    std::array<int, 12> dof{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
      for (int j = 0; j < 3; ++j) {
        static const char* names[] = {"hip", "thigh", "calf"};
        const std::string joint = std::string(kLegNames[leg]) + "_" + names[j] + "_joint";
        const int id = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
        d->qpos[m->jnt_qposadr[id]] = q[3 * leg + j];
        dof[3 * leg + j] = m->jnt_dofadr[id];
      }
    }
    mj_forward(m, d);
    const Vec3 g_body = qb.toRotationMatrix().transpose() * Vec3(m->opt.gravity[0], m->opt.gravity[1], m->opt.gravity[2]);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const Vec3 tau = kin::legGravityTorque(leg, kin::legJoints(q, leg), g_body);
      for (int j = 0; j < 3; ++j) {
        const double ref = d->qfrc_bias[dof[3 * leg + j]];
        worst_tau = std::max(worst_tau, std::abs(tau[j] - ref));
        EXPECT_NEAR(tau[j], ref, 1e-6) << kLegNames[leg] << " joint " << j;
      }
    }
  }
  std::printf("[ info ] worst leg gravity torque deviation from MuJoCo: %.2e Nm\n", worst_tau);

  // Leg mass matrix vs the leg-joint block of MuJoCo's joint-space inertia.
  std::vector<mjtNum> Mfull(m->nv * m->nv);
  double worst_M = 0.0;
  for (int trial = 0; trial < 100; ++trial) {
    mj_resetData(m, d);
    for (int i = 0; i < 7; ++i) d->qpos[i] = (i == 3) ? 1.0 : 0.0;
    Vec12 q;
    std::array<int, 12> dof{};
    for (int leg = 0; leg < kNumLegs; ++leg) {
      q.segment<3>(3 * leg) = randomJoints(rng);
      for (int j = 0; j < 3; ++j) {
        static const char* names[] = {"hip", "thigh", "calf"};
        const std::string joint = std::string(kLegNames[leg]) + "_" + names[j] + "_joint";
        const int id = mj_name2id(m, mjOBJ_JOINT, joint.c_str());
        d->qpos[m->jnt_qposadr[id]] = q[3 * leg + j];
        dof[3 * leg + j] = m->jnt_dofadr[id];
      }
    }
    mj_forward(m, d);
    mj_fullM(m, Mfull.data(), d->qM);
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const Mat3 M = kin::legMassMatrix(leg, kin::legJoints(q, leg));
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          const double ref = Mfull[dof[3 * leg + r] * m->nv + dof[3 * leg + c]];
          worst_M = std::max(worst_M, std::abs(M(r, c) - ref));
          EXPECT_NEAR(M(r, c), ref, 1e-6) << kLegNames[leg] << " M(" << r << "," << c << ")";
        }
      }
    }
  }
  std::printf("[ info ] worst leg mass matrix deviation from MuJoCo: %.2e\n", worst_M);
  mj_deleteData(d);
  mj_deleteModel(m);
}
#endif
