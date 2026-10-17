#pragma once

#include <array>
#include <vector>

#include "go2_mpc/types.hpp"

namespace go2 {

// Convex single-rigid-body MPC (Di Carlo et al., IROS 2018), solved with HPIPM.
//
// State  x = [rpy, p_com, omega_world, v_com]  (12, world frame)
// Input  u = ground reaction forces of the legs in contact at that stage (3 per leg)
//
//   rpy'   = Rz(yaw)^T omega            (small roll/pitch)
//   p'     = v
//   omega' = I_w^-1 sum r_i x f_i,       I_w = Rz I_body Rz^T,  r_i = foot_i - p_com
//   v'     = sum f_i / m + g
//
// discretised with forward Euler. Each stage only carries the forces of the legs in
// contact, so a swing leg has no input at all (no zero-force constraints needed).
// Yaw and lever arms r_i at stage k come from the current state shifted by the
// reference's motion since stage 0: x0 + (x_ref[k] - x_ref[0]).
class SrbMpc {
 public:
  static constexpr int kNx = 12;
  using StateVec = Eigen::Matrix<double, kNx, 1>;

  struct Params {
    int horizon = 15;
    double dt = 0.02;
    double mass = 15.206;
    Mat3 inertia = Mat3::Identity();  // about the CoM, body frame
    double gravity = 9.81;
    double mu = 0.35;
    double fz_min = 10.0;
    double fz_max = 150.0;
    StateVec q_weights = StateVec::Ones();
    double r_weight = 1e-6;
    int max_iter = 30;
    double tol = 1e-6;
  };

  struct Problem {
    StateVec x0 = StateVec::Zero();
    std::vector<StateVec> x_ref;                         // horizon + 1 (cost from index 1)
    std::vector<std::array<bool, kNumLegs>> contact;     // horizon (stages 0..N-1)
    std::vector<std::array<Vec3, kNumLegs>> foot_world;  // horizon, foot positions
  };

  struct Solution {
    bool ok = false;
    int status = -1;      // HPIPM status (0 = success)
    int iterations = 0;
    double solve_ms = 0.0;
    Vec12 f0 = Vec12::Zero();          // first-stage forces, SDK leg order, world frame
    std::vector<Vec12> f;              // forces of every stage (horizon), zero for swing legs
    std::vector<StateVec> x_pred;      // horizon + 1
  };

  explicit SrbMpc(const Params& params);
  ~SrbMpc();
  SrbMpc(const SrbMpc&) = delete;
  SrbMpc& operator=(const SrbMpc&) = delete;

  Solution solve(const Problem& problem);
  const Params& params() const { return params_; }

 private:
  struct Workspace;  // HPIPM structures + reusable memory
  Params params_;
  Workspace* ws_;
};

}  // namespace go2
