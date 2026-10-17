// hpipm_smoke: checks the vendored HPIPM build and measures solve time for an SRB
// problem of the size the MPC will use (N = 15, 12 states, 12 inputs, 4 legs in
// stance, friction pyramid). The model here is a throwaway copy for timing only;
// the real one lives in srb_mpc (M3).
//
// Pass criteria: status 0, every force inside the friction pyramid, and a solve time
// far below the 20 ms MPC period. The start state is falling and 3 cm low, so the
// first-step vertical force is well above m*g while it recovers.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <Eigen/Dense>

extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
}

namespace {

constexpr int N = 15, NX = 12, NU = 12, NG = 16;
constexpr double kDt = 0.02, kMass = 15.2, kMu = 0.35, kG = 9.81;
constexpr double kFzMin = 10.0, kFzMax = 150.0;

using MatX = Eigen::Matrix<double, NX, NX>;
using MatXU = Eigen::Matrix<double, NX, NU>;
using VecX = Eigen::Matrix<double, NX, 1>;
using VecU = Eigen::Matrix<double, NU, 1>;

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}

}  // namespace

int main() {
  // ---- model: x = [rpy, p, omega, v], u = [f_FR, f_FL, f_RR, f_RL], yaw = 0 ----
  const Eigen::Matrix3d inertia = Eigen::Vector3d(0.107, 0.098, 0.024).asDiagonal();
  const Eigen::Vector3d feet[4] = {{0.19, -0.14, -0.30}, {0.19, 0.14, -0.30},
                                   {-0.19, -0.14, -0.30}, {-0.19, 0.14, -0.30}};
  MatX A = MatX::Identity();
  A.block<3, 3>(0, 6) = kDt * Eigen::Matrix3d::Identity();
  A.block<3, 3>(3, 9) = kDt * Eigen::Matrix3d::Identity();
  MatXU B = MatXU::Zero();
  for (int i = 0; i < 4; ++i) {
    B.block<3, 3>(6, 3 * i) = kDt * inertia.inverse() * skew(feet[i]);
    B.block<3, 3>(9, 3 * i) = kDt / kMass * Eigen::Matrix3d::Identity();
  }
  VecX b = VecX::Zero();
  b(11) = -kG * kDt;

  // ---- cost: track a 0.30 m stand from a perturbed start ----
  VecX qdiag;
  qdiag << 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1;
  MatX Q = qdiag.asDiagonal();
  Eigen::Matrix<double, NU, NU> R = 1e-6 * Eigen::Matrix<double, NU, NU>::Identity();
  VecX x_ref = VecX::Zero();
  x_ref(5) = 0.30;
  VecX qvec = -Q * x_ref;
  VecU rvec = VecU::Zero();

  VecX x0 = VecX::Zero();
  x0(0) = 0.10;  // roll
  x0(5) = 0.27;  // height
  x0(11) = -0.2; // falling

  // ---- constraints: fz box + friction pyramid (|fx|,|fy| <= mu fz) ----
  int idxbu[4] = {2, 5, 8, 11};
  double lbu[4], ubu[4];
  std::fill(lbu, lbu + 4, kFzMin);
  std::fill(ubu, ubu + 4, kFzMax);
  Eigen::Matrix<double, NG, NU> D = Eigen::Matrix<double, NG, NU>::Zero();
  for (int i = 0; i < 4; ++i) {
    const int r = 4 * i, c = 3 * i;
    D(r + 0, c + 0) = 1;  D(r + 0, c + 2) = -kMu;  //  fx - mu fz <= 0
    D(r + 1, c + 0) = -1; D(r + 1, c + 2) = -kMu;  // -fx - mu fz <= 0
    D(r + 2, c + 1) = 1;  D(r + 2, c + 2) = -kMu;
    D(r + 3, c + 1) = -1; D(r + 3, c + 2) = -kMu;
  }
  Eigen::Matrix<double, NG, NX> C = Eigen::Matrix<double, NG, NX>::Zero();
  std::vector<double> lg(NG, 0.0), ug(NG, 0.0), lg_mask(NG, 0.0);  // lower side disabled
  int idxbx[NX];
  for (int i = 0; i < NX; ++i) idxbx[i] = i;

  // ---- dimensions ----
  d_ocp_qp_dim dim;
  std::vector<char> dim_mem(d_ocp_qp_dim_memsize(N));
  d_ocp_qp_dim_create(N, &dim, dim_mem.data());
  for (int k = 0; k <= N; ++k) {
    d_ocp_qp_dim_set_nx(k, NX, &dim);
    d_ocp_qp_dim_set_nu(k, k < N ? NU : 0, &dim);
    d_ocp_qp_dim_set_nbx(k, k == 0 ? NX : 0, &dim);
    d_ocp_qp_dim_set_nbu(k, k < N ? 4 : 0, &dim);
    d_ocp_qp_dim_set_ng(k, k < N ? NG : 0, &dim);
    d_ocp_qp_dim_set_ns(k, 0, &dim);
  }

  // ---- qp ----
  d_ocp_qp qp;
  std::vector<char> qp_mem(d_ocp_qp_memsize(&dim));
  d_ocp_qp_create(&dim, &qp, qp_mem.data());
  for (int k = 0; k <= N; ++k) {
    d_ocp_qp_set_Q(k, Q.data(), &qp);
    d_ocp_qp_set_q(k, qvec.data(), &qp);
    if (k < N) {
      d_ocp_qp_set_A(k, A.data(), &qp);
      d_ocp_qp_set_B(k, B.data(), &qp);
      d_ocp_qp_set_b(k, b.data(), &qp);
      d_ocp_qp_set_R(k, R.data(), &qp);
      d_ocp_qp_set_r(k, rvec.data(), &qp);
      d_ocp_qp_set_idxbu(k, idxbu, &qp);
      d_ocp_qp_set_lbu(k, lbu, &qp);
      d_ocp_qp_set_ubu(k, ubu, &qp);
      d_ocp_qp_set_C(k, C.data(), &qp);
      d_ocp_qp_set_D(k, D.data(), &qp);
      d_ocp_qp_set_lg(k, lg.data(), &qp);
      d_ocp_qp_set_ug(k, ug.data(), &qp);
      d_ocp_qp_set_lg_mask(k, lg_mask.data(), &qp);
    }
  }
  d_ocp_qp_set_idxbx(0, idxbx, &qp);
  d_ocp_qp_set_lbx(0, x0.data(), &qp);
  d_ocp_qp_set_ubx(0, x0.data(), &qp);

  // ---- solver ----
  d_ocp_qp_sol sol;
  std::vector<char> sol_mem(d_ocp_qp_sol_memsize(&dim));
  d_ocp_qp_sol_create(&dim, &sol, sol_mem.data());

  d_ocp_qp_ipm_arg arg;
  std::vector<char> arg_mem(d_ocp_qp_ipm_arg_memsize(&dim));
  d_ocp_qp_ipm_arg_create(&dim, &arg, arg_mem.data());
  d_ocp_qp_ipm_arg_set_default(SPEED, &arg);
  int iter_max = 30;
  double tol = 1e-6;
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
  d_ocp_qp_ipm_arg_set_tol_stat(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_eq(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_comp(&tol, &arg);

  d_ocp_qp_ipm_ws ws;
  std::vector<char> ws_mem(d_ocp_qp_ipm_ws_memsize(&dim, &arg));
  d_ocp_qp_ipm_ws_create(&dim, &arg, &ws, ws_mem.data());

  // ---- timing ----
  constexpr int kReps = 2000;
  std::vector<double> times(kReps);
  int status = -1, iters = 0;
  for (int rep = 0; rep < kReps; ++rep) {
    const auto t0 = std::chrono::steady_clock::now();
    d_ocp_qp_ipm_solve(&qp, &sol, &arg, &ws);
    times[rep] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                     .count();
  }
  d_ocp_qp_ipm_get_status(&ws, &status);
  d_ocp_qp_ipm_get_iter(&ws, &iters);
  std::sort(times.begin(), times.end());

  // ---- checks ----
  VecU u0;
  d_ocp_qp_sol_get_u(0, &sol, u0.data());
  double fz_sum = 0.0, worst_friction = 0.0;
  for (int i = 0; i < 4; ++i) {
    const double fx = u0(3 * i), fy = u0(3 * i + 1), fz = u0(3 * i + 2);
    fz_sum += fz;
    worst_friction = std::max({worst_friction, std::abs(fx) - kMu * fz, std::abs(fy) - kMu * fz});
  }
  VecX xN;
  d_ocp_qp_sol_get_x(N, &sol, xN.data());

  std::printf("HPIPM status %d, %d iterations\n", status, iters);
  std::printf("solve time [ms] over %d runs: min %.3f  median %.3f  p99 %.3f  max %.3f\n", kReps,
              times.front(), times[kReps / 2], times[kReps * 99 / 100], times.back());
  std::printf("u0 forces [N]:\n");
  for (int i = 0; i < 4; ++i) {
    std::printf("  leg %d: %+7.2f %+7.2f %+7.2f\n", i, u0(3 * i), u0(3 * i + 1), u0(3 * i + 2));
  }
  std::printf("sum fz %.2f N (m*g = %.2f N), worst friction margin %+.2e (<= 0 is inside)\n",
              fz_sum, kMass * kG, worst_friction);
  std::printf("x_N: roll %+.4f rad, height %.4f m, vz %+.4f m/s\n", xN(0), xN(5), xN(11));

  const bool ok = status == 0 && worst_friction <= 1e-6 && times[kReps * 99 / 100] < 5.0;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
