#include "go2_mpc/srb_mpc.hpp"

#include <chrono>
#include <cstdlib>
#include <stdexcept>

extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
}

namespace go2 {
namespace {

constexpr int kNx = SrbMpc::kNx;
using MatX = Eigen::Matrix<double, kNx, kNx>;
using MatXd = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

Mat3 skew(const Vec3& v) {
  Mat3 s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}

Mat3 rotZ(double yaw) {
  return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
}

// 64-byte aligned memory that only grows: after the first few solves (once the
// largest contact pattern has been seen) no allocation happens any more.
class Buffer {
 public:
  ~Buffer() { std::free(ptr_); }
  void* get(size_t size) {
    if (size > size_) {
      std::free(ptr_);
      size_ = (size + 63) / 64 * 64;
      ptr_ = std::aligned_alloc(64, size_);
      if (!ptr_) throw std::bad_alloc();
    }
    return ptr_;
  }

 private:
  void* ptr_ = nullptr;
  size_t size_ = 0;
};

}  // namespace

struct SrbMpc::Workspace {
  Buffer dim_mem, qp_mem, sol_mem, arg_mem, ipm_mem;
  // Per-stage data kept alive while HPIPM copies it.
  std::vector<MatX> A;
  std::vector<MatXd> B, D, C;
  std::vector<std::vector<int>> idxbu;
  std::vector<VecXd> lbu, ubu, lg, ug, lg_mask, r;
  std::vector<int> nu, ng;
};

SrbMpc::SrbMpc(const Params& params) : params_(params), ws_(new Workspace) {
  if (params_.horizon < 1) throw std::invalid_argument("SrbMpc: horizon must be >= 1");
}

SrbMpc::~SrbMpc() { delete ws_; }

SrbMpc::Solution SrbMpc::solve(const Problem& pb) {
  const auto t_start = std::chrono::steady_clock::now();
  const int N = params_.horizon;
  const double dt = params_.dt;
  if (static_cast<int>(pb.x_ref.size()) != N + 1 || static_cast<int>(pb.contact.size()) != N ||
      static_cast<int>(pb.foot_world.size()) != N) {
    throw std::invalid_argument("SrbMpc::solve: problem size does not match the horizon");
  }

  Workspace& w = *ws_;
  w.A.resize(N); w.B.resize(N); w.D.resize(N); w.C.resize(N); w.idxbu.resize(N);
  w.lbu.resize(N); w.ubu.resize(N); w.lg.resize(N); w.ug.resize(N); w.lg_mask.resize(N);
  w.r.resize(N); w.nu.assign(N + 1, 0); w.ng.assign(N + 1, 0);

  // ---- stage data ----
  const Mat3 I_body_inv = params_.inertia.inverse();
  for (int k = 0; k < N; ++k) {
    // Linearisation point: the current state moved along with the reference, so the
    // lever arms stay right even when the robot is away from the reference position.
    const double yaw = pb.x0[2] + (pb.x_ref[k][2] - pb.x_ref[0][2]);
    const Vec3 p_com = pb.x0.segment<3>(3) + (pb.x_ref[k].segment<3>(3) - pb.x_ref[0].segment<3>(3));
    const Mat3 Rz = rotZ(yaw);
    const Mat3 I_world_inv = Rz * I_body_inv * Rz.transpose();

    MatX& A = w.A[k];
    A.setIdentity();
    A.block<3, 3>(0, 6) = dt * Rz.transpose();
    A.block<3, 3>(3, 9) = dt * Mat3::Identity();

    int n = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) n += pb.contact[k][leg] ? 1 : 0;
    const int nu = 3 * n, ng = 4 * n;
    w.nu[k] = nu;
    w.ng[k] = ng;

    MatXd& B = w.B[k];
    B.setZero(kNx, nu);
    w.D[k].setZero(ng, nu);
    w.C[k].setZero(ng, kNx);
    w.idxbu[k].resize(n);
    w.lbu[k].setConstant(n, params_.fz_min);
    w.ubu[k].setConstant(n, params_.fz_max);
    w.lg[k].setZero(ng);
    w.ug[k].setZero(ng);
    w.lg_mask[k].setZero(ng);  // friction rows are one-sided (<= 0)
    w.r[k].setZero(nu);

    int j = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (!pb.contact[k][leg]) continue;
      const Vec3 r = pb.foot_world[k][leg] - p_com;
      B.block<3, 3>(6, 3 * j) = dt * I_world_inv * skew(r);
      B.block<3, 3>(9, 3 * j) = (dt / params_.mass) * Mat3::Identity();
      w.idxbu[k][j] = 3 * j + 2;  // fz
      const double mu = params_.mu;
      const int row = 4 * j, col = 3 * j;
      w.D[k](row + 0, col + 0) = 1;  w.D[k](row + 0, col + 2) = -mu;  //  fx <= mu fz
      w.D[k](row + 1, col + 0) = -1; w.D[k](row + 1, col + 2) = -mu;  // -fx <= mu fz
      w.D[k](row + 2, col + 1) = 1;  w.D[k](row + 2, col + 2) = -mu;
      w.D[k](row + 3, col + 1) = -1; w.D[k](row + 3, col + 2) = -mu;
      ++j;
    }
  }

  // ---- dimensions ----
  d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(N, &dim, w.dim_mem.get(d_ocp_qp_dim_memsize(N)));
  for (int k = 0; k <= N; ++k) {
    d_ocp_qp_dim_set_nx(k, kNx, &dim);
    d_ocp_qp_dim_set_nu(k, w.nu[k], &dim);
    d_ocp_qp_dim_set_nbx(k, k == 0 ? kNx : 0, &dim);
    d_ocp_qp_dim_set_nbu(k, w.nu[k] / 3, &dim);
    d_ocp_qp_dim_set_ng(k, w.ng[k], &dim);
    d_ocp_qp_dim_set_ns(k, 0, &dim);
  }

  // ---- QP ----
  d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, w.qp_mem.get(d_ocp_qp_memsize(&dim)));

  MatX Q = params_.q_weights.asDiagonal();
  StateVec b = StateVec::Zero();
  b[11] = -params_.gravity * dt;
  for (int k = 0; k <= N; ++k) {
    StateVec q = -(Q * pb.x_ref[k]);
    if (k == 0) q.setZero();  // x0 is fixed; its cost is irrelevant
    d_ocp_qp_set_Q(k, Q.data(), &qp);
    d_ocp_qp_set_q(k, q.data(), &qp);
    if (k == N) break;

    d_ocp_qp_set_A(k, w.A[k].data(), &qp);
    d_ocp_qp_set_b(k, b.data(), &qp);
    if (w.nu[k] > 0) {
      MatXd R = params_.r_weight * MatXd::Identity(w.nu[k], w.nu[k]);
      d_ocp_qp_set_B(k, w.B[k].data(), &qp);
      d_ocp_qp_set_R(k, R.data(), &qp);
      d_ocp_qp_set_r(k, w.r[k].data(), &qp);
      d_ocp_qp_set_idxbu(k, w.idxbu[k].data(), &qp);
      d_ocp_qp_set_lbu(k, w.lbu[k].data(), &qp);
      d_ocp_qp_set_ubu(k, w.ubu[k].data(), &qp);
      d_ocp_qp_set_C(k, w.C[k].data(), &qp);
      d_ocp_qp_set_D(k, w.D[k].data(), &qp);
      d_ocp_qp_set_lg(k, w.lg[k].data(), &qp);
      d_ocp_qp_set_ug(k, w.ug[k].data(), &qp);
      d_ocp_qp_set_lg_mask(k, w.lg_mask[k].data(), &qp);
    }
  }
  int idxbx[kNx];
  for (int i = 0; i < kNx; ++i) idxbx[i] = i;
  StateVec x0 = pb.x0;
  d_ocp_qp_set_idxbx(0, idxbx, &qp);
  d_ocp_qp_set_lbx(0, x0.data(), &qp);
  d_ocp_qp_set_ubx(0, x0.data(), &qp);

  // ---- solver ----
  d_ocp_qp_sol sol;
  d_ocp_qp_sol_create(&dim, &sol, w.sol_mem.get(d_ocp_qp_sol_memsize(&dim)));

  d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, w.arg_mem.get(d_ocp_qp_ipm_arg_memsize(&dim)));
  d_ocp_qp_ipm_arg_set_default(SPEED, &arg);
  int iter_max = params_.max_iter;
  double tol = params_.tol;
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
  d_ocp_qp_ipm_arg_set_tol_stat(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_eq(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &arg);
  d_ocp_qp_ipm_arg_set_tol_comp(&tol, &arg);

  d_ocp_qp_ipm_ws ipm;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &ipm, w.ipm_mem.get(d_ocp_qp_ipm_ws_memsize(&dim, &arg)));
  d_ocp_qp_ipm_solve(&qp, &sol, &arg, &ipm);

  Solution out;
  d_ocp_qp_ipm_get_status(&ipm, &out.status);
  d_ocp_qp_ipm_get_iter(&ipm, &out.iterations);
  out.ok = (out.status == 0);

  out.x_pred.resize(N + 1);
  for (int k = 0; k <= N; ++k) d_ocp_qp_sol_get_x(k, &sol, out.x_pred[k].data());
  out.f.assign(N, Vec12::Zero());
  for (int k = 0; k < N; ++k) {
    if (w.nu[k] == 0) continue;
    VecXd u(w.nu[k]);
    d_ocp_qp_sol_get_u(k, &sol, u.data());
    int j = 0;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (pb.contact[k][leg]) out.f[k].segment<3>(3 * leg) = u.segment<3>(3 * j++);
    }
    if (!out.f[k].allFinite()) out.ok = false;
  }
  out.f0 = out.f[0];

  out.solve_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
  return out;
}

}  // namespace go2
