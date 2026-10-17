#include "go2_mpc/kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace go2::kin {

Vec3 hipPosition(int leg) { return {frontSign(leg) * kHipX, sideSign(leg) * kHipY, 0.0}; }

// In the hip frame (after the abduction rotation), the leg is a planar 2-link arm in
// x-z plus the sideways offset:  v = [x, side * kHipLength, z] with
//   x = -l1 sin(q2) - l2 sin(q2 + q3),   z = -l1 cos(q2) - l2 cos(q2 + q3)
// and the foot is  p = hip + Rx(q1) v.

Vec3 footPosition(int leg, const Vec3& q) {
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const double s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  const double x = -kThighLength * s2 - kCalfLength * s23;
  const double y = sideSign(leg) * kHipLength;
  const double z = -kThighLength * c2 - kCalfLength * c23;
  return hipPosition(leg) + Vec3(x, c1 * y - s1 * z, s1 * y + c1 * z);
}

Mat3 footJacobian(int leg, const Vec3& q) {
  const double s1 = std::sin(q[0]), c1 = std::cos(q[0]);
  const double s2 = std::sin(q[1]), c2 = std::cos(q[1]);
  const double s23 = std::sin(q[1] + q[2]), c23 = std::cos(q[1] + q[2]);
  const double y = sideSign(leg) * kHipLength;
  const double z = -kThighLength * c2 - kCalfLength * c23;

  // Planar derivatives of (x, z) w.r.t. q2 and q3.
  const double dx2 = -kThighLength * c2 - kCalfLength * c23, dz2 = kThighLength * s2 + kCalfLength * s23;
  const double dx3 = -kCalfLength * c23, dz3 = kCalfLength * s23;

  Mat3 J;
  J.col(0) << 0.0, -s1 * y - c1 * z, c1 * y - s1 * z;
  J.col(1) << dx2, -s1 * dz2, c1 * dz2;
  J.col(2) << dx3, -s1 * dz3, c1 * dz3;
  return J;
}

namespace {

// go2.xml link inertials in each link's own frame. Legs are mirror images, so only
// signs change: products of inertia with y flip left/right, with x flip front/rear.
constexpr double kMass[3] = {0.678, 1.152, 0.241352};  // hip, thigh, calf

Vec3 linkCom(int leg, int link) {
  const double f = frontSign(leg), s = sideSign(leg);
  switch (link) {
    case 0: return {-f * 0.0054, s * 0.00194, -0.000105};
    case 1: return {-0.00374, -s * 0.0223, -0.0327};
    default: return {0.00629595, -s * 0.000622121, -0.141417};
  }
}

Mat3 linkInertia(int leg, int link) {
  const double f = frontSign(leg), s = sideSign(leg);
  double xx, yy, zz, xy, xz, yz;
  switch (link) {
    case 0: xx = 0.00048; yy = 0.000884; zz = 0.000596;
            xy = -s * f * 3.01e-6; xz = f * 1.11e-6; yz = -s * 1.42e-6; break;
    case 1: xx = 0.005840003; yy = 0.005800003; zz = 0.00103;
            xy = s * 8.72e-5; xz = -2.89001e-4; yz = s * 8.08e-4; break;
    default: xx = 0.001463537; yy = 0.001489993; zz = 0.000053269;
             xy = s * 2.18e-7; xz = 5.967e-6; yz = s * 1.2248e-5; break;
  }
  Mat3 I;
  I << xx, xy, xz, xy, yy, yz, xz, yz, zz;
  return I;
}

Mat3 rotX(double a) { return Eigen::AngleAxisd(a, Vec3::UnitX()).toRotationMatrix(); }
Mat3 rotY(double a) { return Eigen::AngleAxisd(a, Vec3::UnitY()).toRotationMatrix(); }

// Linear (at the link CoM) and angular Jacobians of the hip, thigh and calf links,
// base frame. For a point fixed in a link, p = hip + Rx(q1) v1 with
//   hip link:   v1 = c
//   thigh link: v1 = [0, s lh, 0] + Ry(q2) c
//   calf link:  v1 = [0, s lh, 0] + Ry(q2) v2,  v2 = [0, 0, -l1] + Ry(q3) c
// Each column is the joint axis crossed with the vector from the joint to the point.
struct LinkJacobians {
  std::array<Mat3, 3> Jv, Jw, R;  // R: link orientation in the base frame
};

LinkJacobians linkJacobians(int leg, const Vec3& q) {
  const Vec3 ex = Vec3::UnitX(), ey = Vec3::UnitY();
  const Mat3 R1 = rotX(q[0]), R12 = R1 * rotY(q[1]), R123 = R12 * rotY(q[2]);
  const Vec3 hip_off(0.0, sideSign(leg) * kHipLength, 0.0);
  LinkJacobians L;
  for (auto& J : L.Jv) J.setZero();
  for (auto& J : L.Jw) J.setZero();
  L.R = {R1, R12, R123};

  const Vec3 ch = linkCom(leg, 0);
  L.Jv[0].col(0) = R1 * ex.cross(ch);
  L.Jw[0].col(0) = ex;

  const Vec3 ct = linkCom(leg, 1);
  L.Jv[1].col(0) = R1 * ex.cross(hip_off + rotY(q[1]) * ct);
  L.Jv[1].col(1) = R12 * ey.cross(ct);
  L.Jw[1].col(0) = ex;
  L.Jw[1].col(1) = R1 * ey;

  const Vec3 cc = linkCom(leg, 2);
  const Vec3 v2 = Vec3(0.0, 0.0, -kThighLength) + rotY(q[2]) * cc;
  L.Jv[2].col(0) = R1 * ex.cross(hip_off + rotY(q[1]) * v2);
  L.Jv[2].col(1) = R12 * ey.cross(v2);
  L.Jv[2].col(2) = R123 * ey.cross(cc);
  L.Jw[2].col(0) = ex;
  L.Jw[2].col(1) = R1 * ey;
  L.Jw[2].col(2) = R12 * ey;
  return L;
}

}  // namespace

Vec3 legGravityTorque(int leg, const Vec3& q, const Vec3& g_body) {
  const LinkJacobians L = linkJacobians(leg, q);
  Vec3 tau = Vec3::Zero();
  for (int i = 0; i < 3; ++i) tau -= L.Jv[i].transpose() * (kMass[i] * g_body);
  return tau;
}

Mat3 legMassMatrix(int leg, const Vec3& q) {
  const LinkJacobians L = linkJacobians(leg, q);
  Mat3 M = kJointArmature * Mat3::Identity();
  for (int i = 0; i < 3; ++i) {
    const Mat3 I_base = L.R[i] * linkInertia(leg, i) * L.R[i].transpose();
    M += kMass[i] * L.Jv[i].transpose() * L.Jv[i] + L.Jw[i].transpose() * I_base * L.Jw[i];
  }
  return M;
}

bool inverse(int leg, const Vec3& p, Vec3& q) {
  bool reachable = true;
  const Vec3 d = p - hipPosition(leg);
  const double y = sideSign(leg) * kHipLength;

  // Abduction: Rx(q1) maps (y, z) to (d.y, d.z), with z < 0 (foot below the hip).
  double r2 = d.y() * d.y() + d.z() * d.z() - y * y;
  if (r2 < 1e-8) {
    r2 = 1e-8;
    reachable = false;
  }
  const double z = -std::sqrt(r2);
  q[0] = std::atan2(d.z(), d.y()) - std::atan2(z, y);
  q[0] = std::atan2(std::sin(q[0]), std::cos(q[0]));

  // Knee: law of cosines in the leg plane, knee bent backwards (q3 < 0).
  const double x = d.x();
  double c3 = (x * x + z * z - kThighLength * kThighLength - kCalfLength * kCalfLength) /
              (2.0 * kThighLength * kCalfLength);
  if (c3 > 1.0 || c3 < -1.0) {
    c3 = std::clamp(c3, -1.0, 1.0);
    reachable = false;
  }
  q[2] = -std::acos(c3);

  // Hip pitch: [-x, -z] = R(q2 + phi) [a, b] form, see footPosition.
  const double a = kThighLength + kCalfLength * c3;
  const double b = kCalfLength * std::sin(q[2]);
  q[1] = std::atan2(-x, -z) - std::atan2(b, a);
  return reachable;
}

}  // namespace go2::kin
