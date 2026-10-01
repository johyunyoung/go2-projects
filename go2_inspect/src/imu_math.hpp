// READ-ONLY: no publishers
//
// 쿼터니언 → 몸통 좌표계 중력 방향 계산.
// 쿼터니언은 몸통→월드 회전 R 을 나타낸다고 보고 g_body = R^T * (0, 0, -1) 을 구한다.
// 똑바로 놓인 로봇이라면 올바른 해석에서 g_body ≈ (0, 0, -1).
#pragma once

#include <array>
#include <cmath>

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad2Deg = 180.0 / kPi;

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

inline double Norm(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

// 두 벡터 사이 각도 [deg]. 길이가 0 이면 NaN.
inline double AngleDeg(const Vec3& a, const Vec3& b) {
  const double na = Norm(a), nb = Norm(b);
  if (na < 1e-9 || nb < 1e-9) return std::nan("");
  double c = (a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb);
  if (c > 1.0) c = 1.0;
  if (c < -1.0) c = -1.0;
  return std::acos(c) * kRad2Deg;
}

// (w, x, y, z) 단위 쿼터니언 기준 g_body = R^T (0,0,-1) = -(R20, R21, R22)
inline Vec3 GravityInBody(double w, double x, double y, double z) {
  const double n = std::sqrt(w * w + x * x + y * y + z * z);
  if (n < 1e-6) return Vec3{};
  w /= n; x /= n; y /= n; z /= n;
  Vec3 g;
  g.x = -2.0 * (x * z - w * y);
  g.y = -2.0 * (y * z + w * x);
  g.z = -(1.0 - 2.0 * (x * x + y * y));
  return g;
}

// 해석 (a): q[0..3] = (w, x, y, z)
template <typename T>
inline Vec3 GravityWXYZ(const std::array<T, 4>& q) {
  return GravityInBody(q[0], q[1], q[2], q[3]);
}

// 해석 (b): q[0..3] = (x, y, z, w)
template <typename T>
inline Vec3 GravityXYZW(const std::array<T, 4>& q) {
  return GravityInBody(q[3], q[0], q[1], q[2]);
}

// 가속도계 기준 중력 방향: 정지 상태 가속도계는 비력(위쪽 +g)을 재므로 -acc/|acc|.
template <typename T>
inline Vec3 GravityFromAcc(const std::array<T, 3>& acc) {
  Vec3 v{-static_cast<double>(acc[0]), -static_cast<double>(acc[1]), -static_cast<double>(acc[2])};
  const double n = Norm(v);
  if (n < 1e-6) return Vec3{};
  return Vec3{v.x / n, v.y / n, v.z / n};
}

inline Vec3 DownVector() { return Vec3{0.0, 0.0, -1.0}; }
