#pragma once

#include "go2_mpc/types.hpp"

namespace go2::kin {

// Go2 leg geometry (go2.xml == go2_description). All positions are in the base frame,
// relative to the base origin. The foot point is the *_foot body origin at the end of
// the calf (the collision sphere, r = 0.022 m, is centred 2 mm behind it).
//
// Each leg: hip joint (axis x) at hipPosition(leg); thigh joint (axis y) offset
// sideways by side * kHipLength; calf joint (axis y) kThighLength below it; foot
// kCalfLength below the calf joint. Joint angles q = (hip, thigh, calf).
constexpr double kHipX = 0.1934;
constexpr double kHipY = 0.0465;
constexpr double kHipLength = 0.0955;
constexpr double kThighLength = 0.213;
constexpr double kCalfLength = 0.213;
constexpr double kFootRadius = 0.022;
// Rotor inertia reflected to each joint (go2.xml armature). The real motor's value
// should be checked before relying on the inertia feed-forward on the robot.
constexpr double kJointArmature = 0.01;

// +1 for front legs / left legs, -1 for rear / right legs.
inline double frontSign(int leg) { return (leg == FR || leg == FL) ? 1.0 : -1.0; }
inline double sideSign(int leg) { return (leg == FL || leg == RL) ? 1.0 : -1.0; }

// Hip (abduction) joint position.
Vec3 hipPosition(int leg);

// Foot position for joint angles q of one leg.
Vec3 footPosition(int leg, const Vec3& q);

// d footPosition / dq (3x3, columns = hip, thigh, calf).
Mat3 footJacobian(int leg, const Vec3& q);

// Joint angles that place the foot at p (base frame). Uses the knee-backward branch
// (calf < 0) with the foot below the hip, which is the Go2's working configuration.
// Unreachable targets are projected onto the workspace boundary; returns false then.
bool inverse(int leg, const Vec3& p, Vec3& q);

// Joint torques that hold the leg's own links (hip, thigh, calf) against gravity,
// for gravity g_body expressed in the base frame (g_body = R^T [0, 0, -g]).
// Link masses and CoMs from go2.xml. Does not include any load on the foot.
Vec3 legGravityTorque(int leg, const Vec3& q, const Vec3& g_body);

// Joint-space inertia of one leg (3x3) with the base held still: link masses and
// inertia tensors from go2.xml plus the rotor armature.
Mat3 legMassMatrix(int leg, const Vec3& q);

// Helpers for the 12-vector in SDK order.
inline Vec3 legJoints(const Vec12& q, int leg) { return q.segment<3>(3 * leg); }

}  // namespace go2::kin
