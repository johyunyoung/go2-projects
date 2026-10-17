#pragma once

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Mode 1: moves the joints from wherever they are to the initial pose.
//
// Each segment goes from the previous waypoint (the measured posture for the first)
// to the next one with a smoothstep profile s = 3u^2 - 2u^3, so position and velocity
// are continuous and the velocity is zero at every waypoint. The motor PD tracks
// q_des with dq_des as velocity feed-forward. After the last waypoint it holds.
//
// With gravity compensation, each leg also gets tau = -J^T R^T [0, 0, w m g / 4]:
// the joint torque that makes the foot carry a quarter of the weight. w fades from 0
// to 1 with the last segment's smoothstep (before that the body lies on the ground).
class StandController {
 public:
  explicit StandController(const Config& config) : config_(config), stand_(config.stand) {}

  // hold = false: run every waypoint (getting up from the ground).
  // hold = true: go straight to the last waypoint with full gravity compensation
  // (the robot is already standing, e.g. when leaving the MPC).
  void start(const Vec12& q_now, double t, bool hold = false);

  // q: measured joints, R: body orientation (body -> world).
  JointCommand step(double t, const Vec12& q, const Mat3& R) const;

  bool finished(double t) const { return t - t0_ >= total_duration_; }
  const Vec12& target() const { return stand_.waypoints.back(); }

  // True exactly once, report_delay seconds after finishing: time to log how well
  // the pose is held.
  bool reportDue(double t);

 private:
  // Fraction of the weight the legs carry at time t (0..1).
  double loadFraction(double t) const;

  const Config& config_;
  const Config::Stand& stand_;
  Vec12 q_start_ = Vec12::Zero();
  size_t first_ = 0;  // first waypoint of this run
  double t0_ = 0.0;
  double total_duration_ = 0.0;
  bool reported_ = false;
};

}  // namespace go2
