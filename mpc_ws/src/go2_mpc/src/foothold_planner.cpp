#include "go2_mpc/foothold_planner.hpp"

#include <cmath>

#include "go2_mpc/kinematics.hpp"

namespace go2 {

FootholdPlanner::FootholdPlanner(const Config& config, double stance_time)
    : config_(config), stance_time_(stance_time) {
  const Vec12& q_stand = config.stand.waypoints.back();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    nominal_[leg] = kin::footPosition(leg, kin::legJoints(q_stand, leg));
  }
}

Vec3 FootholdPlanner::plan(int leg, const Input& in, double t_td) const {
  const double yaw_td = in.yaw + in.wz_cmd * t_td;
  const Vec3 nominal_xy(nominal_[leg].x(), nominal_[leg].y(), 0.0);
  const Vec3 hip_td = in.base_pos + in.vel_cmd * t_td +
                      Eigen::AngleAxisd(yaw_td, Vec3::UnitZ()).toRotationMatrix() * nominal_xy;

  const Vec3 v(in.vel.x(), in.vel.y(), 0.0);
  const Vec3 v_cmd(in.vel_cmd.x(), in.vel_cmd.y(), 0.0);
  Vec3 offset = 0.5 * stance_time_ * v + config_.foothold.k_v * (v - v_cmd) +
                0.5 * std::sqrt(in.height / config_.model.gravity) * v.cross(Vec3(0, 0, in.wz_cmd));
  offset.z() = 0.0;
  const double max_step = config_.foothold.max_step;
  if (offset.norm() > max_step) offset *= max_step / offset.norm();

  Vec3 p = hip_td + offset;
  p.z() = kin::kFootRadius;  // flat ground at z = 0
  return p;
}

}  // namespace go2
