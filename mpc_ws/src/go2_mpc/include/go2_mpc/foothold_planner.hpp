#pragma once

#include <array>

#include "go2_mpc/config.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Raibert heuristic foothold (world frame) for a leg that touches down after
// time_to_touchdown:
//
//   p_hip_td = p_base + v_cmd t_td + Rz(yaw + wz_cmd t_td) nominal_xy
//   p_foot   = p_hip_td + v T_stance / 2              (symmetric stance)
//                       + k_v (v - v_cmd)                (velocity error)
//                       + 0.5 sqrt(h / g) (v x w_cmd)    (centrifugal)
//
// v is the measured horizontal base velocity, v_cmd the commanded one (world).
// The offset from p_hip_td is clipped to foothold.max_step; z is the ground plus
// the foot radius (flat ground).
class FootholdPlanner {
 public:
  struct Input {
    Vec3 base_pos = Vec3::Zero();
    double yaw = 0.0;
    Vec3 vel = Vec3::Zero();      // measured, world
    Vec3 vel_cmd = Vec3::Zero();  // commanded, world
    double wz_cmd = 0.0;
    double height = 0.3;          // CoM height for the centrifugal term
  };

  FootholdPlanner(const Config& config, double stance_time);

  Vec3 plan(int leg, const Input& in, double time_to_touchdown) const;
  const Vec3& nominal(int leg) const { return nominal_[leg]; }

 private:
  const Config& config_;
  double stance_time_;
  std::array<Vec3, kNumLegs> nominal_{};  // foot at the stand pose, base frame
};

}  // namespace go2
