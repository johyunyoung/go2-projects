#pragma once

// Wire format of the controller's debug stream (UDP, one packet per datagram).
//
// Shared verbatim by go2_mpc (sender, no ROS) and go2_mpc_viz (receiver, ROS 2), so
// this header must stay dependency-free. Fixed-width fields only: x86_64 and aarch64
// (the robot's Jetson) are both little-endian and lay this struct out identically.
// Bump kDebugPacketVersion whenever the layout changes; the receiver drops packets
// whose magic, version or size does not match.

#include <cstdint>

namespace go2 {

constexpr uint32_t kDebugPacketMagic = 0x504D3247;  // "G2MP"
constexpr uint16_t kDebugPacketVersion = 4;

struct DebugPacket {
  // --- header ---
  uint32_t magic = kDebugPacketMagic;
  uint16_t version = kDebugPacketVersion;
  uint16_t size = sizeof(DebugPacket);
  uint64_t seq = 0;
  double t = 0.0;  // controller time [s]

  // --- controller ---
  uint8_t mode = 0;  // go2::Mode
  uint8_t ground_truth_valid = 0;
  uint8_t mpc_has_solution = 0;
  int8_t mpc_status = -1;  // HPIPM status of the last solve (0 = success)
  uint8_t pad0[4] = {};
  float loop_period_ms = 0.f;   // last measured control period
  float loop_compute_ms = 0.f;  // time spent computing the last tick
  float lowstate_age_ms = 0.f;
  float cmd_vel[3] = {};  // vx, vy, wz (body frame)

  // --- robot state (SDK joint order: FR, FL, RR, RL x hip, thigh, calf) ---
  float q[12] = {};
  float dq[12] = {};
  float tau_est[12] = {};
  float quat[4] = {1.f, 0.f, 0.f, 0.f};  // w, x, y, z (body -> world)
  float gyro[3] = {};
  float acc[3] = {};
  float base_pos[3] = {};  // world, from ground truth when valid
  float base_vel[3] = {};  // world, from ground truth when valid

  // --- command actually sent to rt/lowcmd ---
  float q_des[12] = {};
  float dq_des[12] = {};
  float kp[12] = {};
  float kd[12] = {};
  float tau_ff[12] = {};

  // --- MPC (v2) ---
  uint16_t mpc_iterations = 0;
  uint16_t pad1 = 0;
  float mpc_solve_ms = 0.f;   // problem build + solve
  float mpc_age_ms = 0.f;     // age of the solution being applied
  uint32_t mpc_solves = 0;
  uint32_t mpc_failures = 0;
  float grf[12] = {};         // forces applied, world frame, per leg
  float com_state[12] = {};   // MPC x0: rpy, p_com, omega_world, v_com
  float com_ref[12] = {};     // MPC reference (first predicted stage)
  float foot_world[12] = {};  // foot positions, world frame
  float pose_cmd[4] = {};     // dz, roll, pitch, yaw offsets

  // --- foot plan (v3): swing test (M4), walking (M5) ---
  uint8_t plan_valid = 0;
  uint8_t stance_mask = 0;    // bit i set = leg i (FR, FL, RR, RL) in stance
  uint8_t pad2[2] = {};
  float foot_des_world[12] = {};
  float swing_curve[4][10][3] = {};  // per leg, sampled swing path (stance: repeated point)

  // --- estimation (v4) --- base_pos/base_vel above are what the controller uses
  uint8_t kalman_in_use = 0;  // 1: base_pos/base_vel come from the Kalman filter
  uint8_t gt_valid = 0;       // simulator ground truth available
  uint8_t pad3[2] = {};
  float kf_pos[3] = {};       // Kalman filter, world
  float kf_vel[3] = {};
  float gt_pos[3] = {};       // ground truth (simulator), world
  float gt_vel[3] = {};
  float contact_trust[4] = {};
};

static_assert(sizeof(DebugPacket) == 1336, "DebugPacket layout changed: bump the version");

}  // namespace go2
