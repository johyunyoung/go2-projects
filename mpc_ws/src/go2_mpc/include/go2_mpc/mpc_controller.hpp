#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "go2_mpc/config.hpp"
#include "go2_mpc/foot_plan.hpp"
#include "go2_mpc/foothold_planner.hpp"
#include "go2_mpc/gait_scheduler.hpp"
#include "go2_mpc/srb_mpc.hpp"
#include "go2_mpc/state_estimator.hpp"
#include "go2_mpc/swing_trajectory.hpp"
#include "go2_mpc/types.hpp"

namespace go2 {

// Body pose offsets commanded in 4-leg stance (gait.type = stand, M3).
struct PoseCommand {
  double dz = 0.0;     // height offset from the height at MPC entry [m]
  double roll = 0.0;   // [rad]
  double pitch = 0.0;  // [rad], positive = nose down
  double yaw = 0.0;    // offset from the yaw at MPC entry [rad]
};

// What the debug stream shows about the MPC.
struct MpcStatus {
  bool has_solution = false;
  int status = -1;
  int iterations = 0;
  double solve_ms = 0.0;
  double age_s = 1e9;  // since the solution in use was computed
  uint64_t solves = 0;
  uint64_t failures = 0;
  Vec12 f = Vec12::Zero();  // forces being applied, world frame
  SrbMpc::StateVec x = SrbMpc::StateVec::Zero();
  SrbMpc::StateVec x_ref = SrbMpc::StateVec::Zero();
};

// Mode 2: SRB-MPC in its own thread at mpc.rate_hz, and the 500 Hz leg control.
//
// Control thread (step):
//   - command filtering, position/yaw target integration
//   - gait phase; at lift-off a world-frame Bezier swing from the measured foot to the
//     Raibert foothold (re-planned until swing.freeze_after); swing legs tracked in the
//     base frame with gravity + inertia feed-forward
//   - stance legs apply the MPC force of the stage that matches the current time
//   - gait start/stop (gait.idle = stand): starts when the command exceeds
//     start_speed / start_yaw_rate; stops once the command has been below them for
//     stop_delay and the robot is slow, at the next instant all feet are down
//   - publishes a snapshot (state, reference at stage 0, gait start/stop times, next
//     footholds), so the horizon already knows about a coming start or stop
// MPC thread: builds the horizon (reference, contact schedule from the gait, foot
// positions: current for legs still in stance, next foothold after a touch-down)
// from the latest snapshot and solves it.
class MpcController {
 public:
  explicit MpcController(const Config& config);
  ~MpcController();

  void start(const RobotState& state, double t);  // entering the mode
  void stop();                                     // leaving the mode

  JointCommand step(double t, const RobotState& state, const PoseCommand& pose,
                    const VelocityCommand& vel);

  bool timedOut(double t) const;
  MpcStatus status(double t) const;
  const FootPlan& plan() const { return plan_; }
  // Contact trust for the state estimator: ramps up over the first and down over the
  // last estimator.trust_window of each stance, 0 in swing, 1 while standing.
  ContactEstimate contact(double t) const;

 private:
  static constexpr double kNever = 1e300;

  struct Snapshot {
    RobotState state;
    double t = 0.0;
    uint64_t seq = 0;
    SrbMpc::StateVec ref0 = SrbMpc::StateVec::Zero();
    Vec3 vel_cmd_body = Vec3::Zero();  // vx, vy for propagating the reference
    double wz_cmd = 0.0;
    std::array<Vec3, kNumLegs> next_foothold{};
    double gait_start = kNever, gait_stop = kNever;
  };
  struct Output {
    SrbMpc::Solution sol;
    SrbMpc::StateVec x0 = SrbMpc::StateVec::Zero();
    SrbMpc::StateVec x_ref = SrbMpc::StateVec::Zero();
    double t_state = 0.0;  // controller time of the state it was computed from
    uint64_t solves = 0, failures = 0;
    bool valid = false;
  };

  bool stanceOnly() const { return config_.mpc.stance_only; }
  // Contact of a leg at time t for a gait running in [start, stop): stance outside.
  bool inContact(int leg, double t, double gait_start, double gait_stop) const;
  bool gaitRunning(double t) const { return t >= gait_start_ && t < gait_stop_; }
  void updateGait(double t, const RobotState& s, const VelocityCommand& vel);
  void threadLoop();
  SrbMpc::Problem buildProblem(const Snapshot& snap) const;
  SrbMpc::StateVec comState(const RobotState& s) const;
  void updateCommands(double t, const RobotState& s, const PoseCommand& pose,
                      const VelocityCommand& vel);

  const Config& config_;
  std::unique_ptr<SrbMpc> mpc_;
  GaitScheduler gait_;
  FootholdPlanner footholds_;

  // Targets (control thread).
  double t_start_ = 0.0, t_last_ = 0.0;
  double gait_start_ = kNever, gait_stop_ = kNever;  // gait runs in [start, stop)
  double command_seen_ = 0.0;                        // last time the command was non-zero
  double height_ = 0.0, yaw_anchor_ = 0.0;
  Vec3 pos_des_ = Vec3::Zero();  // CoM x/y target (z unused)
  double yaw_des_ = 0.0;
  PoseCommand pose_filtered_;
  VelocityCommand vel_filtered_;

  // Legs (control thread).
  std::array<bool, kNumLegs> was_stance_{};
  std::array<SwingTrajectory, kNumLegs> swing_{};
  std::array<double, kNumLegs> swing_t0_{};
  std::array<Vec3, kNumLegs> next_foothold_{};
  FootPlan plan_;

  mutable std::mutex in_mutex_, out_mutex_;
  Snapshot input_;
  Output output_;
  Vec12 applied_f_ = Vec12::Zero();  // forces sent to the stance legs (debug)

  std::atomic<bool> active_{false};
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace go2
