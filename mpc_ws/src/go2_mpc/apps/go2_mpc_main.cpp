// go2_mpc: SRB-MPC controller for Unitree Go2 (unitree_sdk2, no ROS).
//
//   go2_mpc <config.yaml> [--network IFACE]
//
// Against unitree_mujoco: network "lo". On the robot: the NIC wired to the Go2.

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "go2_mpc/config.hpp"
#include "go2_mpc/debug_link.hpp"
#include "go2_mpc/fsm.hpp"
#include "go2_mpc/remote.hpp"
#include "go2_mpc/robot_interface.hpp"
#include "go2_mpc/script.hpp"
#include "go2_mpc/sim_noise.hpp"
#include "go2_mpc/state_estimator.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) { g_stop = true; }

double toSec(const timespec& ts) { return ts.tv_sec + 1e-9 * ts.tv_nsec; }

void addNs(timespec& ts, long ns) {
  ts.tv_nsec += ns;
  while (ts.tv_nsec >= 1000000000L) {
    ts.tv_nsec -= 1000000000L;
    ++ts.tv_sec;
  }
}

void setupRealtime(const go2::Config::Control& c) {
  if (!c.realtime) return;
  sched_param sp{};
  sp.sched_priority = c.priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    std::fprintf(stderr, "[main] SCHED_FIFO not permitted, running with normal priority\n");
  }
  if (c.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(c.cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
      std::fprintf(stderr, "[main] could not pin to CPU %d\n", c.cpu);
    }
  }
}

// Min / mean / max of the loop period and compute time over a print window.
struct LoopStats {
  double period_min = std::numeric_limits<double>::max(), period_max = 0, period_sum = 0;
  double compute_max = 0;
  long n = 0, overruns = 0;

  void add(double period, double compute, double dt) {
    period_min = std::min(period_min, period);
    period_max = std::max(period_max, period);
    period_sum += period;
    compute_max = std::max(compute_max, compute);
    if (compute > dt) ++overruns;
    ++n;
  }
  void print(go2::Mode mode, const go2::VelocityCommand& v, const go2::MpcStatus& mpc) const {
    if (n == 0) return;
    std::printf(
        "[loop] %-7s period ms min %.3f mean %.3f max %.3f | compute max %.3f ms | "
        "overruns %ld | cmd vx %+.2f vy %+.2f wz %+.2f\n",
        go2::modeName(mode), 1e3 * period_min, 1e3 * period_sum / n, 1e3 * period_max,
        1e3 * compute_max, overruns, v.vx, v.vy, v.wz);
    if (mode == go2::Mode::Mpc) {
      const auto& x = mpc.x;
      std::printf(
          "[mpc]  solves %lu failures %lu | last solve %.3f ms, %d it | rpy %+.3f %+.3f %+.3f "
          "| com z %.3f (ref %.3f) | fz %.1f %.1f %.1f %.1f\n",
          static_cast<unsigned long>(mpc.solves), static_cast<unsigned long>(mpc.failures),
          mpc.solve_ms, mpc.iterations, x[0], x[1], x[2], x[5], mpc.x_ref[5], mpc.f[2], mpc.f[5],
          mpc.f[8], mpc.f[11]);
    }
  }
};

void fillPacket(go2::DebugPacket& p, double t, go2::Mode mode, const go2::LowStateData& s,
                const go2::RobotState& rs, const go2::VelocityCommand& v,
                const go2::JointCommand& cmd, const go2::MpcStatus& mpc,
                const go2::PoseCommand& pose, const go2::FootPlan* plan, bool kalman_in_use,
                const go2::RobotState& kf, const std::optional<go2::BaseGroundTruth>& gt,
                const go2::ContactEstimate& contact, double period, double compute, double age) {
  ++p.seq;
  p.t = t;
  p.mode = static_cast<uint8_t>(mode);
  p.ground_truth_valid = rs.valid;
  p.loop_period_ms = static_cast<float>(1e3 * period);
  p.loop_compute_ms = static_cast<float>(1e3 * compute);
  p.lowstate_age_ms = static_cast<float>(1e3 * age);
  p.cmd_vel[0] = v.vx;
  p.cmd_vel[1] = v.vy;
  p.cmd_vel[2] = v.wz;
  for (int i = 0; i < go2::kNumJoints; ++i) {
    p.q[i] = s.q[i];
    p.dq[i] = s.dq[i];
    p.tau_est[i] = s.tau_est[i];
    p.q_des[i] = cmd.q[i];
    p.dq_des[i] = cmd.dq[i];
    p.kp[i] = cmd.kp[i];
    p.kd[i] = cmd.kd[i];
    p.tau_ff[i] = cmd.tau[i];
  }
  p.quat[0] = s.quat.w();
  p.quat[1] = s.quat.x();
  p.quat[2] = s.quat.y();
  p.quat[3] = s.quat.z();
  for (int i = 0; i < 3; ++i) {
    p.gyro[i] = s.gyro[i];
    p.acc[i] = s.acc[i];
    p.base_pos[i] = rs.pos[i];
    p.base_vel[i] = rs.vel[i];
  }
  p.mpc_has_solution = mpc.has_solution;
  p.mpc_status = static_cast<int8_t>(mpc.status);
  p.mpc_iterations = static_cast<uint16_t>(mpc.iterations);
  p.mpc_solve_ms = static_cast<float>(mpc.solve_ms);
  p.mpc_age_ms = static_cast<float>(std::min(1e3 * mpc.age_s, 1e6));
  p.mpc_solves = static_cast<uint32_t>(mpc.solves);
  p.mpc_failures = static_cast<uint32_t>(mpc.failures);
  for (int i = 0; i < 12; ++i) {
    p.grf[i] = mpc.f[i];
    p.com_state[i] = mpc.x[i];
    p.com_ref[i] = mpc.x_ref[i];
    p.foot_world[i] = rs.foot_world[i / 3][i % 3];
  }
  p.pose_cmd[0] = pose.dz;
  p.pose_cmd[1] = pose.roll;
  p.pose_cmd[2] = pose.pitch;
  p.pose_cmd[3] = pose.yaw;
  p.kalman_in_use = kalman_in_use;
  p.gt_valid = gt.has_value();
  for (int i = 0; i < 3; ++i) {
    p.kf_pos[i] = kf.pos[i];
    p.kf_vel[i] = kf.vel[i];
    p.gt_pos[i] = gt ? gt->pos[i] : 0.0;
    p.gt_vel[i] = gt ? gt->vel[i] : 0.0;
  }
  for (int leg = 0; leg < go2::kNumLegs; ++leg) p.contact_trust[leg] = contact.trust[leg];
  p.plan_valid = plan != nullptr;
  p.stance_mask = 0;
  if (plan) {
    for (int leg = 0; leg < go2::kNumLegs; ++leg) {
      if (plan->stance[leg]) p.stance_mask |= 1u << leg;
      for (int k = 0; k < 3; ++k) {
        p.foot_des_world[3 * leg + k] = plan->des_world[leg][k];
        for (int i = 0; i < go2::FootPlan::kCurvePoints; ++i) p.swing_curve[leg][i][k] = plan->curve[leg][i][k];
      }
    }
  }
}

// Pose offsets for 4-leg stance from the sticks (--stance-only, M3):
//   W/S (ly) -> pitch (W = nose down), A/D (lx) -> roll (A = lean left), Q/E (rx) -> yaw (Q = left)
go2::PoseCommand poseFromRemote(const go2::RemoteAxes& a, const go2::Config::Mpc& m) {
  go2::PoseCommand p;
  p.pitch = m.pitch_scale * a.ly;
  p.roll = m.roll_scale * a.lx;
  p.yaw = -m.yaw_scale * a.rx;
  return p;
}

// auto_stand_at < 0 disables the automatic Stand request.
int run(const go2::Config& config, double auto_stand_at, std::optional<go2::Script> script) {
  go2::RobotInterface robot(config);
  robot.start();
  std::printf("[main] waiting for rt/lowstate on '%s' ...\n", config.robot.network.c_str());
  if (!robot.waitForLowState(10.0)) {
    std::fprintf(stderr, "[main] no rt/lowstate within 10 s (simulator running?)\n");
    return 1;
  }
  std::printf("[main] connected. keys in the MuJoCo window: 1 stand, 2 run, 0 passive\n");

  go2::Remote remote(config.command);
  go2::SimNoise noise(config.sim_noise, config.sim_noise.scale);
  if (noise.enabled()) std::printf("[main] simulated sensor noise, scale %.2f\n", config.sim_noise.scale);
  go2::GroundTruthEstimator gt_estimator;
  go2::KalmanEstimator kalman(config);
  const bool use_kalman = config.robot.estimator == "kalman";
  std::printf("[main] state estimate driving the controller: %s\n", config.robot.estimator.c_str());
  go2::Fsm fsm(config);
  std::unique_ptr<go2::DebugLink> debug;
  if (config.debug.enable) {
    debug = std::make_unique<go2::DebugLink>(config.debug.host, config.debug.port);
    std::printf("[main] debug stream -> %s:%d\n", config.debug.host.c_str(), config.debug.port);
  }
  go2::DebugPacket packet;

  setupRealtime(config.control);
  const long dt_ns = static_cast<long>(config.control.dt * 1e9);

  timespec start, next, now;
  clock_gettime(CLOCK_MONOTONIC, &start);
  next = start;
  double last_tick = toSec(start);
  double last_print = last_tick;
  LoopStats stats;
  long tick = 0;
  go2::JointCommand sent;

  while (!g_stop) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    const double t_now = toSec(now);
    const double period = t_now - last_tick;
    last_tick = t_now;

    go2::LowStateData low = robot.lowState();
    noise.apply(low);
    const auto gt = robot.groundTruth();
    const double age = robot.lowStateAge();
    const double t_ctrl = t_now - toSec(start);
    // Contact trust from the gait as of the previous tick; the Kalman filter always
    // runs (in the simulator alongside ground truth, for comparison).
    const go2::ContactEstimate contact = fsm.contact(t_ctrl);
    const go2::RobotState kf = kalman.update(low, gt, contact);
    const go2::RobotState rs = use_kalman ? kf : gt_estimator.update(low, gt, contact);
    remote.update(low.remote);

    go2::RemoteEvents events = remote.events();
    go2::PoseCommand pose = poseFromRemote(remote.axes(), config.mpc);
    go2::VelocityCommand vel = remote.velocity();
    if (script) script->apply(t_ctrl, events, pose, vel);
    if (auto_stand_at >= 0.0 && t_ctrl >= auto_stand_at) {
      std::printf("[main] --auto-stand: requesting Stand\n");
      events.stand = true;
      auto_stand_at = -1.0;
    }

    go2::ControlInput in;
    in.t = t_ctrl;
    in.low = &low;
    in.state = &rs;
    in.events = &events;
    in.pose = &pose;
    in.velocity = &vel;
    in.lowstate_stale = age > config.safety.lowstate_timeout;

    sent = robot.send(fsm.step(in));

    clock_gettime(CLOCK_MONOTONIC, &now);
    const double compute = toSec(now) - t_now;
    if (tick > 0) stats.add(period, compute, config.control.dt);

    if (debug && tick % config.debug.decimation == 0) {
      fillPacket(packet, in.t, fsm.mode(), low, rs, vel, sent, fsm.mpcStatus(in.t),
                 pose, fsm.footPlan(), use_kalman, kf, gt, contact, period, compute, age);
      debug->send(packet);
    }
    if (config.debug.print_period > 0 && t_now - last_print >= config.debug.print_period) {
      stats.print(fsm.mode(), vel, fsm.mpcStatus(in.t));
      stats = LoopStats{};
      last_print = t_now;
    }

    ++tick;
    addNs(next, dt_ns);
    // If we fell more than one period behind, resynchronise instead of bursting.
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (toSec(now) - toSec(next) > config.control.dt) {
      next = now;
      addNs(next, dt_ns);
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
  }

  // Leave the motors damped, not holding whatever the last command was.
  go2::JointCommand damp;
  damp.kd = config.passive.kd;
  for (int i = 0; i < 50; ++i) {
    robot.send(damp);
    timespec ts{0, 2000000L};
    nanosleep(&ts, nullptr);
  }
  std::printf("[main] stopped, motors left in damping\n");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: %s <config.yaml> [--network IFACE] [--auto-stand SEC] [--script FILE]\n"
                 "  --auto-stand SEC  request Stand SEC seconds after start, as if LT+A\n"
                 "                    were pressed (for tests without the remote)\n"
                 "  --script FILE     scripted events and pose commands (see script.hpp)\n"
                 "  --swing-test      start runs the M4 bench swing test instead of the MPC\n"
                 "  --stance-only     MPC keeps all four feet down; sticks = body pose (M3)\n"
                 "  --estimator E     ground_truth | kalman: state estimate driving the controller\n"
                 "  --sim-noise SCALE simulated sensor noise (1 = typical MEMS IMU, 0 = off)\n"
                 "  --idle MODE       zero command: stand | step (trot in place)\n",
                 argv[0]);
    return 2;
  }
  std::setvbuf(stdout, nullptr, _IOLBF, 0);  // keep log lines live when redirected
  try {
    go2::Config config = go2::loadConfig(argv[1]);
    double auto_stand_at = -1.0;
    std::optional<go2::Script> script;
    for (int i = 2; i + 1 < argc; ++i) {
      if (std::strcmp(argv[i], "--network") == 0) config.robot.network = argv[++i];
      else if (std::strcmp(argv[i], "--estimator") == 0) config.robot.estimator = argv[++i];
      else if (std::strcmp(argv[i], "--sim-noise") == 0) config.sim_noise.scale = std::stod(argv[++i]);
      else if (std::strcmp(argv[i], "--idle") == 0) config.gait.idle = argv[++i];
      else if (std::strcmp(argv[i], "--auto-stand") == 0) auto_stand_at = std::stod(argv[++i]);
      else if (std::strcmp(argv[i], "--script") == 0) script = go2::Script::load(argv[++i]);
    }
    for (int i = 2; i < argc; ++i) {
      if (std::strcmp(argv[i], "--swing-test") == 0) config.swing_test.enable = true;
      if (std::strcmp(argv[i], "--stance-only") == 0) config.mpc.stance_only = true;
    }
    if (config.swing_test.enable) {
      std::printf("[main] --swing-test: start runs the bench swing test (base must be welded:\n"
                  "       entrypoint.sh sim -s <mpc_ws>/src/go2_mpc/sim/scene_fixed.xml)\n");
    }
    if (config.robot.estimator != "ground_truth" && config.robot.estimator != "kalman") {
      throw std::runtime_error("--estimator must be ground_truth or kalman");
    }
    if (config.gait.idle != "stand" && config.gait.idle != "step") {
      throw std::runtime_error("--idle must be stand or step");
    }
    if (config.sim_noise.scale > 0.0 && !config.robot.use_ground_truth) {
      throw std::runtime_error("sim_noise is for the simulator only (robot.use_ground_truth: false)");
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    return run(config, auto_stand_at, std::move(script));
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[main] error: %s\n", e.what());
    return 1;
  }
}
