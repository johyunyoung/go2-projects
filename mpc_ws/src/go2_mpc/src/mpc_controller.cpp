#include "go2_mpc/mpc_controller.hpp"

#include <time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "go2_mpc/kinematics.hpp"
#include "go2_mpc/leg_controller.hpp"

namespace go2 {
namespace {

double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

double approach(double current, double target, double max_step) {
  return current + std::clamp(target - current, -max_step, max_step);
}

Mat3 rotZ(double yaw) { return Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix(); }

void addNs(timespec& ts, long ns) {
  ts.tv_nsec += ns;
  while (ts.tv_nsec >= 1000000000L) {
    ts.tv_nsec -= 1000000000L;
    ++ts.tv_sec;
  }
}

SrbMpc::Params mpcParams(const Config& c) {
  SrbMpc::Params p;
  p.horizon = c.mpc.horizon;
  p.dt = c.mpc.dt;
  p.mass = c.model.mass;
  p.inertia = c.mpc.inertia;
  p.gravity = c.model.gravity;
  p.mu = c.mpc.mu;
  p.fz_min = c.mpc.fz_min;
  p.fz_max = c.mpc.fz_max;
  p.q_weights = c.mpc.q_weights;
  p.r_weight = c.mpc.r_weight;
  p.max_iter = c.mpc.max_iter;
  p.tol = c.mpc.tol;
  return p;
}

GaitScheduler::Params gaitParams(const Config& c) {
  GaitScheduler::Params p;
  p.period = c.gait.period;
  p.duty = c.gait.duty;
  p.offsets = c.gait.offsets;
  return p;
}

}  // namespace

MpcController::MpcController(const Config& config)
    : config_(config),
      mpc_(std::make_unique<SrbMpc>(mpcParams(config))),
      gait_(gaitParams(config)),
      footholds_(config, config.gait.duty * config.gait.period) {
  thread_ = std::thread([this] { threadLoop(); });
}

MpcController::~MpcController() {
  running_ = false;
  if (thread_.joinable()) thread_.join();
}

SrbMpc::StateVec MpcController::comState(const RobotState& s) const {
  const Vec3 c_world = s.R * config_.mpc.com_offset;
  SrbMpc::StateVec x;
  x.segment<3>(0) = s.rpy;
  x.segment<3>(3) = s.pos + c_world;
  x.segment<3>(6) = s.omega_world;
  x.segment<3>(9) = s.vel + s.omega_world.cross(c_world);
  return x;
}

bool MpcController::inContact(int leg, double t, double gait_start, double gait_stop) const {
  if (stanceOnly() || t < gait_start || t >= gait_stop) return true;
  GaitScheduler g(gait_.params());
  g.start(gait_start);
  return g.inContact(leg, t);
}

void MpcController::updateGait(double t, const RobotState& s, const VelocityCommand& vel) {
  if (stanceOnly()) return;
  const auto& g = config_.gait;

  if (g.idle == "step") {  // trot in place from stand_before on, never stop
    if (gait_start_ == kNever && t >= t_start_ + g.stand_before) {
      gait_start_ = t;
      gait_.start(t);
      std::printf("[gait] t=%.3f start (idle: step)\n", t);
    }
    return;
  }

  const bool commanded = std::abs(vel.vx) > g.start_speed || std::abs(vel.vy) > g.start_speed ||
                         std::abs(vel.wz) > g.start_yaw_rate;
  if (commanded) command_seen_ = t;

  if (t >= gait_stop_) {  // the scheduled stop has been reached: all feet are down
    gait_start_ = gait_stop_ = kNever;
    // Re-centre on the feet as they landed, keep the current heading.
    pos_des_.setZero();
    for (const auto& f : s.foot_world) pos_des_ += f / kNumLegs;
    pos_des_.z() = 0.0;
    yaw_des_ = s.rpy.z();
    std::printf("[gait] t=%.3f stopped, standing\n", t);
  }

  if (!gaitRunning(t)) {
    if (commanded && t >= t_start_ + g.stand_before) {
      gait_start_ = t;
      gait_stop_ = kNever;
      gait_.start(t);
      std::printf("[gait] t=%.3f start\n", t);
    }
    return;
  }

  if (commanded) {
    if (gait_stop_ != kNever) {
      gait_stop_ = kNever;
      std::printf("[gait] t=%.3f stop cancelled\n", t);
    }
    return;
  }

  // Stop once the command has been idle for stop_delay, the filtered command has
  // ramped down and the robot has slowed down: at the moment the legs in the air land
  // (for a trot, the other pair would lift at that instant and now stays down).
  const double speed = std::hypot(s.vel.x(), s.vel.y());
  const bool settled = std::abs(vel_filtered_.vx) < 1e-3 && std::abs(vel_filtered_.vy) < 1e-3 &&
                       std::abs(vel_filtered_.wz) < 1e-3;
  if (gait_stop_ == kNever && t - command_seen_ >= g.stop_delay && settled &&
      speed < g.stop_speed) {
    double stop = t;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const auto ph = gait_.leg(leg, t);
      if (!ph.stance) stop = std::max(stop, t + ph.remaining);
    }
    gait_stop_ = stop;
    std::printf("[gait] t=%.3f stop scheduled at t=%.3f (speed %.3f m/s)\n", t, stop, speed);
  }
}

void MpcController::start(const RobotState& state, double t) {
  const SrbMpc::StateVec x = comState(state);
  t_start_ = t_last_ = t;
  gait_start_ = gait_stop_ = kNever;
  command_seen_ = t;
  height_ = x[5];
  yaw_anchor_ = yaw_des_ = state.rpy.z();
  // Centre the CoM over the feet.
  pos_des_.setZero();
  for (const auto& f : state.foot_world) pos_des_ += f / kNumLegs;
  pos_des_.z() = 0.0;
  pose_filtered_ = PoseCommand{};
  vel_filtered_ = VelocityCommand{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    was_stance_[leg] = true;
    next_foothold_[leg] = state.foot_world[leg];
  }
  {
    std::lock_guard<std::mutex> lock(out_mutex_);
    output_ = Output{};
  }
  {
    std::lock_guard<std::mutex> lock(in_mutex_);
    input_ = Snapshot{};
    input_.state = state;
    input_.t = t;
    input_.seq = 1;
    input_.ref0 = x;
    input_.ref0.segment<3>(3) = Vec3(pos_des_.x(), pos_des_.y(), height_);
    input_.ref0.segment<3>(6).setZero();
    input_.ref0.segment<3>(9).setZero();
    input_.ref0.segment<2>(0).setZero();
    input_.next_foothold = next_foothold_;
  }
  active_ = true;
  std::printf("[mpc] start (%s): CoM height %.3f m, yaw %.3f rad, feet centroid (%.3f, %.3f)\n",
              stanceOnly() ? "stance only" : ("idle: " + config_.gait.idle).c_str(), height_,
              yaw_anchor_, pos_des_.x(), pos_des_.y());
}

void MpcController::stop() { active_ = false; }

void MpcController::updateCommands(double t, const RobotState& s, const PoseCommand& pose,
                                   const VelocityCommand& vel) {
  const double dt = std::max(t - t_last_, 0.0);
  const auto& m = config_.mpc;
  const auto& c = config_.command;

  if (!stanceOnly()) {
    // Velocity command, rate limited; zero unless the gait runs and is not stopping.
    const bool walk = gaitRunning(t) && gait_stop_ == kNever;
    const double dv = c.lin_accel * dt, dw = c.ang_accel * dt;
    vel_filtered_.vx = approach(vel_filtered_.vx, walk ? vel.vx : 0.0, dv);
    vel_filtered_.vy = approach(vel_filtered_.vy, walk ? vel.vy : 0.0, dv);
    vel_filtered_.wz = approach(vel_filtered_.wz, walk ? vel.wz : 0.0, dw);
  } else {
    const double da = m.angle_rate * dt, dh = m.height_rate * dt;
    pose_filtered_.roll = approach(pose_filtered_.roll, std::clamp(pose.roll, -m.max_angle, m.max_angle), da);
    pose_filtered_.pitch = approach(pose_filtered_.pitch, std::clamp(pose.pitch, -m.max_angle, m.max_angle), da);
    pose_filtered_.yaw = approach(pose_filtered_.yaw, std::clamp(pose.yaw, -m.max_angle, m.max_angle), da);
    pose_filtered_.dz = approach(pose_filtered_.dz, std::clamp(pose.dz, m.dz_min, m.dz_max), dh);
  }

  // Integrate the targets, but never further than max_*_error from the robot.
  yaw_des_ += vel_filtered_.wz * dt;
  const double yaw_target = stanceOnly() ? yaw_anchor_ + pose_filtered_.yaw : yaw_des_;
  yaw_des_ = stanceOnly() ? yaw_target
                          : s.rpy.z() + std::clamp(wrapAngle(yaw_target - s.rpy.z()),
                                                   -m.max_yaw_error, m.max_yaw_error);
  const Vec3 v_world = rotZ(yaw_des_) * Vec3(vel_filtered_.vx, vel_filtered_.vy, 0.0);
  pos_des_ += v_world * dt;
  const Vec3 com = comState(s).segment<3>(3);
  Vec3 err(pos_des_.x() - com.x(), pos_des_.y() - com.y(), 0.0);
  if (err.norm() > m.max_pos_error) err *= m.max_pos_error / err.norm();
  pos_des_ = Vec3(com.x(), com.y(), 0.0) + err;
}

JointCommand MpcController::step(double t, const RobotState& s, const PoseCommand& pose,
                                 const VelocityCommand& vel) {
  updateGait(t, s, vel);
  updateCommands(t, s, pose, vel);
  t_last_ = t;

  // ---- legs: gait phase, swing planning ----
  FootholdPlanner::Input fin;
  fin.base_pos = s.pos;
  fin.yaw = s.rpy.z();
  fin.vel = s.vel;
  fin.vel_cmd = rotZ(s.rpy.z()) * Vec3(vel_filtered_.vx, vel_filtered_.vy, 0.0);
  fin.wz_cmd = vel_filtered_.wz;
  fin.height = height_;

  std::array<bool, kNumLegs> stance{};
  for (int leg = 0; leg < kNumLegs; ++leg) {
    stance[leg] = inContact(leg, t, gait_start_, gait_stop_);
    if (!gaitRunning(t)) {
      next_foothold_[leg] = s.foot_world[leg];
      was_stance_[leg] = true;
      continue;
    }
    const auto ph = gait_.leg(leg, t);
    if (was_stance_[leg] && !ph.stance) {  // lift-off: swing from where the foot is
      swing_[leg] = SwingTrajectory(s.foot_world[leg], footholds_.plan(leg, fin, ph.remaining),
                                    config_.swing.height, gait_.swingTime());
      swing_t0_[leg] = t - ph.progress * gait_.swingTime();
    }
    was_stance_[leg] = ph.stance;
    if (ph.stance) {
      next_foothold_[leg] = footholds_.plan(leg, fin, ph.remaining + gait_.swingTime());
    } else {
      if (ph.progress < config_.swing.freeze_after) {
        swing_[leg].setTarget(footholds_.plan(leg, fin, ph.remaining));
      }
      next_foothold_[leg] = swing_[leg].target();
    }
  }

  // ---- snapshot for the MPC thread ----
  SrbMpc::StateVec ref0 = SrbMpc::StateVec::Zero();
  ref0[0] = stanceOnly() ? pose_filtered_.roll : 0.0;
  ref0[1] = stanceOnly() ? pose_filtered_.pitch : 0.0;
  ref0[2] = yaw_des_;
  ref0.segment<3>(3) = Vec3(pos_des_.x(), pos_des_.y(), height_ + (stanceOnly() ? pose_filtered_.dz : 0.0));
  ref0[8] = vel_filtered_.wz;
  ref0.segment<3>(9) = rotZ(yaw_des_) * Vec3(vel_filtered_.vx, vel_filtered_.vy, 0.0);
  {
    std::lock_guard<std::mutex> lock(in_mutex_);
    input_.state = s;
    input_.t = t;
    ++input_.seq;
    input_.ref0 = ref0;
    input_.vel_cmd_body = Vec3(vel_filtered_.vx, vel_filtered_.vy, 0.0);
    input_.wz_cmd = vel_filtered_.wz;
    input_.next_foothold = next_foothold_;
    input_.gait_start = gait_start_;
    input_.gait_stop = gait_stop_;
  }

  // ---- stance forces: the stage of the latest solution that matches now ----
  Vec12 f = Vec12::Zero();
  bool have = false;
  {
    std::lock_guard<std::mutex> lock(out_mutex_);
    if (output_.valid && !output_.sol.f.empty()) {
      const int k = std::clamp(static_cast<int>((t - output_.t_state) / config_.mpc.dt), 0,
                               static_cast<int>(output_.sol.f.size()) - 1);
      f = output_.sol.f[k];
      have = true;
    }
  }
  int n_stance = 0;
  for (bool st : stance) n_stance += st ? 1 : 0;
  const double share = n_stance > 0 ? config_.model.mass * config_.model.gravity / n_stance : 0.0;

  // ---- joint commands ----
  JointCommand cmd;
  const Vec3 g_body = s.R.transpose() * Vec3(0.0, 0.0, -config_.model.gravity);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    plan_.stance[leg] = stance[leg];
    if (stance[leg]) {
      Vec3 fl = f.segment<3>(3 * leg);
      // No force from the MPC for a leg the gait has in stance (no solution yet, or a
      // phase boundary between solves): carry an equal share of the weight.
      if (!have || fl.z() <= 0.0) fl = Vec3(0.0, 0.0, share);
      f.segment<3>(3 * leg) = fl;
      const Vec3 tau_g = config_.mpc.stance_leg_gravity
                             ? kin::legGravityTorque(leg, kin::legJoints(s.q, leg), g_body)
                             : Vec3::Zero();
      leg::stance(cmd, leg, s.q, s.R, fl, config_.mpc.stance_kd.segment<3>(3 * leg), tau_g);
      plan_.des_world[leg] = s.foot_world[leg];
      plan_.curve[leg].fill(s.foot_world[leg]);
      continue;
    }
    f.segment<3>(3 * leg).setZero();
    const auto pt = swing_[leg].evaluate(t - swing_t0_[leg]);
    leg::FootTarget target;
    target.pos = s.R.transpose() * (pt.pos - s.pos);
    target.vel = leg::worldToBodyVelocity(pt.vel, s.vel, s.omega_body, s.R, target.pos);
    target.acc = s.R.transpose() * pt.acc;  // base acceleration ignored
    const Vec3 tau_g = config_.swing.gravity_compensation
                           ? kin::legGravityTorque(leg, kin::legJoints(s.q, leg), g_body)
                           : Vec3::Zero();
    leg::swing(cmd, leg, target, config_.swing.kp.segment<3>(3 * leg),
               config_.swing.kd.segment<3>(3 * leg), config_.swing.inertia_feedforward, tau_g);
    plan_.des_world[leg] = pt.pos;
    for (int i = 0; i < FootPlan::kCurvePoints; ++i) {
      plan_.curve[leg][i] = swing_[leg].evaluate(swing_[leg].duration() * i / (FootPlan::kCurvePoints - 1)).pos;
    }
  }
  {
    std::lock_guard<std::mutex> lock(out_mutex_);
    applied_f_ = f;
  }
  return cmd;
}

ContactEstimate MpcController::contact(double t) const {
  ContactEstimate c;
  if (stanceOnly() || !gaitRunning(t)) return c;
  const double w = std::max(config_.estimator.trust_window, 1e-6);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    const auto ph = gait_.leg(leg, t);
    c.trust[leg] = ph.stance ? std::min({1.0, ph.progress / w, (1.0 - ph.progress) / w}) : 0.0;
  }
  return c;
}

bool MpcController::timedOut(double t) const {
  const double limit = config_.mpc.timeout_cycles / config_.mpc.rate_hz;
  std::lock_guard<std::mutex> lock(out_mutex_);
  if (!output_.valid) return t - t_start_ > limit + 1.0 / config_.mpc.rate_hz;
  return t - output_.t_state > limit;
}

MpcStatus MpcController::status(double t) const {
  std::lock_guard<std::mutex> lock(out_mutex_);
  MpcStatus s;
  s.has_solution = output_.valid;
  s.status = output_.sol.status;
  s.iterations = output_.sol.iterations;
  s.solve_ms = output_.sol.solve_ms;
  s.age_s = output_.valid ? t - output_.t_state : 1e9;
  s.solves = output_.solves;
  s.failures = output_.failures;
  s.f = applied_f_;
  s.x = output_.x0;
  s.x_ref = output_.x_ref;
  return s;
}

SrbMpc::Problem MpcController::buildProblem(const Snapshot& snap) const {
  const int N = config_.mpc.horizon;
  const double dt = config_.mpc.dt;
  SrbMpc::Problem pb;

  // Reference: stage 0 from the control thread, propagated with the velocity command.
  pb.x_ref.resize(N + 1);
  SrbMpc::StateVec r = snap.ref0;
  for (int k = 0; k <= N; ++k) {
    pb.x_ref[k] = r;
    r[2] += snap.wz_cmd * dt;
    r.segment<3>(9) = rotZ(r[2]) * snap.vel_cmd_body;
    r.segment<3>(3) += r.segment<3>(9) * dt;
    r[5] = snap.ref0[5];
    r[11] = 0.0;
  }

  pb.x0 = comState(snap.state);
  pb.x0[2] = snap.ref0[2] + wrapAngle(pb.x0[2] - snap.ref0[2]);  // no 2*pi jumps

  // Contacts and foot positions: a leg uses its current foot position until it has
  // swung within the horizon, its next foothold afterwards.
  pb.contact.resize(N);
  pb.foot_world.resize(N);
  std::array<bool, kNumLegs> swung{};
  for (int k = 0; k < N; ++k) {
    const double tk = snap.t + k * dt;
    for (int leg = 0; leg < kNumLegs; ++leg) {
      const bool c = inContact(leg, tk, snap.gait_start, snap.gait_stop);
      if (!c) swung[leg] = true;
      pb.contact[k][leg] = c;
      pb.foot_world[k][leg] = swung[leg] ? snap.next_foothold[leg] : snap.state.foot_world[leg];
    }
  }
  return pb;
}

void MpcController::threadLoop() {
  const long period_ns = static_cast<long>(1e9 / config_.mpc.rate_hz);
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  uint64_t last_seq = 0;

  while (running_) {
    addNs(next, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    if (!active_) continue;

    Snapshot snap;
    {
      std::lock_guard<std::mutex> lock(in_mutex_);
      snap = input_;
    }
    if (snap.seq == last_seq || !snap.state.valid) continue;
    last_seq = snap.seq;

    const SrbMpc::Problem pb = buildProblem(snap);
    SrbMpc::Solution sol = mpc_->solve(pb);

    std::lock_guard<std::mutex> lock(out_mutex_);
    ++output_.solves;
    if (sol.ok) {
      output_.sol = std::move(sol);
      output_.x0 = pb.x0;
      output_.x_ref = pb.x_ref[1];
      output_.t_state = snap.t;
      output_.valid = true;
    } else {
      // Keep applying the previous solution; the watchdog trips if this persists.
      ++output_.failures;
      output_.sol.status = sol.status;
    }
  }
}

}  // namespace go2
