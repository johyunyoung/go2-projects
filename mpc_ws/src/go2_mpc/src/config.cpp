#include "go2_mpc/config.hpp"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace go2 {
namespace {

template <typename T>
void read(const YAML::Node& node, const char* key, T& value) {
  if (node && node[key]) value = node[key].as<T>();
}

// Accepts either 12 values, or 3 values (hip, thigh, calf) repeated for every leg.
Vec12 toVec12(const YAML::Node& node, const std::string& what) {
  const auto v = node.as<std::vector<double>>();
  Vec12 value;
  if (v.size() == kNumJoints) {
    for (int i = 0; i < kNumJoints; ++i) value[i] = v[i];
  } else if (v.size() == 3) {
    for (int i = 0; i < kNumJoints; ++i) value[i] = v[i % 3];
  } else {
    throw std::runtime_error("config: '" + what + "' needs 3 or 12 values");
  }
  return value;
}

void readVec12(const YAML::Node& node, const char* key, Vec12& value) {
  if (node && node[key]) value = toVec12(node[key], key);
}

}  // namespace

Config loadConfig(const std::string& path) {
  Config c;
  // Go2 motor limits (go2.xml ctrlrange): hip/thigh 23.7 Nm, calf 45.43 Nm.
  for (int l = 0; l < kNumLegs; ++l) c.safety.tau_max.segment<3>(3 * l) << 23.7, 23.7, 45.43;

  const YAML::Node root = YAML::LoadFile(path);

  const auto robot = root["robot"];
  read(robot, "network", c.robot.network);
  read(robot, "domain_id", c.robot.domain_id);
  read(robot, "use_ground_truth", c.robot.use_ground_truth);
  read(robot, "estimator", c.robot.estimator);
  if (c.robot.estimator != "ground_truth" && c.robot.estimator != "kalman") {
    throw std::runtime_error("config: robot.estimator must be ground_truth or kalman");
  }
  if (c.robot.estimator == "ground_truth" && !c.robot.use_ground_truth) {
    throw std::runtime_error("config: robot.estimator ground_truth needs use_ground_truth");
  }

  const auto est = root["estimator"];
  read(est, "process_p", c.estimator.process_p);
  read(est, "process_v", c.estimator.process_v);
  read(est, "process_foot", c.estimator.process_foot);
  read(est, "meas_p_rel", c.estimator.meas_p_rel);
  read(est, "meas_v", c.estimator.meas_v);
  read(est, "meas_z", c.estimator.meas_z);
  read(est, "untrusted_scale", c.estimator.untrusted_scale);
  read(est, "trust_window", c.estimator.trust_window);
  read(est, "foot_rolling", c.estimator.foot_rolling);

  const auto control = root["control"];
  read(control, "dt", c.control.dt);
  read(control, "realtime", c.control.realtime);
  read(control, "cpu", c.control.cpu);
  read(control, "priority", c.control.priority);

  const auto model = root["model"];
  read(model, "mass", c.model.mass);
  read(model, "gravity", c.model.gravity);

  readVec12(root["passive"], "kd", c.passive.kd);

  const auto stand = root["stand"];
  if (!stand || !stand["waypoints"] || !stand["durations"]) {
    throw std::runtime_error("config: stand.waypoints and stand.durations are required");
  }
  for (const auto& wp : stand["waypoints"]) c.stand.waypoints.push_back(toVec12(wp, "waypoints"));
  c.stand.durations = stand["durations"].as<std::vector<double>>();
  readVec12(stand, "kp", c.stand.kp);
  readVec12(stand, "kd", c.stand.kd);
  read(stand, "report_delay", c.stand.report_delay);
  read(stand, "gravity_compensation", c.stand.gravity_compensation);
  if (c.stand.waypoints.empty() || c.stand.waypoints.size() != c.stand.durations.size()) {
    throw std::runtime_error("config: stand.waypoints and stand.durations must match in length");
  }
  for (double d : c.stand.durations) {
    if (d <= 0.0) throw std::runtime_error("config: stand.durations must be positive");
  }

  const auto mpc = root["mpc"];
  read(mpc, "rate_hz", c.mpc.rate_hz);
  read(mpc, "dt", c.mpc.dt);
  read(mpc, "horizon", c.mpc.horizon);
  if (mpc && mpc["inertia"]) {
    const auto v = mpc["inertia"].as<std::vector<double>>();
    if (v.size() != 9) throw std::runtime_error("config: mpc.inertia needs 9 values (row-major)");
    c.mpc.inertia = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(v.data());
  }
  if (mpc && mpc["com_offset"]) {
    const auto v = mpc["com_offset"].as<std::vector<double>>();
    if (v.size() != 3) throw std::runtime_error("config: mpc.com_offset needs 3 values");
    c.mpc.com_offset = Vec3(v[0], v[1], v[2]);
  }
  read(mpc, "mu", c.mpc.mu);
  read(mpc, "fz_min", c.mpc.fz_min);
  read(mpc, "fz_max", c.mpc.fz_max);
  if (mpc && mpc["q_weights"]) {
    const auto v = mpc["q_weights"].as<std::vector<double>>();
    if (v.size() != 12) throw std::runtime_error("config: mpc.q_weights needs 12 values");
    for (int i = 0; i < 12; ++i) c.mpc.q_weights[i] = v[i];
  }
  read(mpc, "r_weight", c.mpc.r_weight);
  read(mpc, "max_iter", c.mpc.max_iter);
  read(mpc, "tol", c.mpc.tol);
  read(mpc, "timeout_cycles", c.mpc.timeout_cycles);
  readVec12(mpc, "stance_kd", c.mpc.stance_kd);
  read(mpc, "stance_leg_gravity", c.mpc.stance_leg_gravity);
  read(mpc, "roll_scale", c.mpc.roll_scale);
  read(mpc, "pitch_scale", c.mpc.pitch_scale);
  read(mpc, "yaw_scale", c.mpc.yaw_scale);
  read(mpc, "max_angle", c.mpc.max_angle);
  read(mpc, "dz_min", c.mpc.dz_min);
  read(mpc, "dz_max", c.mpc.dz_max);
  read(mpc, "angle_rate", c.mpc.angle_rate);
  read(mpc, "height_rate", c.mpc.height_rate);
  read(mpc, "max_pos_error", c.mpc.max_pos_error);
  read(mpc, "max_yaw_error", c.mpc.max_yaw_error);
  read(mpc, "stance_only", c.mpc.stance_only);
  if (c.mpc.rate_hz <= 0 || c.mpc.dt <= 0 || c.mpc.horizon < 1) {
    throw std::runtime_error("config: mpc.rate_hz, mpc.dt and mpc.horizon must be positive");
  }

  const auto gait = root["gait"];
  read(gait, "type", c.gait.type);
  read(gait, "idle", c.gait.idle);
  read(gait, "stand_before", c.gait.stand_before);
  read(gait, "start_speed", c.gait.start_speed);
  read(gait, "start_yaw_rate", c.gait.start_yaw_rate);
  read(gait, "stop_delay", c.gait.stop_delay);
  read(gait, "stop_speed", c.gait.stop_speed);
  if (c.gait.type != "trot") throw std::runtime_error("config: gait.type must be trot");
  if (c.gait.idle != "stand" && c.gait.idle != "step") {
    throw std::runtime_error("config: gait.idle must be stand or step");
  }
  read(gait, "period", c.gait.period);
  read(gait, "duty", c.gait.duty);
  if (gait && gait["offsets"]) {
    const auto v = gait["offsets"].as<std::vector<double>>();
    if (v.size() != kNumLegs) throw std::runtime_error("config: gait.offsets needs 4 values");
    for (int i = 0; i < kNumLegs; ++i) c.gait.offsets[i] = v[i];
  }
  if (c.gait.period <= 0 || c.gait.duty <= 0 || c.gait.duty >= 1) {
    throw std::runtime_error("config: gait.period > 0 and 0 < gait.duty < 1 required");
  }

  const auto foothold = root["foothold"];
  read(foothold, "k_v", c.foothold.k_v);
  read(foothold, "max_step", c.foothold.max_step);

  const auto swing = root["swing"];
  read(swing, "height", c.swing.height);
  read(swing, "freeze_after", c.swing.freeze_after);
  readVec12(swing, "kp", c.swing.kp);
  readVec12(swing, "kd", c.swing.kd);
  read(swing, "gravity_compensation", c.swing.gravity_compensation);
  read(swing, "inertia_feedforward", c.swing.inertia_feedforward);

  const auto swing_test = root["swing_test"];
  read(swing_test, "enable", c.swing_test.enable);
  read(swing_test, "speed", c.swing_test.speed);
  read(swing_test, "ramp", c.swing_test.ramp);
  read(swing_test, "report_period", c.swing_test.report_period);

  const auto command = root["command"];
  read(command, "scale", c.command.scale);
  read(command, "vx_max", c.command.vx_max);
  read(command, "vy_max", c.command.vy_max);
  read(command, "wz_max", c.command.wz_max);
  read(command, "lin_accel", c.command.lin_accel);
  read(command, "ang_accel", c.command.ang_accel);

  const auto safety = root["safety"];
  readVec12(safety, "tau_max", c.safety.tau_max);
  read(safety, "lowstate_timeout", c.safety.lowstate_timeout);
  read(safety, "max_tilt", c.safety.max_tilt);

  const auto noise = root["sim_noise"];
  auto readVec3 = [](const YAML::Node& n, const char* key, Vec3& v) {
    if (!n || !n[key]) return;
    const auto x = n[key].as<std::vector<double>>();
    if (x.size() != 3) throw std::runtime_error(std::string("config: sim_noise.") + key + " needs 3 values");
    v = Vec3(x[0], x[1], x[2]);
  };
  read(noise, "scale", c.sim_noise.scale);
  read(noise, "seed", c.sim_noise.seed);
  read(noise, "gyro_std", c.sim_noise.gyro_std);
  readVec3(noise, "gyro_bias", c.sim_noise.gyro_bias);
  read(noise, "acc_std", c.sim_noise.acc_std);
  readVec3(noise, "acc_bias", c.sim_noise.acc_bias);
  if (noise && noise["attitude_bias"]) {
    const auto x = noise["attitude_bias"].as<std::vector<double>>();
    if (x.size() != 2) throw std::runtime_error("config: sim_noise.attitude_bias needs 2 values");
    c.sim_noise.attitude_bias = {x[0], x[1]};
  }
  read(noise, "attitude_std", c.sim_noise.attitude_std);
  read(noise, "dq_std", c.sim_noise.dq_std);

  const auto debug = root["debug"];
  read(debug, "enable", c.debug.enable);
  read(debug, "host", c.debug.host);
  read(debug, "port", c.debug.port);
  read(debug, "decimation", c.debug.decimation);
  read(debug, "print_period", c.debug.print_period);

  if (c.control.dt <= 0.0) throw std::runtime_error("config: control.dt must be positive");
  if (c.debug.decimation < 1) throw std::runtime_error("config: debug.decimation must be >= 1");
  return c;
}

}  // namespace go2
