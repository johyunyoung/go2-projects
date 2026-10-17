#pragma once

#include <array>
#include <string>
#include <vector>

#include "go2_mpc/types.hpp"

namespace go2 {

struct Config {
  struct Robot {
    std::string network = "lo";  // DDS interface: lo (simulator) or the robot NIC
    int domain_id = 0;
    bool use_ground_truth = true;  // read rt/sportmodestate (simulator only)
    // Which estimate drives the controller: ground_truth (simulator) | kalman.
    // With ground_truth the Kalman filter still runs alongside for comparison.
    std::string estimator = "ground_truth";
  } robot;

  // KalmanEstimator noise (variances per second for the process, per sample for the
  // measurements; same scaling as the MIT Cheetah estimator).
  struct Estimator {
    double process_p = 0.02;     // base position
    double process_v = 0.02;     // base velocity (accelerometer)
    double process_foot = 0.002; // planted foot position
    double meas_p_rel = 0.001;   // foot position relative to the base (kinematics)
    double meas_v = 0.1;         // base velocity from leg odometry
    double meas_z = 0.001;       // planted foot height
    double untrusted_scale = 100.0;  // noise multiplier for a foot not in contact
    double trust_window = 0.2;   // stance fraction over which trust ramps at both ends
    // The foot is a ball: in stance it rolls, so its centre (the foot point) moves with
    // r * (w_calf x z) while the touching surface point stays still. Without this the
    // leg odometry under-reads the speed by about r / leg length (~6 %).
    bool foot_rolling = true;
  } estimator;

  struct Control {
    double dt = 0.002;      // control period [s]
    bool realtime = false;  // SCHED_FIFO + CPU pinning (needs privileges)
    int cpu = -1;           // CPU to pin the control thread to, -1 = none
    int priority = 80;
  } control;

  // Rigid-body properties shared by the controllers.
  struct Model {
    double mass = 15.206;  // go2.xml: trunk 6.921 + 4 x (hip 0.678 + thigh 1.152 + calf 0.241)
    double gravity = 9.81;
  } model;

  struct Passive {
    Vec12 kd = Vec12::Constant(3.0);
  } passive;

  // Mode 1: joint-space interpolation from the current posture through each waypoint.
  // The last waypoint is the initial pose the MPC starts from.
  struct Stand {
    std::vector<Vec12> waypoints;  // joint targets, SDK order
    std::vector<double> durations; // [s] to reach each waypoint from the previous one
    Vec12 kp = Vec12::Zero();
    Vec12 kd = Vec12::Zero();
    double report_delay = 0.5;     // [s] after the last waypoint, log tracking error
    // Feed-forward joint torque that carries m*g/4 per foot, faded in over the last
    // segment (the body rests on the ground before that). Removes the PD sag tau/kp.
    bool gravity_compensation = true;
  } stand;

  // Mode 2: SRB-MPC.
  struct Mpc {
    double rate_hz = 50.0;
    double dt = 0.02;     // prediction step [s]
    int horizon = 15;
    // Composite inertia of the whole robot about its CoM at the stand pose, base
    // frame (legs are 54 % of the mass, so the trunk alone is far too small).
    Mat3 inertia = (Mat3() << 0.188, 0.0, -0.020, 0.0, 0.499, 0.0, -0.020, 0.0, 0.532).finished();
    Vec3 com_offset{-0.0019, 0.0, -0.0246};  // CoM in the base frame at the stand pose
    double mu = 0.35;
    double fz_min = 10.0;
    double fz_max = 150.0;
    Eigen::Matrix<double, 12, 1> q_weights =
        (Eigen::Matrix<double, 12, 1>() << 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1).finished();
    double r_weight = 1e-6;
    int max_iter = 30;
    double tol = 1e-6;
    int timeout_cycles = 3;          // stale solution -> back to Stand
    Vec12 stance_kd = Vec12::Constant(1.0);  // joint damping on stance legs
    bool stance_leg_gravity = true;  // add the stance leg's own gravity torque
    // Pose command (4-leg stance): remote axis [-1, 1] -> offset, and limits.
    double roll_scale = 0.3, pitch_scale = 0.3, yaw_scale = 0.3;  // [rad] at full stick
    double max_angle = 0.35;         // [rad]
    double dz_min = -0.10, dz_max = 0.05;  // [m]
    double angle_rate = 0.5;         // command rate limit [rad/s]
    double height_rate = 0.1;        // [m/s]
    // Walking: the position/yaw target integrates the velocity command but never
    // runs further than this ahead of the robot.
    double max_pos_error = 0.1;      // [m]
    double max_yaw_error = 0.2;      // [rad]
    bool stance_only = false;        // never walk; sticks = body pose (M3, --stance-only)
  } mpc;

  // Periodic gait (M4: swing test; M5: walking). Legs FR, FL, RR, RL.
  struct Gait {
    std::string type = "trot";      // gait pattern (only trot so far)
    // What the robot does in Mpc mode while the velocity command is zero:
    //   stand: 4-leg stance; the gait starts on a command and stops after it ends
    //   step:  keeps trotting in place (starts stand_before after entering Mpc)
    std::string idle = "stand";
    double stand_before = 1.0;      // [s] minimum 4-leg stance after entering Mpc
    double start_speed = 0.05;      // [m/s] |vx| or |vy| command above this starts the gait
    double start_yaw_rate = 0.1;    // [rad/s] |wz| command above this starts the gait
    double stop_delay = 0.3;        // [s] command must stay below the thresholds this long
    double stop_speed = 0.1;        // [m/s] and the measured speed below this, to stop
    double period = 0.3;
    double duty = 0.5;
    std::array<double, kNumLegs> offsets{0.0, 0.5, 0.5, 0.0};  // trot: FR+RL, FL+RR
  } gait;

  // Swing legs: Bezier trajectory tracked by joint PD (IK) + velocity feed-forward.
  struct Swing {
    double height = 0.08;       // apex above the higher end point [m]
    double freeze_after = 0.8;  // stop moving the touch-down target after this progress
    Vec12 kp = Vec12::Constant(30.0);
    Vec12 kd = Vec12::Constant(2.0);
    bool gravity_compensation = true;  // add the leg's own gravity torque
    bool inertia_feedforward = true;   // add M(q) qdd for the trajectory's acceleration
  } swing;

  // Raibert foothold (see foothold_planner.hpp).
  struct Foothold {
    double k_v = 0.03;       // velocity-error gain [s]
    double max_step = 0.15;  // max offset from the hip at touch-down [m]
  } foothold;

  // M4 bench test: base welded in the air (sim/scene_fixed.xml); `start` runs the gait
  // with every foot following the stance/swing path it would have on the ground
  // while the body moves forward at `speed` in a virtual world.
  struct SwingTest {
    bool enable = false;  // --swing-test
    double speed = 0.5;   // [m/s] virtual body speed
    double ramp = 1.0;    // [s] to reach it
    double report_period = 2.0;
  } swing_test;

  struct Command {
    double scale = 1.0;  // remote axis [-1, 1] -> velocity
    double vx_max = 1.0;
    double vy_max = 0.5;
    double wz_max = 1.0;
    double lin_accel = 1.0;  // command rate limit [m/s^2]
    double ang_accel = 2.0;  // [rad/s^2]
  } command;

  struct Safety {
    Vec12 tau_max;  // per-joint torque limit [Nm]
    double lowstate_timeout = 0.1;  // [s] without rt/lowstate -> passive
    double max_tilt = 0.8;          // [rad] roll or pitch beyond this in Mpc -> passive
  } safety;

  // Simulation only: sensor noise added before estimation (see sim_noise.hpp). Values
  // are for scale 1 (a typical MEMS IMU); --sim-noise SCALE multiplies them.
  struct SimNoise {
    double scale = 0.0;                    // 0 = off
    unsigned seed = 1;
    double gyro_std = 0.005;               // [rad/s]
    Vec3 gyro_bias{0.01, -0.01, 0.01};     // [rad/s]
    double acc_std = 0.04;                 // [m/s^2]
    Vec3 acc_bias{0.1, -0.1, 0.1};         // [m/s^2]
    std::array<double, 2> attitude_bias{0.01, -0.01};  // roll, pitch [rad]
    double attitude_std = 0.002;           // [rad]
    double dq_std = 0.05;                  // [rad/s]
  } sim_noise;

  struct Debug {
    bool enable = true;
    std::string host = "127.0.0.1";
    int port = 9870;
    int decimation = 5;  // send every N control ticks (500 Hz / 5 = 100 Hz)
    double print_period = 2.0;  // loop timing summary on stdout [s], 0 = off
  } debug;
};

// Loads a yaml file; keys that are absent keep the defaults above.
Config loadConfig(const std::string& path);

}  // namespace go2
