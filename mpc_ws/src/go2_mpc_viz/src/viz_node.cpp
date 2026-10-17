// go2_mpc_viz: receives go2_mpc's UDP debug stream and republishes it on ROS 2.
//
// Runs only on the development PC. go2_mpc itself never links ROS, so the same
// controller binary runs on the robot's Jetson; point its debug.host at this PC
// to watch the real robot the same way as the simulator.
//
// Publishes:
//   /joint_states                   sensor_msgs/JointState
//   /tf  world -> base              packet quaternion + base-origin position (ground
//                                   truth in simulation; fixed height otherwise)
//   /go2_mpc/status                 go2_mpc_msgs/ControllerStatus
//   /go2_mpc/mpc                    go2_mpc_msgs/MpcDebug          (Mpc mode)
//   /go2_mpc/grf                    visualization_msgs/MarkerArray (Mpc mode, force arrows)
//   /go2_mpc/feet                   visualization_msgs/MarkerArray (desired feet + swing curves)

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "go2_mpc/debug_packet.hpp"
#include "go2_mpc_msgs/msg/controller_status.hpp"
#include "go2_mpc_msgs/msg/mpc_debug.hpp"

namespace {

const std::array<const char*, 4> kModeNames = {"Passive", "Stand", "Mpc", "SwingTest"};
constexpr uint8_t kModeMpc = 2;

// Joint names of go2_description, in SDK order.
std::vector<std::string> jointNames() {
  std::vector<std::string> names;
  for (const char* leg : {"FR", "FL", "RR", "RL"}) {
    for (const char* joint : {"hip", "thigh", "calf"}) {
      names.push_back(std::string(leg) + "_" + joint + "_joint");
    }
  }
  return names;
}

template <size_t N>
void copyArray(const float (&src)[N], std::array<float, N>& dst) {
  std::copy(src, src + N, dst.begin());
}

}  // namespace

class VizNode : public rclcpp::Node {
 public:
  VizNode() : Node("go2_mpc_viz") {
    port_ = declare_parameter<int>("port", 9870);
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "base");
    // Height used for the base when no ground-truth position is available (real robot).
    default_height_ = declare_parameter<double>("default_height", 0.3);
    grf_scale_ = declare_parameter<double>("grf_scale", 0.004);  // arrow length [m] per N

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    status_pub_ = create_publisher<go2_mpc_msgs::msg::ControllerStatus>("go2_mpc/status", 10);
    mpc_pub_ = create_publisher<go2_mpc_msgs::msg::MpcDebug>("go2_mpc/mpc", 10);
    grf_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("go2_mpc/grf", 10);
    feet_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("go2_mpc/feet", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_msg_.name = jointNames();
    joint_msg_.position.resize(12);
    joint_msg_.velocity.resize(12);
    joint_msg_.effort.resize(12);

    openSocket();
    rx_thread_ = std::thread([this] { receiveLoop(); });
    RCLCPP_INFO(get_logger(), "listening for go2_mpc debug packets on UDP %d", port_);
  }

  ~VizNode() override {
    running_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

 private:
  void openSocket() {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("socket() failed");
    int yes = 1;
    ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    timeval tv{0, 100000};  // wake up every 100 ms to notice shutdown
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      throw std::runtime_error("bind() to UDP port " + std::to_string(port_) + " failed");
    }
  }

  void receiveLoop() {
    go2::DebugPacket p;
    while (running_ && rclcpp::ok()) {
      const ssize_t n = ::recv(fd_, &p, sizeof(p), 0);
      if (n < 0) continue;  // timeout
      if (n != static_cast<ssize_t>(sizeof(p)) || p.magic != go2::kDebugPacketMagic ||
          p.version != go2::kDebugPacketVersion || p.size != sizeof(p)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "dropping packet: %zd bytes, version %u (expected %zu bytes, v%u). "
                             "Rebuild go2_mpc and go2_mpc_viz together.",
                             n, p.version, sizeof(p), go2::kDebugPacketVersion);
        continue;
      }
      if (last_seq_ != 0 && p.seq > last_seq_ + 1) dropped_ += p.seq - last_seq_ - 1;
      last_seq_ = p.seq;
      publish(p);
    }
  }

  void publish(const go2::DebugPacket& p) {
    const rclcpp::Time stamp = now();

    joint_msg_.header.stamp = stamp;
    for (int i = 0; i < 12; ++i) {
      joint_msg_.position[i] = p.q[i];
      joint_msg_.velocity[i] = p.dq[i];
      joint_msg_.effort[i] = p.tau_est[i];
    }
    joint_pub_->publish(joint_msg_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = p.ground_truth_valid ? p.base_pos[0] : 0.0;
    tf.transform.translation.y = p.ground_truth_valid ? p.base_pos[1] : 0.0;
    tf.transform.translation.z = p.ground_truth_valid ? p.base_pos[2] : default_height_;
    tf.transform.rotation.w = p.quat[0];
    tf.transform.rotation.x = p.quat[1];
    tf.transform.rotation.y = p.quat[2];
    tf.transform.rotation.z = p.quat[3];
    tf_->sendTransform(tf);

    go2_mpc_msgs::msg::ControllerStatus s;
    s.header.stamp = stamp;
    s.header.frame_id = base_frame_;
    s.mode = p.mode;
    s.mode_name = p.mode < kModeNames.size() ? kModeNames[p.mode] : "?";
    s.seq = p.seq;
    s.controller_time = p.t;
    s.loop_period_ms = p.loop_period_ms;
    s.loop_compute_ms = p.loop_compute_ms;
    s.lowstate_age_ms = p.lowstate_age_ms;
    s.cmd_vel.linear.x = p.cmd_vel[0];
    s.cmd_vel.linear.y = p.cmd_vel[1];
    s.cmd_vel.angular.z = p.cmd_vel[2];
    s.ground_truth_valid = p.ground_truth_valid;
    s.base_lin_vel_world.x = p.base_vel[0];
    s.base_lin_vel_world.y = p.base_vel[1];
    s.base_lin_vel_world.z = p.base_vel[2];
    s.base_ang_vel_body.x = p.gyro[0];
    s.base_ang_vel_body.y = p.gyro[1];
    s.base_ang_vel_body.z = p.gyro[2];
    s.base_lin_acc_body.x = p.acc[0];
    s.base_lin_acc_body.y = p.acc[1];
    s.base_lin_acc_body.z = p.acc[2];
    rpyFromQuat(p.quat, s.rpy);
    copyArray(p.tau_est, s.tau_est);
    copyArray(p.q_des, s.q_des);
    copyArray(p.dq_des, s.dq_des);
    copyArray(p.kp, s.kp);
    copyArray(p.kd, s.kd);
    copyArray(p.tau_ff, s.tau_ff);
    s.kalman_in_use = p.kalman_in_use;
    s.ground_truth_available = p.gt_valid;
    auto vec = [](const float (&a)[3]) {
      geometry_msgs::msg::Vector3 v;
      v.x = a[0];
      v.y = a[1];
      v.z = a[2];
      return v;
    };
    s.kf_pos = vec(p.kf_pos);
    s.kf_vel = vec(p.kf_vel);
    s.gt_pos = vec(p.gt_pos);
    s.gt_vel = vec(p.gt_vel);
    copyArray(p.contact_trust, s.contact_trust);
    status_pub_->publish(s);

    if (p.mode == kModeMpc) {
      publishMpc(p, stamp);
      grf_shown_ = true;
    } else if (grf_shown_) {
      visualization_msgs::msg::MarkerArray clear;
      clear.markers.emplace_back();
      clear.markers.back().action = visualization_msgs::msg::Marker::DELETEALL;
      grf_pub_->publish(clear);
      grf_shown_ = false;
    }

    if (p.plan_valid) {
      publishFeet(p, stamp);
      feet_shown_ = true;
    } else if (feet_shown_) {
      visualization_msgs::msg::MarkerArray clear;
      clear.markers.emplace_back();
      clear.markers.back().action = visualization_msgs::msg::Marker::DELETEALL;
      feet_pub_->publish(clear);
      feet_shown_ = false;
    }

    if (dropped_ > 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%lu debug packets dropped so far",
                           static_cast<unsigned long>(dropped_));
    }
  }

  void publishMpc(const go2::DebugPacket& p, const rclcpp::Time& stamp) {
    go2_mpc_msgs::msg::MpcDebug m;
    m.header.stamp = stamp;
    m.header.frame_id = world_frame_;
    m.has_solution = p.mpc_has_solution;
    m.status = p.mpc_status;
    m.iterations = p.mpc_iterations;
    m.solve_ms = p.mpc_solve_ms;
    m.age_ms = p.mpc_age_ms;
    m.solves = p.mpc_solves;
    m.failures = p.mpc_failures;
    for (int i = 0; i < 12; ++i) {
      m.state[i] = p.com_state[i];
      m.reference[i] = p.com_ref[i];
      m.grf[i] = p.grf[i];
      m.foot_world[i] = p.foot_world[i];
    }
    m.cmd_dz = p.pose_cmd[0];
    m.cmd_roll = p.pose_cmd[1];
    m.cmd_pitch = p.pose_cmd[2];
    m.cmd_yaw = p.pose_cmd[3];
    mpc_pub_->publish(m);

    visualization_msgs::msg::MarkerArray arrows;
    for (int leg = 0; leg < 4; ++leg) {
      visualization_msgs::msg::Marker a;
      a.header = m.header;
      a.ns = "grf";
      a.id = leg;
      a.type = visualization_msgs::msg::Marker::ARROW;
      a.action = visualization_msgs::msg::Marker::ADD;
      geometry_msgs::msg::Point from, to;
      from.x = p.foot_world[3 * leg];
      from.y = p.foot_world[3 * leg + 1];
      from.z = p.foot_world[3 * leg + 2];
      to.x = from.x + grf_scale_ * p.grf[3 * leg];
      to.y = from.y + grf_scale_ * p.grf[3 * leg + 1];
      to.z = from.z + grf_scale_ * p.grf[3 * leg + 2];
      a.points = {from, to};
      a.scale.x = 0.012;  // shaft diameter
      a.scale.y = 0.025;  // head diameter
      a.scale.z = 0.03;   // head length
      a.color.r = 1.0;
      a.color.g = 0.45;
      a.color.b = 0.1;
      a.color.a = 1.0;
      arrows.markers.push_back(a);
    }
    grf_pub_->publish(arrows);
  }

  // Desired foot points (green = stance, blue = swing) and the swing curves.
  void publishFeet(const go2::DebugPacket& p, const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray arr;
    for (int leg = 0; leg < 4; ++leg) {
      const bool stance = p.stance_mask & (1u << leg);
      visualization_msgs::msg::Marker dot;
      dot.header.stamp = stamp;
      dot.header.frame_id = world_frame_;
      dot.ns = "foot_des";
      dot.id = leg;
      dot.type = visualization_msgs::msg::Marker::SPHERE;
      dot.pose.position.x = p.foot_des_world[3 * leg];
      dot.pose.position.y = p.foot_des_world[3 * leg + 1];
      dot.pose.position.z = p.foot_des_world[3 * leg + 2];
      dot.pose.orientation.w = 1.0;
      dot.scale.x = dot.scale.y = dot.scale.z = 0.03;
      dot.color.r = 0.1;
      dot.color.g = stance ? 0.9 : 0.4;
      dot.color.b = stance ? 0.2 : 1.0;
      dot.color.a = 0.9;
      arr.markers.push_back(dot);

      visualization_msgs::msg::Marker curve;
      curve.header = dot.header;
      curve.ns = "swing_curve";
      curve.id = leg;
      curve.type = visualization_msgs::msg::Marker::LINE_STRIP;
      curve.pose.orientation.w = 1.0;
      curve.scale.x = 0.005;
      curve.color.r = 0.3;
      curve.color.g = 0.6;
      curve.color.b = 1.0;
      curve.color.a = stance ? 0.0 : 0.9;
      for (int i = 0; i < 10; ++i) {
        geometry_msgs::msg::Point pt;
        pt.x = p.swing_curve[leg][i][0];
        pt.y = p.swing_curve[leg][i][1];
        pt.z = p.swing_curve[leg][i][2];
        curve.points.push_back(pt);
      }
      arr.markers.push_back(curve);
    }
    feet_pub_->publish(arr);
  }

  static void rpyFromQuat(const float (&q)[4], geometry_msgs::msg::Vector3& rpy) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    rpy.x = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    rpy.y = std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0));
    rpy.z = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  int port_;
  std::string world_frame_, base_frame_;
  double default_height_;
  double grf_scale_;
  bool grf_shown_ = false;
  bool feet_shown_ = false;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<go2_mpc_msgs::msg::ControllerStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<go2_mpc_msgs::msg::MpcDebug>::SharedPtr mpc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr feet_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  sensor_msgs::msg::JointState joint_msg_;

  int fd_ = -1;
  std::atomic<bool> running_{true};
  std::thread rx_thread_;
  uint64_t last_seq_ = 0;
  uint64_t dropped_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VizNode>());
  rclcpp::shutdown();
  return 0;
}
