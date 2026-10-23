#!/usr/bin/env python3
"""Republish the Go2's /lowstate (unitree_go/msg/LowState, 500 Hz) as standard ROS 2 messages.

    /joint_states   sensor_msgs/JointState      12 joints, SDK order FR_hip .. RL_calf (position, velocity, effort = tau_est)
    /imu            sensor_msgs/Imu             orientation (wxyz -> xyzw), gyroscope, accelerometer
    /foot_force     std_msgs/Float32MultiArray  [FR, FL, RR, RL] raw foot_force
    /foot_contact   std_msgs/Int8MultiArray     1 where foot_force > contact_threshold (default 10, same as the controller log)
    /joint_temperature std_msgs/Float32MultiArray 12 motor temperatures [degC]

Run in the go2:humble container after `source ~/go2_ws/go2-projects/ros2/env.sh`:
    python3 ~/go2_ws/go2-projects/ros2/go2_state_bridge.py --ros-args -p rate:=100.0
Parameters: rate (Hz, output is decimated from the 500 Hz input; 0 = every message), lowstate_topic,
            base_frame, imu_frame, contact_threshold.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from unitree_go.msg import LowState

JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]
FOOT_NAMES = ["FR", "FL", "RR", "RL"]


class Go2StateBridge(Node):
    def __init__(self):
        super().__init__("go2_state_bridge")
        self.declare_parameter("rate", 100.0)
        self.declare_parameter("lowstate_topic", "/lowstate")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("contact_threshold", 10.0)
        rate = float(self.get_parameter("rate").value)
        self.min_period = 1.0 / rate if rate > 0 else 0.0
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.imu_frame = str(self.get_parameter("imu_frame").value)
        self.contact_threshold = float(self.get_parameter("contact_threshold").value)
        self.last_pub = 0.0
        self.n_in = 0
        self.n_out = 0

        # The robot's writer is best-effort-compatible; keep the reader cheap and never block the 500 Hz stream.
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.pub_joint = self.create_publisher(JointState, "joint_states", pub_qos)
        self.pub_imu = self.create_publisher(Imu, "imu", pub_qos)
        self.pub_force = self.create_publisher(Float32MultiArray, "foot_force", pub_qos)
        self.pub_contact = self.create_publisher(Int8MultiArray, "foot_contact", pub_qos)
        self.pub_temp = self.create_publisher(Float32MultiArray, "joint_temperature", pub_qos)
        topic = str(self.get_parameter("lowstate_topic").value)
        self.sub = self.create_subscription(LowState, topic, self.on_lowstate, sub_qos)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"bridging {topic} -> /joint_states /imu /foot_force /foot_contact /joint_temperature at {rate:g} Hz")

    def on_lowstate(self, msg: LowState):
        self.n_in += 1
        now = time.monotonic()
        if self.min_period > 0 and now - self.last_pub < self.min_period:
            return
        self.last_pub = now
        self.n_out += 1
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.header.frame_id = self.base_frame
        js.name = JOINT_NAMES
        ms = msg.motor_state
        js.position = [float(ms[i].q) for i in range(12)]
        js.velocity = [float(ms[i].dq) for i in range(12)]
        js.effort = [float(ms[i].tau_est) for i in range(12)]
        self.pub_joint.publish(js)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        q = msg.imu_state.quaternion  # Unitree order w, x, y, z (verified 2026-10-06, go2_inspect README)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        g = msg.imu_state.gyroscope
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(g[0]), float(g[1]), float(g[2]))
        a = msg.imu_state.accelerometer
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(a[0]), float(a[1]), float(a[2]))
        imu.orientation_covariance[0] = -1.0  # unknown
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub_imu.publish(imu)

        force = [float(f) for f in msg.foot_force[:4]]
        self.pub_force.publish(Float32MultiArray(data=force))
        self.pub_contact.publish(Int8MultiArray(data=[1 if f > self.contact_threshold else 0 for f in force]))
        self.pub_temp.publish(Float32MultiArray(data=[float(ms[i].temperature) for i in range(12)]))

    def report(self):
        self.get_logger().info(f"lowstate in {self.n_in / 5.0:.0f} Hz, out {self.n_out / 5.0:.0f} Hz")
        self.n_in = 0
        self.n_out = 0


def main():
    rclpy.init()
    node = Go2StateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
