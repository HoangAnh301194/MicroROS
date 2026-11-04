#!/usr/bin/env python3
"""ROS 2 Humble: subscribe to ESP32 /imu/data and publish /cmd_vel at 10 Hz.

The default command is zero, safe for communication-only testing.
Optional parameters:
  --ros-args -p linear_x:=0.2 -p angular_z:=0.1
Nonzero commands are sent only while IMU messages are arriving.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuCmdVelBridge(Node):
    def __init__(self):
        super().__init__("imu_cmd_vel_bridge")

        self.declare_parameter("linear_x", 0.0)
        self.declare_parameter("angular_z", 0.0)
        self.declare_parameter("imu_timeout_s", 0.5)

        # Match ESP32 BEST_EFFORT IMU publisher.
        self.create_subscription(
            Imu, "/imu/data", self.on_imu, qos_profile_sensor_data
        )
        # ROS 2 default is RELIABLE, matching ESP32 cmd_vel subscription.
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.last_imu_monotonic = None
        self.imu_count = 0
        self.cmd_count = 0
        self.last_imu = None

        self.create_timer(0.1, self.publish_cmd_vel)  # 10 Hz
        self.create_timer(1.0, self.report)
        self.report_start = time.monotonic()
        self.prev_imu_count = 0
        self.prev_cmd_count = 0

        self.get_logger().info(
            "Listening on /imu/data (BEST_EFFORT); publishing /cmd_vel at 10 Hz"
        )

    def on_imu(self, msg: Imu):
        self.last_imu = msg
        self.last_imu_monotonic = time.monotonic()
        self.imu_count += 1

    def publish_cmd_vel(self):
        # Always publish at 10 Hz for transport verification.
        # Stay at zero if ESP32 is not sending recent IMU data.
        cmd = Twist()
        timeout = max(
            0.0, float(self.get_parameter("imu_timeout_s").value)
        )
        is_fresh = (
            self.last_imu_monotonic is not None
            and time.monotonic() - self.last_imu_monotonic <= timeout
        )
        if is_fresh:
            cmd.linear.x = float(self.get_parameter("linear_x").value)
            cmd.angular.z = float(self.get_parameter("angular_z").value)

        self.cmd_publisher.publish(cmd)
        self.cmd_count += 1

    def report(self):
        now = time.monotonic()
        elapsed = max(0.001, now - self.report_start)
        rx_rate = (self.imu_count - self.prev_imu_count) / elapsed
        tx_rate = (self.cmd_count - self.prev_cmd_count) / elapsed

        if self.last_imu is None:
            detail = "no IMU received; sending zero Twist"
        else:
            detail = (
                f"latest accel.z={self.last_imu.linear_acceleration.z:.2f} "
                "m/s^2"
            )

        self.get_logger().info(
            f"IMU RX={rx_rate:.1f} Hz | cmd_vel TX={tx_rate:.1f} Hz | {detail}"
        )
        self.prev_imu_count = self.imu_count
        self.prev_cmd_count = self.cmd_count
        self.report_start = now


def main():
    rclpy.init()
    node = ImuCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
