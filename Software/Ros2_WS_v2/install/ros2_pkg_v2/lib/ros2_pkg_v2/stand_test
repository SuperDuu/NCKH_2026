#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stand Test Node - Robot đứng yên để kiểm tra ổn định.
Gửi lệnh chân thẳng (Z=0.195) cho humanoid_llc_node thông qua /rl/leg_command.
Robot sẽ đứng yên tại chỗ, không di chuyển.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Vector3
import time
import threading
import numpy as np

STD_Z = 0.195   # Chiều cao đứng chuẩn (m)
STD_Y = 0.01    # Khoảng cách Y giữa chân và hông

class StandTestNode(Node):
    def __init__(self):
        super().__init__('stand_test_node')

        # Publishers
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # Subscriber IMU để theo dõi nghiêng
        self.imu_sub = self.create_subscription(Vector3, '/robot_orientation', self.imu_callback, 10)
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.get_logger().info("=" * 60)
        self.get_logger().info("  STAND TEST NODE - Robot đứng yên kiểm tra ổn định")
        self.get_logger().info(f"  Z = {STD_Z}m | Y = ±{STD_Y}m")
        self.get_logger().info("=" * 60)
        
        # Chạy logic trong thread riêng
        threading.Thread(target=self.stand_loop, daemon=True).start()

    def imu_callback(self, msg):
        self.pitch = msg.x * (np.pi / 180.0)
        self.roll = msg.y * (np.pi / 180.0)
        self.yaw = msg.z * (np.pi / 180.0)

    def stand_loop(self):
        # Đợi hệ thống sẵn sàng
        self.get_logger().info("Đợi 3 giây để Gazebo ổn định...")
        time.sleep(3.0)
        
        # Bỏ reset flag
        self.reset_pub.publish(Bool(data=False))
        time.sleep(0.5)

        # Gửi lệnh đứng yên liên tục
        self.get_logger().info(">>> Bắt đầu gửi lệnh ĐỨNG YÊN <<<")
        
        count = 0
        while rclpy.ok():
            msg = Float64MultiArray()
            # [x_left, y_left, z_left, lift_left, x_right, y_right, z_right, lift_right, duration]
            msg.data = [
                0.0,     STD_Y,  STD_Z, 0.0,   # Chân trái: thẳng, không nhấc
                0.0,    -STD_Y,  STD_Z, 0.0,   # Chân phải: thẳng, không nhấc
                1.0                              # Duration dài (giữ yên)
            ]
            self.action_pub.publish(msg)
            
            # In trạng thái mỗi 2 giây
            count += 1
            if count % 20 == 0:
                roll_deg = np.degrees(self.roll)
                pitch_deg = np.degrees(self.pitch)
                stable = "✅ ỔN ĐỊNH" if (abs(roll_deg) < 5.0 and abs(pitch_deg) < 5.0) else "⚠️ NGHIÊNG"
                self.get_logger().info(
                    f"[{stable}] Roll: {roll_deg:+.2f}° | Pitch: {pitch_deg:+.2f}° | Yaw: {np.degrees(self.yaw):+.2f}°"
                )

            time.sleep(0.1)  # 10Hz


def main():
    rclpy.init()
    node = StandTestNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
