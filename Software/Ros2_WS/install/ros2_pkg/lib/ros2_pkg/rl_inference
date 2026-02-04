#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3
import numpy as np
import torch
import torch.nn as nn
import threading
import time
import subprocess
import os

# ==============================================================================
# MẠNG ACTOR - CHỈ GIỮ LẠI PHẦN SUY LUẬN
# ==============================================================================
class ActorNet(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorNet, self).__init__()
        # Cấu trúc phải khớp hoàn toàn với file train để load được best.pt
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(),
            nn.Linear(256, action_dim), nn.Tanh()
        )

    def forward(self, state):
        # Lấy trực tiếp giá trị Mean để robot không bị rung lắc ngẫu nhiên
        return self.actor(state)

# ==============================================================================
# NODE SUY LUẬN (INFERENCE)
# ==============================================================================
class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')
        
        # Đường dẫn tới trọng số tốt nhất của bạn
        user_home = os.path.expanduser("~")
        self.model_path = os.path.join(user_home, "Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights/best.pt")
        
        # ROS 2 Publishers & Subscribers
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        
        # Khởi tạo mạng và nạp trọng số
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.actor_net = ActorNet(state_dim=6, action_dim=9).to(self.device)
        self.load_best_weights()
        self.actor_net.eval() # Chuyển sang chế độ chạy (Inference)

        # Biến trạng thái
        self.current_obs = np.zeros(3)
        self.prev_obs = np.zeros(3)
        self.data_received = False
        self.is_falling = False
        
        # Bounds thông số (Giữ nguyên như lúc bạn Train)
        self.param_mins = np.array([0.05, 0.10, 60.0, 8.0, 30.0, 12.0, 0.01, 0.15, 0.05])
        self.param_maxs = np.array([0.3, 0.35, 100.0, 15.0, 40.0, 25.0, 0.3, 0.35, 0.25])
        
        # Bắt đầu luồng chạy chính
        self.running = True
        threading.Thread(target=self.main_loop, daemon=True).start()

    def load_best_weights(self):
        if os.path.exists(self.model_path):
            # Chỉ load phần actor từ file best.pt
            full_state_dict = torch.load(self.model_path, map_location=self.device)
            # Lọc ra các trọng số của actor để tránh lỗi mismatch nếu file best.pt chứa cả critic
            actor_state_dict = {k.replace('actor.', ''): v for k, v in full_state_dict.items() if k.startswith('actor.')}
            if not actor_state_dict: # Trường hợp file chỉ lưu mỗi actor
                self.actor_net.actor.load_state_dict(full_state_dict)
            else:
                self.actor_net.actor.load_state_dict(actor_state_dict)
            print(f"✅ Đã tải trọng số từ: {self.model_path}")
        else:
            print(f"❌ KHÔNG tìm thấy file trọng số tại: {self.model_path}")

    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True
        # Ngưỡng ngã 45 độ
        if abs(msg.x) > 45.0 or abs(msg.y) > 45.0:
            self.is_falling = True

    def get_state_vector(self):
        curr = self.current_obs
        delta = curr - self.prev_obs
        # Chuẩn hóa đầu vào tương tự như file train
        return np.concatenate([curr/np.array([45.0, 45.0, 1.0]), 
                               delta/np.array([10.0, 10.0, 0.2])]).astype(np.float32)

    def run_gz_command(self, service, req_type, req_data):
        cmd = ['gz', 'service', '-s', service, '--reqtype', req_type, 
               '--reptype', 'gz.msgs.Boolean', '--timeout', '1000', '--req', req_data]
        try:
            result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
            return result.returncode == 0
        except:
            return False

    def sync_reset(self):
        """Hàm reset đồng bộ hoàn toàn bằng vòng lặp while"""
        print("🔄 Đang Reset Robot...")
        self.reset_pub.publish(Bool(data=True))
        
        # Dọn dẹp Shared Memory cho CycloneDDS
        os.system("ipcs -m | awk '{print $2}' | xargs -rn1 ipcrm -m > /dev/null 2>&1")
        
        # Ép dừng mô phỏng
        while not self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true'):
            time.sleep(0.1)

        # Đưa robot về vị trí cân bằng
        z_reset = (205.0 + 57.0 + 18.0 + 15.0) / 1000.0
        pose_msg = f'name: "humanoid_robot" position {{ x: 0.0 y: 0.0 z: {z_reset:.3f} }} orientation {{ x: 0.0 y: 0.0 z: 0.0 w: 1.0 }}'
        while not self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', pose_msg):
            time.sleep(0.1)
        
        # Kích hoạt lại mô phỏng
        while not self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false'):
            time.sleep(0.1)

        self.reset_pub.publish(Bool(data=False))
        self.is_falling = False
        self.data_received = False
        print("✅ Robot đã sẵn sàng!")

    def main_loop(self):
        # Thông số khởi động an toàn
        warmup_params = [0.01, 0.01, 100.0, 10.0, 25.0, 5.0, 0.1, 0.2, 0.05]
        
        while self.running:
            self.sync_reset()
            time.sleep(0.5)
            
            # Gửi thông số warmup
            for _ in range(5):
                self.param_pub.publish(Float64MultiArray(data=warmup_params))
                time.sleep(0.1)

            step_count = 0
            while not self.is_falling:
                if not self.data_received:
                    time.sleep(0.001)
                    continue
                
                self.data_received = False
                state = self.get_state_vector()
                
                with torch.no_grad():
                    state_tensor = torch.as_tensor(state, device=self.device).unsqueeze(0)
                    action = self.actor_net(state_tensor).cpu().numpy().flatten()
                
                # Chuyển đổi action [-1, 1] sang dải tham số thực tế
                phys_act = self.param_mins + (np.clip(action, -1, 1) + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                self.param_pub.publish(Float64MultiArray(data=list(phys_act)))
                
                step_count += 1
                if step_count % 1000 == 0:
                    print(f"⏱️ Đang đứng vững: {step_count} steps")

def main():
    rclpy.init()
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Đã dừng Inference.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()