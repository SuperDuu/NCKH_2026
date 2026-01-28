#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import numpy as np
import torch
import torch.nn as nn
import os
import sys
from torch.distributions import Normal

# ==========================================
# 1. PPO NETWORK (PHẢI GIỐNG HỆT FILE TRAIN)
# ==========================================
class PPOActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(PPOActorCritic, self).__init__()
        self.action_var = torch.full((action_dim,), action_std_init * action_std_init)
        
        # ACTOR
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        
        # CRITIC
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 1)
        )

    def act(self, state):
        action_mean = self.actor(state)
        return action_mean.detach() # Inference chỉ lấy Mean (tốt nhất), không lấy mẫu ngẫu nhiên

# ==========================================
# 2. INFERENCE NODE
# ==========================================
class RLInferenceNode(Node):
    def __init__(self):
        super().__init__('rl_inference_node')

        # --- LOAD MODEL ---
        self.weight_path = "src/ros2_pkg/weights/best.pt"
        if not os.path.exists(self.weight_path):
            self.weight_path = os.path.join(os.getcwd(), "src/ros2_pkg/weights/best.pt")
            if not os.path.exists(self.weight_path):
                # Thử tìm trong /tmp nếu vừa train xong
                self.weight_path = "/tmp/best_policy.pt"

        self.get_logger().info(f"📂 Loading weights from: {self.weight_path}")

        # Cấu hình mạng (Khớp với PPO Training)
        self.state_dim = 6  # [Pitch, Roll, Phase, dPitch, dRoll, dPhase]
        self.action_dim = 7
        self.policy = PPOActorCritic(self.state_dim, self.action_dim)
        
        try:
            self.policy.load_state_dict(torch.load(self.weight_path, map_location=torch.device('cpu')))
            self.policy.eval()
            self.get_logger().info("✅ BRAIN LOADED SUCCESSFULLY!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load model: {e}")
            # sys.exit(1) # Không exit để tránh crash launch file, chỉ báo lỗi

        # --- ROS SETUP ---
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)

        # Tham số Scale (Khớp file train)
        self.param_mins = np.array([0.05, 0.05, 15.0, 3.0, 20.0, 5.0, 0.01])
        self.param_maxs = np.array([0.50, 0.40, 50.0, 15.0, 60.0, 25.0, 0.40])
        
        # Biến lưu trạng thái cũ để tính đạo hàm (Delta)
        self.current_obs = np.zeros(3)
        self.prev_obs = np.zeros(3)

    def scale_action(self, action):
        return self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)

    def get_state_vector(self, msg):
        # Cập nhật quan sát
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        
        # Tính Delta (Vận tốc thay đổi)
        delta = self.current_obs - self.prev_obs
        
        # Normalize (Giống hệt lúc train)
        norm_obs = self.current_obs / np.array([45.0, 45.0, 1.0])
        norm_delta = delta / np.array([5.0, 5.0, 0.1])
        
        return np.concatenate([norm_obs, norm_delta], axis=0).astype(np.float32)

    def feedback_callback(self, msg):
        # 1. Xử lý đầu vào
        state = self.get_state_vector(msg)
        state_tensor = torch.FloatTensor(state)

        # 2. Chạy qua não (Policy)
        with torch.no_grad():
            action = self.policy.act(state_tensor)
            
        # 3. Scale ra tham số vật lý
        phys_act = self.scale_action(action.numpy())

        # 4. Gửi xuống Robot
        msg_out = Float64MultiArray()
        msg_out.data = list(phys_act)
        self.param_pub.publish(msg_out)

def main():
    rclpy.init()
    node = RLInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()