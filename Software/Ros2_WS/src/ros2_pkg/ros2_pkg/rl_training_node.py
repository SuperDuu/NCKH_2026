#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from ros_gz_interfaces.srv import ControlWorld # Service để reset Gazebo
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import threading
import time
import subprocess
# --- MẠNG NEURAL ---
class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU()
        )
        self.mu = nn.Linear(256, action_dim)
        self.log_std = nn.Parameter(torch.ones(1, action_dim) * -0.5)

    def forward(self, x):
        x = self.fc(x)
        mu = torch.tanh(self.mu(x)) # Action trong khoảng [-1, 1]
        std = torch.exp(self.log_std)
        return mu, std

class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # 1. ROS Communication
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        
        # Client để Reset môi trường Gazebo
        self.reset_client = self.create_client(ControlWorld, '/world/empty/control')
        
        # 2. State Variables
        self.current_obs = np.array([0.0, 0.0, 0.0]) 
        self.data_received = False
        self.is_falling = False
        
        # 3. RL Hyperparameters
        self.state_dim = 3
        self.action_dim = 7
        self.policy = PolicyNetwork(self.state_dim, self.action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.0003)
        
        # Mapping action thực tế
        self.param_mins = np.array([0.05, 25.0, 5.0, 5.0, 1.0, 0.5, 0.1])
        self.param_maxs = np.array([0.30, 45.0, 20.0, 20.0, 5.0, 5.0, 2.0])

        # 4. Threading
        self.train_thread = threading.Thread(target=self.train_loop)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True
        print(f"DEBUG: Nhan du lieu IMU - Pitch: {msg.x:.2f}, Roll: {msg.y:.2f}")
        # Nếu nghiêng quá 30 độ (Pitch hoặc Roll)
        if abs(msg.x) > 30.0 or abs(msg.y) > 30.0:
            self.is_falling = True

    def reset_simulation(self):
        self.get_logger().info('--- ĐANG RESET ROBOT ---')
        try:
            # Cách 1: Gửi lệnh trực tiếp bằng công cụ của Gazebo (Cực kỳ ổn định)
            subprocess.run([
                "gz", "service", "-s", "/world/empty/control",
                "--reqtype", "gz.msgs.WorldControl",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "500",
                "--req", "reset: {all: true}, pause: false"
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            # Cách 2: Vẫn gọi service ROS2 nếu bridge đã chạy
            req = ControlWorld.Request()
            req.world_control.reset.all = True
            self.reset_client.call_async(req)
            
            self.is_falling = False
            time.sleep(1.0) # Chờ 1 giây để robot rơi xuống sàn và ổn định
        except Exception as e:
            self.get_logger().error(f"Lỗi khi reset: {e}")
    def scale_action(self, action):
        return self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)

    def train_loop(self):
        time.sleep(3) 
        self.get_logger().info("--- HỆ THỐNG RL ĐÃ SẴN SÀNG ---")
        
        for episode in range(5000):
            self.reset_simulation()
            ep_reward = 0
            
            for step in range(500): # Tăng thời gian mỗi episode
                if not self.data_received:
                    time.sleep(0.1)
                    continue

                # Predict Action
                state_tensor = torch.FloatTensor(self.current_obs)
                mu, std = self.policy(state_tensor)
                dist = Normal(mu, std)
                action = dist.sample()
                
                # Publish to C++
                phys_act = self.scale_action(action.detach().numpy())
                msg = Float64MultiArray()
                msg.data = phys_act.tolist()
                self.param_pub.publish(msg)
                
                # Log hành động mỗi 50 bước
                if step % 50 == 0:
                    self.get_logger().info(f"Ep {episode} | Gain: {phys_act[0]:.3f} | Kp: {phys_act[5]:.2f}")

                time.sleep(0.05) # Khớp với 20Hz
                
                # Reward calculation
                pitch, roll, _ = self.current_obs
                reward = 1.0 / (1.0 + 0.1*abs(pitch) + 0.1*abs(roll))
                ep_reward += reward

                # Học từ sai lầm (Policy Gradient đơn giản)
                loss = -dist.log_prob(action) * reward
                self.optimizer.zero_grad()
                loss.mean().backward()
                self.optimizer.step()

                if self.is_falling:
                    self.get_logger().warn(f"Episode {episode} kết thúc - Robot ngã tại bước {step}")
                    break
            
            self.get_logger().info(f"==> Kết quả Ep {episode}: Total Reward = {ep_reward:.2f}")

def main():
    rclpy.init()
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()