#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TASK SPACE CONTROL - PPO TRAINING
==================================
Phương pháp điều khiển mới: AI điều khiển TRỰC TIẾP tọa độ bàn chân (X, Y, Z)
thay vì điều chỉnh tham số sóng Sin như cũ.

THAY ĐỔI CHÍNH:
1. Observation: Nhớ 3 frame gần nhất (History Stacking) + Action trước đó
2. Action: [ΔX_L, ΔY_L, ΔZ_L, ΔX_R, ΔY_R, ΔZ_R] - Độ dời bàn chân (mm)
3. Reward: Thưởng đứng thẳng + Phạt nặng thay đổi đột ngột (Smoothness)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import threading
import time
import subprocess
import os
from collections import deque
import csv
from datetime import datetime

# ==============================================================================
# CẤU HÌNH
# ==============================================================================
OBS_FRAME_DIM = 4      # [Roll, Pitch, dRoll, dPitch] mỗi frame
HISTORY_LEN = 3        # Nhớ 3 frame gần nhất
ACTION_DIM = 6         # [ΔX_L, ΔY_L, ΔZ_L, ΔX_R, ΔY_R, ΔZ_R]
STATE_DIM = (OBS_FRAME_DIM * HISTORY_LEN) + ACTION_DIM  # 4*3 + 6 = 18

# Scaling cho action (mm) - Giới hạn robot dịch chuyển mỗi bước
ACTION_SCALE = np.array([
    20.0,  # ΔX_left:  Trước/sau ±20mm
    20.0,  # ΔY_left:  Trái/phải ±20mm
    40.0,  # ΔZ_left:  Lên/xuống ±40mm
    20.0,  # ΔX_right: Trước/sau ±20mm
    20.0,  # ΔY_right: Trái/phải ±20mm
    40.0   # ΔZ_right: Lên/xuống ±40mm
])

# Đường dẫn lưu model
USER_HOME = os.path.expanduser("~")
WEIGHTS_DIR = os.path.join(USER_HOME, "Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights")
MODEL_PATH = os.path.join(WEIGHTS_DIR, "task_space_best.pt")

# Đường dẫn lưu CSV logs
LOGS_DIR = os.path.join(USER_HOME, "Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/logs")
CSV_LOG_PATH = os.path.join(LOGS_DIR, f"training_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

# ==============================================================================
# MẠNG NEURAL - ACTOR-CRITIC
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.3):
        super(ActorCritic, self).__init__()
        
        # Action variance (độ phân tán của phân phối Gaussian)
        self.action_var = nn.Parameter(
            torch.full((action_dim,), action_std_init * action_std_init),
            requires_grad=False
        )
        
        # Actor Network (Policy): State → Action mean
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
            nn.Tanh()  # Output trong khoảng [-1, 1]
        )
        
        # Critic Network (Value Function): State → Value
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 1)
        )
    
    def forward(self):
        raise NotImplementedError
    
    def act(self, state):
        """Chọn action từ policy (dùng khi test hoặc thu thập dữ liệu)"""
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        state_value = self.critic(state)
        
        return action.detach(), action_logprob.detach(), state_value.detach()
    
    def evaluate(self, state, action):
        """Đánh giá action đã thực hiện (dùng khi training)"""
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        
        return action_logprobs, state_values, dist_entropy

# ==============================================================================
# PPO AGENT
# ==============================================================================
class PPOAgent:
    def __init__(self, state_dim, action_dim):
        self.device = torch.device("cpu")  # Dùng CPU cho nhẹ Gazebo
        
        # Policy hiện tại (đang train)
        self.policy = ActorCritic(state_dim, action_dim).to(self.device)
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=0.0001)
        
        # Policy cũ (dùng để thu thập dữ liệu)
        self.policy_old = ActorCritic(state_dim, action_dim).to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()
        
        # Buffer lưu trữ trajectory
        self.buffer = {
            's': [],   # States
            'a': [],   # Actions
            'lp': [],  # Log probabilities
            'r': [],   # Rewards
            'd': []    # Done flags
        }
    
    def update(self):
        """Cập nhật policy bằng PPO algorithm"""
        # 1. Tính Discounted Rewards (Monte Carlo)
        rewards = []
        discounted_reward = 0
        gamma = 0.99  # Discount factor
        
        for reward, is_terminal in zip(reversed(self.buffer['r']), reversed(self.buffer['d'])):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        # Chuẩn hóa rewards (giúp training ổn định hơn)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-8)
        
        # Chuyển buffer sang tensor
        old_states = torch.stack(self.buffer['s']).detach()
        old_actions = torch.stack(self.buffer['a']).detach()
        old_logprobs = torch.stack(self.buffer['lp']).detach()
        
        # 2. Optimize policy với PPO (10 epochs)
        for _ in range(10):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            
            # Importance sampling ratio
            ratios = torch.exp(logprobs - old_logprobs)
            
            # Advantages
            advantages = rewards - state_values.detach().squeeze()
            
            # PPO Clipped Surrogate Loss
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 0.8, 1.2) * advantages
            
            # Tổng loss
            loss = -torch.min(surr1, surr2) + \
                   0.5 * self.MseLoss(state_values.squeeze(), rewards) - \
                   0.01 * dist_entropy
            
            # Backpropagation
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        
        # 3. Copy policy sang policy_old
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        # 4. Xóa buffer
        for k in self.buffer:
            self.buffer[k].clear()

# ==============================================================================
# ROS2 NODE - TRAINING
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # Tạo thư mục lưu weights
        os.makedirs(WEIGHTS_DIR, exist_ok=True)
        
        # Tạo thư mục lưu logs
        os.makedirs(LOGS_DIR, exist_ok=True)
        
        # Khởi tạo CSV logger
        self.csv_file = open(CSV_LOG_PATH, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Episode', 'Reward', 'Steps', 'Best_Reward', 'Avg_Roll', 'Avg_Pitch',
            'Max_Roll', 'Max_Pitch', 'Smoothness_Score', 'Fall_Count', 'Timestamp'
        ])
        self.csv_file.flush()
        
        # =================== PUBLISHERS ===================
        self.action_pub = self.create_publisher(
            Float64MultiArray, '/uvc_rl_action', 10
        )
        self.reset_pub = self.create_publisher(
            Bool, '/uvc_reset', 10
        )
        
        # Publishers cho reset khớp (backup)
        self.joint_pubs = {}
        joint_names = [
            'hip_knee_left_joint', 'hip_knee_right_joint',
            'knee_ankle_left_joint', 'knee_ankle_right_joint',
            'hip_hip_left_joint', 'hip_hip_right_joint'
        ]
        for j in joint_names:
            self.joint_pubs[j] = self.create_publisher(
                Float64, f'/model/humanoid_robot/joint/{j}/cmd_pos', 10
            )
        
        # =================== SUBSCRIBERS ===================
        self.imu_sub = self.create_subscription(
            Vector3, '/robot_orientation',
            self.imu_callback, 10
        )
        
        # =================== BIẾN TRẠNG THÁI ===================
        self.obs_history = deque(maxlen=HISTORY_LEN)  # Lịch sử observation
        self.prev_action = np.zeros(ACTION_DIM)       # Action trước đó
        
        # IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.roll_prev = 0.0
        self.pitch_prev = 0.0
        
        # Flags
        self.is_falling = False
        self.data_received = False
        
        # =================== PPO AGENT ===================
        self.ppo = PPOAgent(STATE_DIM, ACTION_DIM)
        self.best_reward = -99999
        
        # =================== METRICS TRACKING ===================
        self.roll_history = []
        self.pitch_history = []
        self.action_smoothness = []
        self.fall_count = 0
        
        # =================== TRAINING THREAD ===================
        self.thread = threading.Thread(target=self.train_loop, daemon=True)
        self.thread.start()
        
        print("╔════════════════════════════════════════╗", flush=True)
        print("║  🤖 RL TRAINING NODE INITIALIZED       ║", flush=True)
        print("║  📊 State Dim:  {:2d}                     ║".format(STATE_DIM), flush=True)
        print("║  🎯 Action Dim: {:2d}                     ║".format(ACTION_DIM), flush=True)
        print("║  📚 History:    {:2d} frames              ║".format(HISTORY_LEN), flush=True)
        print("╚════════════════════════════════════════╝", flush=True)
    
    # =================== IMU CALLBACK ===================
    def imu_callback(self, msg):
        """Nhận dữ liệu IMU từ Gazebo"""
        self.data_received = True
        
        # Lưu giá trị trước đó
        self.roll_prev = self.roll
        self.pitch_prev = self.pitch
        
        # Cập nhật giá trị mới (đơn vị: radian)
        self.roll = msg.x
        self.pitch = msg.y
        
        # Kiểm tra ngã (góc nghiêng > ~40 độ)
        if abs(self.roll) > 0.7 or abs(self.pitch) > 0.7:
            self.is_falling = True
    
    # =================== GET STATE ===================
    def get_state(self):
        """
        Tạo vector state từ:
        - 3 frame observation gần nhất (mỗi frame: [Roll, Pitch, dRoll, dPitch])
        - Action trước đó
        """
        # Tính đạo hàm góc (tốc độ góc xấp xỉ)
        d_roll = self.roll - self.roll_prev
        d_pitch = self.pitch - self.pitch_prev
        
        # Frame hiện tại
        current_frame = np.array([self.roll, self.pitch, d_roll, d_pitch])
        
        # Khởi tạo history nếu chưa đủ
        while len(self.obs_history) < HISTORY_LEN:
            self.obs_history.append(current_frame)
        
        # Thêm frame mới
        self.obs_history.append(current_frame)
        
        # Ghép: [frame_1, frame_2, frame_3, prev_action]
        state = np.concatenate([
            np.concatenate(self.obs_history),  # 3 frames = 12 số
            self.prev_action                    # 6 số
        ]).astype(np.float32)
        
        return state  # Shape: (18,)
    
    # =================== GAZEBO SERVICE HELPER ===================
    def run_gz(self, service, req_type, data):
        """Gọi Gazebo service (pause, unpause, set_pose, v.v.)"""
        cmd = [
            'gz', 'service',
            '-s', service,
            '--reqtype', req_type,
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', data
        ]
        result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return result.returncode == 0
    
    # =================== RESET "BẤT TỬ" ===================
    def reset_simulation_physics(self):
        """
        Reset Gazebo về trạng thái ban đầu
        Lặp cho tới khi thành công (Bất tử)
        """
        print("🔄 Resetting simulation...", end="", flush=True)
        
        # Bước 1: Xóa IPC (Shared Memory) - Fix lỗi dính khớp
        os.system("ipcs -m | awk '{print $2}' | xargs -rn1 ipcrm -m > /dev/null 2>&1")
        
        # Bước 2: PAUSE simulation
        while not self.run_gz('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true'):
            time.sleep(0.5)
        
        # Bước 3: Teleport robot về vị trí ban đầu
        pose_cmd = 'name: "humanoid_robot" position { z: 0.42 } orientation { w: 1.0 }'
        while not self.run_gz('/world/empty/set_pose', 'gz.msgs.Pose', pose_cmd):
            time.sleep(0.5)
        
        # Bước 4: Set góc khớp ban đầu (tư thế đứng)
        init_joints = {
            'hip_knee_left_joint': 0.3,
            'hip_knee_right_joint': 0.3,
            'knee_ankle_left_joint': -0.15,
            'knee_ankle_right_joint': -0.15
        }
        for joint_name, angle in init_joints.items():
            self.joint_pubs[joint_name].publish(Float64(data=angle))
        
        # Bước 5: Gửi tín hiệu reset cho controller
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.5)
        self.reset_pub.publish(Bool(data=False))
        
        # Reset biến training
        self.prev_action = np.zeros(ACTION_DIM)
        self.obs_history.clear()
        
        # Bước 6: UNPAUSE simulation
        while not self.run_gz('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false'):
            time.sleep(0.5)
        
        self.is_falling = False
        print(" ✅ Done!", flush=True)
        return True
    
    # =================== TRAINING LOOP ===================
    def train_loop(self):
        """Vòng lặp training chính"""
        time.sleep(5)  # Chờ Gazebo khởi động
        
        total_steps = 0
        
        for episode in range(1, 10000):
            # Reset môi trường
            if not self.reset_simulation_physics():
                continue
            
            # Reset metrics cho episode mới
            self.roll_history = []
            self.pitch_history = []
            self.action_smoothness = []
            episode_fall = False
            
            episode_reward = 0
            state = self.get_state()
            
            print(f"📝 Episode {episode:4d} | ", end="", flush=True)
            
            # Rollout episode (tối đa 1500 bước = 15 giây)
            for step in range(1500):
                # Kiểm tra ngã
                if self.is_falling:
                    episode_fall = True
                    break
                
                # Thu thập metrics
                self.roll_history.append(abs(self.roll))
                self.pitch_history.append(abs(self.pitch))
                
                # --- Chọn action từ policy ---
                with torch.no_grad():
                    action_tensor, logprob_tensor, _ = self.ppo.policy_old.act(
                        torch.FloatTensor(state)
                    )
                action = action_tensor.numpy()  # Shape: (6,)
                
                # Thu thập smoothness metric
                if len(self.action_smoothness) > 0 or step > 0:
                    action_diff = np.mean((action - self.prev_action)**2)
                    self.action_smoothness.append(action_diff)
                
                # --- Gửi action ra controller ---
                action_msg = Float64MultiArray()
                action_msg.data = (action * ACTION_SCALE).tolist()
                self.action_pub.publish(action_msg)
                
                # Chờ controller xử lý
                time.sleep(0.01)
                
                # --- Tính reward ---
                # R1: Thưởng đứng thẳng (Exponential decay theo góc nghiêng)
                stability_reward = 1.0 + 2.0 * np.exp(-15.0 * (self.roll**2 + self.pitch**2))
                
                # R2: Phạt thay đổi action đột ngột (Smoothness penalty)
                action_diff = action - self.prev_action
                smoothness_penalty = -0.5 * np.mean(action_diff**2)
                
                reward = stability_reward + smoothness_penalty
                
                # R3: Phạt nặng nếu ngã
                if self.is_falling:
                    reward -= 20.0
                
                # --- Lưu vào buffer ---
                self.ppo.buffer['s'].append(torch.FloatTensor(state))
                self.ppo.buffer['a'].append(action_tensor)
                self.ppo.buffer['lp'].append(logprob_tensor)
                self.ppo.buffer['r'].append(reward)
                self.ppo.buffer['d'].append(self.is_falling)
                
                # --- Cập nhật trạng thái ---
                state = self.get_state()
                episode_reward += reward
                self.prev_action = action
                total_steps += 1
                
                # --- Update policy mỗi 2048 bước ---
                if total_steps % 2048 == 0:
                    print("\n🧠 Updating policy...", end="", flush=True)
                    self.ppo.update()
                    print(" ✅", flush=True)
            
            # --- Tính toán metrics cuối episode ---
            avg_roll = np.mean(self.roll_history) if self.roll_history else 0.0
            avg_pitch = np.mean(self.pitch_history) if self.pitch_history else 0.0
            max_roll = np.max(self.roll_history) if self.roll_history else 0.0
            max_pitch = np.max(self.pitch_history) if self.pitch_history else 0.0
            smoothness_score = 1.0 - np.mean(self.action_smoothness) if self.action_smoothness else 0.0
            
            if episode_fall:
                self.fall_count += 1
            
            # --- Ghi vào CSV ---
            self.csv_writer.writerow([
                episode,
                f"{episode_reward:.2f}",
                step,
                f"{self.best_reward:.2f}",
                f"{avg_roll:.4f}",
                f"{avg_pitch:.4f}",
                f"{max_roll:.4f}",
                f"{max_pitch:.4f}",
                f"{smoothness_score:.4f}",
                self.fall_count,
                datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            ])
            self.csv_file.flush()  # Ghi ngay lập tức
            
            # --- In kết quả episode ---
            print(f"Reward: {episode_reward:7.2f} | Steps: {step:4d} | "
                  f"AvgRoll: {avg_roll:.3f} | AvgPitch: {avg_pitch:.3f}", flush=True)
            
            # --- Lưu model tốt nhất ---
            if episode_reward > self.best_reward:
                self.best_reward = episode_reward
                torch.save(self.ppo.policy.state_dict(), MODEL_PATH)
                print(f"🏆 NEW BEST! Reward: {self.best_reward:.2f} → Saved to {MODEL_PATH}", flush=True)
        
        # Đóng CSV file khi training kết thúc
        self.csv_file.close()

# ==============================================================================
# MAIN
# ==============================================================================
def main():
    rclpy.init()
    node = RLTrainingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⚠️  Training interrupted by user", flush=True)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()