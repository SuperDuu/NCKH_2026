#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REACTIVE BALANCE TRAINING (PPO) — 4 Trụ Cột Kỹ Thuật v7
=========================================================
Mục tiêu: Robot đứng yên. Khi bị đẩy (400N theo Y) → bước chân lăng để 
phục hồi cân bằng → thu chân về vị trí chuẩn khi ổn định.

4 Trụ Cột:
  1. Input Discretization (STE Floor) — triệt tiêu nhiễu trắng IMU
  2. Dynamic Action Variance — Sigma phụ thuộc State, không dùng hằng số
  3. Independent Critic — mạng sâu hơn, không chia sẻ trọng số Actor
  4. Anti-overfitting — Dropout + BatchNorm + Entropy regularization

STATE_DIM = 18: [roll, pitch, d_roll, d_pitch, 6_offsets, 6_prev_actions, angular_vel_mag, yaw]
ACTION_DIM = 6: [offset_x_L, offset_y_L, offset_z_L, offset_x_R, offset_y_R, offset_z_R]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3, Wrench, Pose
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from ros_gz_interfaces.msg import WorldControl, Entity

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import threading
import time
import os
import json
import csv
import subprocess
from datetime import datetime

# ==============================================================================
# CẤU HÌNH
# ==============================================================================
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT = 0.05                 # 20Hz control loop
MAX_STEPS = 2000          # ~100 giây mỗi episode (20Hz * 100s)
MAX_LEG_LENGTH = 0.2204
SAFE_LIMIT = MAX_LEG_LENGTH * 0.98

STD_Z = 0.185             # Chiều cao Nominal
STD_Y = 0.01              # Khoảng cách Y chuẩn mỗi chân
LIFT_H = 0.04             # Nhấc chân cao hơn

ACTION_DIM = 6
STATE_DIM = 18            # Expanded: roll, pitch, d_roll, d_pitch, 6_offsets, 6_prev_actions, angular_vel_mag, yaw
TILT_THRESHOLD = 0.1     # >0.05 rad (~2.8°) → Nhạy cảm hơn
STABLE_THRESHOLD = 0.08   # <0.02 rad (~1.1°) → coi là ổn định
STABLE_COUNT_NEEDED = 30  # Ổn định liên tục lâu hơn

# Lực đẩy
PUSH_FORCE_MIN = 300.0
PUSH_FORCE_MAX = 500.0
PUSH_INTERVAL_MIN = 60    # Bước tối thiểu giữa các lần đẩy
PUSH_INTERVAL_MAX = 150

# ==============================================================================
# TRỤ CỘT 1: INPUT DISCRETIZATION (Straight-Through Estimator)
# ==============================================================================
class SteFloor(torch.autograd.Function):
    """Floor operation với gradient pass-through (STE).
    Forward: floor(x)   |   Backward: grad truyền thẳng qua."""
    @staticmethod
    def forward(ctx, x):
        return x.floor()

    @staticmethod
    def backward(ctx, grad):
        return grad  # Straight-through: không triệt tiêu gradient


class InputDiscretizer(nn.Module):
    """Ép kiểu số nguyên roll/pitch trước khi vào mạng.
    Độ phân giải 1° → scale = 180/π ≈ 57.29.
    x_out = floor(x_in * scale) / scale"""
    def __init__(self, scale=57.29):
        super().__init__()
        self.scale = scale

    def forward(self, x):
        return SteFloor.apply(x * self.scale) / self.scale


# ==============================================================================
# PPO NETWORK — 4 Trụ Cột Kỹ Thuật
# ==============================================================================
class ActorCritic(nn.Module):
    """Actor-Critic với:
    - InputDiscretizer trên roll/pitch (Trụ 1)
    - Shared Extractor + Mean Head + Sigma Head (Trụ 2)
    - Independent Critic (Trụ 3)
    - Dropout + GroupNorm (Trụ 4)
    """
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()

        # --- Trụ 1: Input Discretization ---
        self.imu_discretizer = InputDiscretizer(scale=57.29)

        # --- Trụ 2: Actor (Shared Extractor + Mean/Sigma Heads) ---
        self.actor_extractor = nn.Sequential(
            nn.Linear(state_dim, 512),
            nn.ELU(),
            nn.GroupNorm(32, 512),       # Trụ 4: GroupNorm (ổn định với batch=1, STM32-friendly)
            nn.Linear(512, 256),
            nn.ELU(),
            nn.Dropout(0.1),            # Trụ 4: Dropout
        )
        self.mean_head = nn.Sequential(
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
            nn.Tanh(),
        )
        self.log_std_head = nn.Linear(256, action_dim)
        nn.init.constant_(self.log_std_head.bias, -0.5)  # init std ≈ exp(-0.5) ≈ 0.60

        # --- Trụ 3: Independent Critic ---
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU(),
            nn.Linear(256, 1),
        )

    def _discretize_input(self, state):
        """Áp dụng STE Floor lên 4 chiều IMU đầu (roll, pitch, d_roll, d_pitch)."""
        s = state.clone()
        if s.dim() == 1:
            s[:4] = self.imu_discretizer(s[:4])
        else:
            s[:, :4] = self.imu_discretizer(s[:, :4])
        return s

    def act(self, state):
        """Inference: sample action từ Normal(mean, std)."""
        s = self._discretize_input(state)
        features = self.actor_extractor(s.unsqueeze(0) if s.dim() == 1 else s)
        mean = self.mean_head(features)
        log_std = torch.clamp(self.log_std_head(features), min=-2.0, max=0.5)
        std = torch.exp(log_std)  # σ ∈ [0.13, 1.65]
        dist = Normal(mean, std)
        
        # TRỤ 1: KHÔNG torch.clamp lên sample(). 
        # Việc kẹp cứng hành động ở đây làm méo mó Gradient của PPO.
        action = dist.sample()
        
        log_prob = dist.log_prob(action).sum(dim=-1)
        value = self.critic(s.unsqueeze(0) if s.dim() == 1 else s)
        return action.squeeze(0).detach(), log_prob.squeeze(0).detach(), value.squeeze(0).detach()

    def evaluate(self, state, action):
        """Training: tính log_prob, value, entropy cho batch."""
        s = self._discretize_input(state)
        features = self.actor_extractor(s)
        mean = self.mean_head(features)
        log_std = torch.clamp(self.log_std_head(features), min=-2.0, max=0.5)
        std = torch.exp(log_std)
        dist = Normal(mean, std)
        log_prob = dist.log_prob(action).sum(dim=-1)
        value = self.critic(s)
        entropy = dist.entropy().sum(dim=-1)
        return log_prob, value, entropy


class PPO:
    def __init__(self, state_dim, action_dim):
        self.gamma, self.eps_clip, self.K_epochs = 0.99, 0.2, 20
        self.buffer_states, self.buffer_actions, self.buffer_logprobs = [], [], []
        self.buffer_rewards, self.buffer_is_terminals = [], []
        self.policy = ActorCritic(state_dim, action_dim).to(DEVICE)
        self.optimizer = torch.optim.Adam([
            {'params': list(self.policy.actor_extractor.parameters()) +
                       list(self.policy.mean_head.parameters()) +
                       [*self.policy.log_std_head.parameters()], 'lr': 3e-4},
            {'params': self.policy.critic.parameters(), 'lr': 1e-3}
        ])
        self.policy_old = ActorCritic(state_dim, action_dim).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()
        
        # LR Scheduler: Giảm dần LR để ổn định chính sách sau khi đã thoát vùng cục bộ
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=1, gamma=0.999)

    def select_action(self, state):
        with torch.no_grad():
            state_t = torch.FloatTensor(state).to(DEVICE)
            self.policy_old.eval()  # BatchNorm ở eval mode khi inference
            action, logprob, _ = self.policy_old.act(state_t)
            self.policy_old.train()
        self.buffer_states.append(state_t)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(logprob)
        return action.cpu().numpy().flatten()

    def update(self):
        min_size = min(len(self.buffer_states), len(self.buffer_rewards))
        if min_size == 0:
            return
        print(f"\n\033[1;32m[PPO UPDATE] >>> Training với {min_size} steps...\033[0m")
        rewards, discounted_reward = [], 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards[:min_size]),
                                        reversed(self.buffer_is_terminals[:min_size])):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        rew_t = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        rewards_norm = (rew_t - rew_t.mean()) / (rew_t.std() + 1e-7)
        old_states = torch.stack(self.buffer_states[:min_size]).detach()
        old_actions = torch.stack(self.buffer_actions[:min_size]).detach()
        old_logprobs = torch.stack(self.buffer_logprobs[:min_size]).detach()
        self.policy.train()  # Đảm bảo BatchNorm/Dropout hoạt động khi train
        
        # Check LayerNorm vs GroupNorm compatibility with batch size
        if min_size < 32:
            print(f"\033[1;33m[WARNING] Batch size {min_size} < 32 which may affect GroupNorm(32, 512). Consider lowering groups or using more data.\033[0m")
            
        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards_norm - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            # Trụ 4: Entropy coefficient 0.01 ngăn sigma collapse
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards_norm) - 0.01 * dist_entropy
            
            # Khởi tạo loss optimization
            self.optimizer.zero_grad()
            loss.mean().backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=0.5)
            self.optimizer.step()
            
            # KL divergence early stopping
            with torch.no_grad():
                logprobs_new, _, _ = self.policy.evaluate(old_states, old_actions)
                # Approximation KL chuẩn trong PPO papers
                approx_kl = ((logprobs_new - old_logprobs).exp() - 1 - (logprobs_new - old_logprobs)).mean().item()
                if approx_kl > 0.015:
                    print(f"      \033[1;33m[PPO] Early stopping at epoch {_} due to approx KL div {approx_kl:.4f}\033[0m")
                    break
        
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear()
        self.buffer_actions.clear()
        self.buffer_logprobs.clear()
        self.buffer_rewards.clear()
        self.buffer_is_terminals.clear()
        
        # Update LR
        self.scheduler.step()
        current_lr = self.optimizer.param_groups[0]['lr']
        print(f"\033[1;32m[PPO UPDATE] >>> Thành công! Current LR: {current_lr:.6f}\033[0m\n")


# ==============================================================================
# REACTIVE BALANCE NODE
# ==============================================================================
class ReactiveBalanceNode(Node):
    def __init__(self):
        super().__init__('reactive_balance_node')

        # --- Đường dẫn lưu trữ ---
        self.weights_dir = os.path.join(
            os.path.expanduser("~"),
            "Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/weights/reactive_balance"
        )
        os.makedirs(self.weights_dir, exist_ok=True)
        self.latest_model_path = os.path.join(self.weights_dir, "latest_model.pth")
        self.best_model_path = os.path.join(self.weights_dir, "best_model.pth")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")
        self.history_path = os.path.join(self.weights_dir, "training_history.csv")

        # --- Publishers ---
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # --- Subscribers ---
        # Subscribe topic từ imu_process_node (geometry_msgs/Vector3)
        self.imu_sub = self.create_subscription(Vector3, '/robot_orientation', self.imu_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # --- Service Clients ---
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # Array of publishers for all 17 joints to force loosening during reset
        # Cố định Thân và Tay trên Gazebo Harmonic vì không nằm trong controller_config.yaml
        self.upper_joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in
                           ['base_hip_middle',
                            'hip_shoulder_left', 'shoulder_shoulder_left', 'shoulder_elbow_left',
                            'hip_shoulder_right', 'shoulder_shoulder_right', 'shoulder_elbow_right']}
        self.leg_joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in
                           ['base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
                            'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right']}

        # --- State ---
        self.roll = self.pitch = self.yaw = 0.0
        self.prev_roll = self.prev_pitch = 0.0
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.current_sim_time = 0.0
        self.prev_action = np.zeros(ACTION_DIM)
        self.current_offset = np.zeros(ACTION_DIM)  # 10 cái bù đắp (Offsets cho Chân + Lưng + Tay)

        # --- Trạng thái cân bằng (Residual RL) ---
        self.stable_count = 0          # Đếm số step liên tục ổn định (cho Recover reward)
        # Ngưỡng gia tốc góc để phát hiện robot còn chao đảo sau push (rad/s)
        self.ANGULAR_VEL_SETTLE_THRESH = 0.3  # ~17°/s
        
        # Thread safety lock cho state access
        self.state_lock = threading.Lock()

        # --- Training state ---
        self.best_reward = -float('inf')
        self.start_episode = 0
        self.step_in_episode = 0
        self.pushes_survived = 0
        self.total_pushes = 0
        self.total_pushes_global = 0 # Bộ đếm không bị reset bởi reset_simulation()
        self.imu_call_count = 0  # Đếm số lần imu_callback được gọi

        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()

        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    "Episode", "Reward", "Steps", "Pushes_Survived",
                    "Total_Pushes", "Reason", "STD", "Time"
                ])

        self.get_logger().info("=" * 60)
        self.get_logger().info("  REACTIVE BALANCE TRAINING v7 — 4 Pillars + Clean PPO")
        self.get_logger().info(f"  DT={DT}s ({1/DT:.0f}Hz) | LIFT_H={LIFT_H}m")
        self.get_logger().info(f"  TILT_THRESH={np.degrees(TILT_THRESHOLD):.1f}°")
        self.get_logger().info(f"  Push: {PUSH_FORCE_MIN}-{PUSH_FORCE_MAX}N (Y axis)")
        self.get_logger().info(f"  Pillars: STE-Floor | Dynamic-Sigma | Deep-Critic | Dropout+BN")
        self.get_logger().info("=" * 60)

        threading.Thread(target=self.train_loop, daemon=True).start()

    # ---------- CALLBACKS ----------
    def imu_callback(self, msg):
        """Dữ liệu từ imu_process_node: Vector3(y=roll, x=pitch, z=yaw) dạng ĐỘ."""
        
        # Calculate true radians first
        true_roll = np.radians(msg.y)
        true_pitch = np.radians(msg.x)
        
        # Apply offset so the resting position is mathematically 0.0
        with self.state_lock:
            self.imu_call_count += 1
            self.roll = true_roll - self.roll_offset
            self.pitch = true_pitch - self.pitch_offset
            self.yaw = np.radians(msg.z)

        # Trực tiếp cố định các khớp tay và eo ở 0 để tay không rũ xuống
        if hasattr(self, 'upper_joint_pubs'):
            for pub in self.upper_joint_pubs.values():
                pub.publish(Float64(data=0.0))

    def clock_callback(self, msg):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def sim_sleep(self, duration):
        """Chờ thời gian mô phỏng (sys_time) trôi qua một khoảng duration.
        Cách này không block Python thread theo clock thật, cho phép Fast-Forward.
        Thêm safety timeout để tránh busy loop."""
        start = self.current_sim_time
        deadline = time.time() + duration * 10
        while self.current_sim_time - start < duration:
            if time.time() > deadline:
                break
            # Ngủ 1ms thật để nhường CPU cho thread spin nhận clock
            time.sleep(0.001)

    # ---------- OBSERVATION ----------
    def get_observation(self):
        # Đảm bảo d_roll và d_pitch được tính từ giá trị đã lọc (Filtered)
        with self.state_lock:
            roll = self.roll
            pitch = self.pitch
            yaw = self.yaw
            prev_roll = self.prev_roll    # Cùng chung một lock
            prev_pitch = self.prev_pitch
            
        d_roll = (roll - prev_roll) / DT
        d_pitch = (pitch - prev_pitch) / DT
        angular_vel_mag = np.sqrt(d_roll**2 + d_pitch**2)
        
        return np.array([
            roll, pitch,
            d_roll, d_pitch,
            self.current_offset[0], self.current_offset[1], self.current_offset[2],
            self.current_offset[3], self.current_offset[4], self.current_offset[5],
            self.prev_action[0], self.prev_action[1], self.prev_action[2],
            self.prev_action[3], self.prev_action[4], self.prev_action[5],
            angular_vel_mag,                       # index 16
            yaw                                    # index 17 (Total: 18 elements)
        ])

    # ---------- REACTIVE BALANCE LOGIC ----------
    def compute_leg_command(self, action):
        """
        Logic cân bằng phản ứng theo Residual Reinforcement Learning.
        PPO Action (6 chiều): offset_x_L, offset_y_L, offset_z_L, offset_x_R, offset_y_R, offset_z_R
        """
        # Giả sử action từ [-1, 1], scale sang max offset
        # X, Y max = 0.08m, Z max = 0.04m
        scale_factors = np.array([0.08, 0.08, 0.04, 0.08, 0.08, 0.04])
        target_offset = action * scale_factors
        
        # Mô phỏng độ trễ của Servo MG996R
        VMAX_CARTESIAN = 0.20
        max_delta = VMAX_CARTESIAN * DT  # 0.020m/step
        
        # Rate-limit ĐỐI XỨNG cho tất cả các trục (bao gồm Z)
        delta = np.clip(target_offset - self.current_offset, -max_delta, max_delta)
        self.current_offset = self.current_offset + delta
        
        # --- BẢO VỆ SÀN: Z-offset chỉ được phép ≤ 0 ---
        self.current_offset[2] = min(self.current_offset[2], 0.0)
        self.current_offset[5] = min(self.current_offset[5], 0.0)
        
        # Lưu lại action cho step sau
        self.prev_action = action.copy()

        # Vị trí cuối cùng = Nominal Pose + Offset
        lx = 0.0 + self.current_offset[0]
        ly = STD_Y + self.current_offset[1]
        lz = STD_Z + self.current_offset[2]
        
        rx = 0.0 + self.current_offset[3]
        ry = -STD_Y + self.current_offset[4]
        rz = STD_Z + self.current_offset[5]
        
        # --- HEURISTIC SWING: Tự động nhấc chân khi XY dịch chuyển nhanh ---
        # Tránh ma sát sàn khi quẹt chân (Sliding vs Stepping)
        xy_vel_l = np.linalg.norm(delta[0:2]) / DT
        xy_vel_r = np.linalg.norm(delta[3:5]) / DT
        l_lift = LIFT_H if xy_vel_l > 0.05 else 0.0
        r_lift = LIFT_H if xy_vel_r > 0.05 else 0.0

        # --- Safety check ---
        pen = 0.0
        if (np.linalg.norm([lx, ly, lz]) > SAFE_LIMIT or
                np.linalg.norm([rx, ry, rz]) > SAFE_LIMIT):
            pen = -10.0

        msg = Float64MultiArray()
        msg.data = [lx, ly, lz, l_lift, rx, ry, rz, r_lift, DT]
        return msg, pen

    # ---------- FORCE PUSH ----------
    def apply_push(self, max_f):
        """Đẩy robot theo trục Y với lực max_f nhận từ curriculum."""
        threading.Thread(target=self._push_worker, args=(max_f,), daemon=True).start()
        self.total_pushes += 1
        self.total_pushes_global += 1

    def _push_worker(self, max_f):
        """
        Áp dụng lực đúng cách trong Gazebo Harmonic.
        Không sửa push_cooldown ở đây — cooldown được quản lý bởi main loop.
        """
        f_val = np.random.uniform(0.6 * max_f, max_f) * np.random.choice([-1, 1])
        axis = np.random.choice(['x', 'y', 'xy'])
        model_name = "humanoid_robot"

        force_str = ""
        if axis == 'y':
            force_str = f"{{y: {f_val:.1f}}}"
        elif axis == 'x':
            force_str = f"{{x: {f_val:.1f}}}"
        else:
            # Cross axis push
            f_val_x = f_val * 0.707
            f_val_y = f_val * 0.707 * np.random.choice([-1, 1])
            force_str = f"{{x: {f_val_x:.1f}, y: {f_val_y:.1f}}}"

        msg_str = (
            f"entity: {{name: '{model_name}', type: MODEL}}, "
            f"wrench: {{force: {force_str}}}"
        )
        msg_zero = f"entity: {{name: '{model_name}', type: MODEL}}, wrench: {{force: {{x: 0, y: 0, z: 0}}}}"

        print(f"\033[1;35m💥 PUSH: {f_val:.0f}N along {axis.upper()} | {model_name}(MODEL)\033[0m")

        # Áp dụng lực
        cmd_on = ["gz", "topic", "-t", "/world/empty/wrench",
                  "-m", "gz.msgs.EntityWrench", "-p", msg_str]
        subprocess.Popen(cmd_on, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.sim_sleep(0.3)

        # Dừng lực
        cmd_off = ["gz", "topic", "-t", "/world/empty/wrench",
                   "-m", "gz.msgs.EntityWrench", "-p", msg_zero]
        subprocess.Popen(cmd_off, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ---------- REWARD ----------
    def compute_reward(self, action, prev_action_snapshot, t, pen, angular_vel_mag):
        with self.state_lock:
            roll = self.roll
            pitch = self.pitch
            yaw = self.yaw
            
        tilt = np.sqrt(roll**2 + pitch**2)

        # 1. Thưởng sống sót và cân bằng (Tăng r_alive)
        r_alive = 5.0
        r_pitch = 8.0 * np.exp(-25.0 * abs(pitch))
        r_roll = 5.0 * np.exp(-15.0 * abs(roll))

        # 2. Phạt Action thay đổi (Giảm từ -3.0 xuống -0.5)
        r_action_diff = -0.5 * np.sum(np.square(action - prev_action_snapshot))

        # 3. Phạt nếu Offset quá lớn (giữ nguyên)
        r_limit = -0.5 * np.sum(np.maximum(0, np.abs(self.current_offset[[0,1,3,4]]) - 0.08))

        # 4. Z-Control Penalty: Phạt bình phương offset Z (Giảm từ -100 xuống -15.0)
        r_z_pen = -15.0 * np.sum(np.square(self.current_offset[[2, 5]]))

        # 5. Logic thu chân tự nhiên: dùng Threshold động 
        is_settling = angular_vel_mag > self.ANGULAR_VEL_SETTLE_THRESH  # Robot còn chao đảo
        
        r_recover = 0.0
        if is_settling or tilt >= 0.05:
            self.stable_count = 0       # Reset nếu còn chao đảo hoặc nghiêng
        else:
            self.stable_count += 1
        
        # Chỉ thưởng thu chân khi đã ổn định liên tục ≥ STABLE_COUNT_NEEDED steps (~1.5s ở 20Hz)
        if self.stable_count >= STABLE_COUNT_NEEDED:
            r_recover = 8.0 * np.exp(-10.0 * np.sum(np.abs(self.current_offset)))

        r_yaw = -1.0 * max(0.0, abs(yaw) - 0.1)

        return r_alive + r_pitch + r_roll + r_action_diff + r_limit + r_z_pen + r_recover + r_yaw + pen

    # ---------- SIMULATION CONTROL ----------
    def reset_simulation(self):
        """Reset robot — Mềm mại (Soft Landing)."""
        print("\033[93m>>> RESETTING SIMULATION (SOFT LANDING)...\033[0m")

        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.1)

        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--req", "pause: true"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        
        # Hạ thấp xuống z=0.25 (sát đất hơn) và hơi cúi người nhẹ (pitch = 0.05) để chống ngửa ra sau
        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--req", "name: 'humanoid_robot', position: {x: 0.0, y: 0.0, z: 0.26}, orientation: {w: 0.999, x: 0.0, y: 0.0, z: 0.0}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()

        with self.state_lock:
            # STATE DESYNC FIX: Mảng prev_action và current_offset phải 
            # ĐỒNG BỘ hoàn toàn với drop pose (-0.025 ở X)
            self.current_offset = np.array([-0.025, 0.0, 0.0, -0.025, 0.0, 0.0])
            self.prev_action = np.array([-0.025, 0.0, 0.0, -0.025, 0.0, 0.0])
            self.roll = self.pitch = self.yaw = 0.0
            self.prev_roll = self.prev_pitch = 0.0

        # ĐÃ XÓA ĐOẠN ÉP KHỚP VỀ 0.0 Ở ĐÂY! (Không làm chân thẳng tắp nữa)

        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--req", "pause: false"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()

        # Mở bật lại LLC
        self.reset_pub.publish(Bool(data=False))
        msg = Float64MultiArray()
        
        # -------------------------------------------------------------------------
        # TRẠNG THÁI ĐỨNG BAN ĐẦU: Chân đưa lùi về sau (-0.025m, nghiêng trọng tâm)
        # -------------------------------------------------------------------------
        msg.data = [-0.025, STD_Y, STD_Z, 0.0, -0.025, -STD_Y, STD_Z, 0.0, DT]
        
        drop_start = self.current_sim_time
        # Bắn lệnh trong 1.0s để robot kịp chạm đất bám trụ
        while self.current_sim_time - drop_start < 1.0:
            self.action_pub.publish(msg)
            time.sleep(0.05) 

        # Đợi tĩnh lại — PHẢI dùng sim_time thuần, không dùng timeout thực
        # PHẢI tiếp tục publish Nominal Pose để giữ chân robot cố định
        print("    \033[90mĐang đợi robot tĩnh sau khi hạ cánh (3.0s sim)...\033[0m")
        sc = self.current_sim_time
        while self.current_sim_time - sc < 3.0:
            self.action_pub.publish(msg)
            time.sleep(0.05)

        # CHUẨN HOÁ IMU: Lấy góc HIỆN TẠI làm mốc 0 sau khi robot đã hoàn toàn tĩnh
        with self.state_lock:
            raw_roll = self.roll + self.roll_offset
            raw_pitch = self.pitch + self.pitch_offset
            self.roll_offset = raw_roll
            self.pitch_offset = raw_pitch
            self.roll = self.pitch = self.yaw = 0.0
            self.prev_roll = self.prev_pitch = 0.0
        
        # Đợi thêm 0.1s sim để imu_callback kịp chạy 1 lần với offset mới
        # Đảm bảo self.roll đã thực sự ≈ 0.0 trước khi train_loop đọc nó
        extra_start = self.current_sim_time
        while self.current_sim_time - extra_start < 0.1:
            self.action_pub.publish(msg)
            time.sleep(0.05)
            
        self.step_in_episode = 0
        self.stable_count = 0
        self.pushes_survived = 0
        self.total_pushes = 0
        
        print("\033[92m>>> RESET DONE. Training starts.\033[0m")

    # ---------- SAVE/LOAD ----------
    def load_training_state(self):
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                data = json.load(f)
                self.best_reward = data.get('best_reward', -float('inf'))
                self.start_episode = data.get('current_episode', 0) + 1
        if os.path.exists(self.latest_model_path):
            self.ppo.policy.load_state_dict(
                torch.load(self.latest_model_path, map_location=DEVICE))
            self.ppo.policy_old.load_state_dict(self.ppo.policy.state_dict())
            self.get_logger().info("Loaded weights (v7 — Dynamic Sigma).")

    def save_state(self, ep, reward, is_best=False):
        data = {"best_reward": float(self.best_reward), "current_episode": int(ep)}
        with open(self.meta_path, 'w') as f:
            json.dump(data, f, indent=4)
        torch.save(self.ppo.policy.state_dict(), self.latest_model_path)
        if is_best:
            torch.save(self.ppo.policy.state_dict(), self.best_model_path)
            print(f"\033[1;33m⭐ NEW BEST: {reward:.2f} | Ep: {ep}\033[0m")

    # ---------- MAIN TRAINING LOOP ----------
    def train_loop(self):
        while self.current_sim_time == 0:
            time.sleep(0.1)
        ep = self.start_episode
        next_push_step = np.random.randint(PUSH_INTERVAL_MIN, PUSH_INTERVAL_MAX)
        prev_survival_rate = 0.0  # Lưu stat của episode trước

        while rclpy.ok():
            print("\033[94m>>> Bắt đầu Episode mới\033[0m")
            
            # Curriculum learning: Tăng lực dựa trên Survival Rate Cached từ episode CŨ
            max_f_scale = min(1.0, ep / 200.0) # Grow over 200 episodes max
            # Giảm nếu fail nhiều, tăng nếu win nhiều (chỉ tính nếu tập trước CÓ bị đẩy)
            if self.total_pushes > 0 and prev_survival_rate < 0.2 and self.total_pushes_global > 5:
                # Phạt lùi
                max_f_scale = max(0.1, max_f_scale - 0.2)
                
            curriculum_f = PUSH_FORCE_MIN + max_f_scale * (PUSH_FORCE_MAX - PUSH_FORCE_MIN)

            # LƯU STATS LẠI TRƯỚC KHI BỊ XÓA BỞI RESET
            prev_survival_rate = self.pushes_survived / max(1, self.total_pushes)
            
            self.reset_simulation()
            
            # Kiểm tra trạng thái vật lý thực sự ngay sau khi reset
            with self.state_lock:
                raw_roll = self.roll + self.roll_offset
                raw_pitch = self.pitch + self.pitch_offset
            initial_tilt = np.sqrt(raw_roll**2 + raw_pitch**2)
            if initial_tilt > 0.5: # ~28 độ
                print(f"\033[91m>>> Lỗi rớt (Nghiêng {np.degrees(initial_tilt):.1f} độ) ngay sau reset! Thử lại...\033[0m")
                continue # Skip this episode and try reset again
            
            ep_reward = 0.0
            reason = "TIME_LIMIT"
            was_tilted_before_push = False
            reset_lag = False

            for t in range(MAX_STEPS):
                self.step_in_episode = t

                # --- Đẩy robot ngẫu nhiên ---
                if t == next_push_step:
                    # Ghi nhận trạng thái trước khi đẩy
                    with self.state_lock:
                        was_tilted_before_push = (
                            np.sqrt(self.roll**2 + self.pitch**2) > TILT_THRESHOLD
                        )
                    self.apply_push(curriculum_f)  # Pass adjusted force, not episode
                    next_push_step = t + np.random.randint(PUSH_INTERVAL_MIN, PUSH_INTERVAL_MAX)

                # --- Observation & Action ---
                state = self.get_observation()
                action = self.ppo.select_action(state)
                
                # Lưu lại cái cũ TRƯỚC khi compute_leg_command update self.prev_action
                prev_action_snapshot = self.prev_action.copy()
                msg, pen = self.compute_leg_command(action)

                if msg is None:
                    self.ppo.buffer_rewards.append(-50.0)
                    self.ppo.buffer_is_terminals.append(True)
                    reason = "IK_FAIL"
                    
                    # Force joints to 0.0 so the robot goes limp and physically falls
                    if hasattr(self, 'joint_pubs'):
                        for pub in self.joint_pubs.values():
                            pub.publish(Float64(data=0.0))
                    
                    break

                self.action_pub.publish(msg)
                
                # Sleep mô phỏng thay vì block hệ thống bằng time.sleep(DT)
                self.sim_sleep(DT)

                # --- Debug IMU mỗi 50 step ---
                # --- Debug IMU mỗi 50 step ---
                with self.state_lock:
                    roll_rad = self.roll
                    pitch_rad = self.pitch
                    yaw_rad = self.yaw
                if t % 50 == 0:
                    print(
                        f"  [t={t:4d}] roll={np.degrees(roll_rad):+6.1f}deg "
                        f"pitch={np.degrees(pitch_rad):+6.1f}deg "
                        f"yaw={np.degrees(yaw_rad):+6.1f}deg "
                        f"imu_calls={self.imu_call_count}"
                    )
    
                # --- Nhất quán logic bằng Radian, 0.785 rad ≈ 45 độ ---
                tilt_now_rad = np.sqrt(roll_rad**2 + pitch_rad**2)
                done = tilt_now_rad > 0.785
                if done:
                    print(
                        f"  \033[91m[FALL t={t}] roll={np.degrees(roll_rad):+.1f}deg "
                        f"pitch={np.degrees(pitch_rad):+.1f}deg\033[0m"
                    )
                    if t == 0:
                        print("\033[91m⚠️ Reset Lag Fall. Skipping episode.\033[0m")
                        reset_lag = True
                        break

                # --- Kiểm tra sống sót sau push ---
                # Ensure it only counts as survived if it actually got pushed and then stabilized
                if self.total_pushes > 0 and tilt_now_rad < STABLE_THRESHOLD and was_tilted_before_push:
                    self.pushes_survived = self.total_pushes
                    was_tilted_before_push = False # Đã hoàn thành phục hồi cho lần đẩy này

                # --- Reward ---
                if done:
                    reward = -100.0
                    self.stable_count = 0
                else:
                    # state[16] is angular_vel_mag from STATE_DIM=18
                    reward = self.compute_reward(action, prev_action_snapshot, t, pen, state[16])
                
                # Cập nhật góc cũ TẠI ĐÂY (luồng đồng bộ 20Hz) để vi phân chính xác
                with self.state_lock:
                    self.prev_roll = self.roll
                    self.prev_pitch = self.pitch

                self.ppo.buffer_rewards.append(reward)
                self.ppo.buffer_is_terminals.append(done)
                ep_reward += reward

                if done:
                    reason = "FALL"
                    break

            # --- Handle Reset Lag Jump ---
            if reset_lag:
                self.ppo.buffer_states.clear()
                self.ppo.buffer_actions.clear()
                self.ppo.buffer_logprobs.clear()
                self.ppo.buffer_rewards.clear()
                self.ppo.buffer_is_terminals.clear()
                continue

            # =========================================================
            # ĐÚNG LOGIC: UPDATE TRƯỚC, LƯU SAU
            # =========================================================
            
            # 1. PPO UPDATE (Cho não học kinh nghiệm mới)
            if len(self.ppo.buffer_states) >= 1024:
                self.ppo.update()

            # 2. TÍNH ĐIỂM BEST CÓ XÉT HỆ SỐ KHÓ (Curriculum Scaling)
            # Tập lực mạnh (500N) phải được nhân hệ số để đấu lại điểm số của tập lực yếu (300N)
            difficulty_multiplier = curriculum_f / PUSH_FORCE_MIN  # Chạy từ 1.0 đến 1.66
            adjusted_score = ep_reward * difficulty_multiplier

            # 3. LƯU TRẠNG THÁI (Lưu bộ não ĐÃ ĐƯỢC UPDATE)
            if adjusted_score > self.best_reward:
                self.best_reward = adjusted_score
                self.save_state(ep, adjusted_score, True)
            else:
                self.save_state(ep, adjusted_score, False)

            # Log CSV (Vẫn ghi ep_reward thật để dễ xem biểu đồ)
            with open(self.history_path, 'a', newline='') as f:
                csv.writer(f).writerow([
                    ep, f"{ep_reward:.2f}", t + 1,
                    self.pushes_survived, self.total_pushes,
                    reason, "dynamic",
                    datetime.now().strftime("%H:%M:%S")
                ])

            print(
                f"RB-Ep {ep} | R: {ep_reward:.2f} (Adj: {adjusted_score:.2f}) | "
                f"Best: {self.best_reward:.2f} | Steps: {t + 1} | "
                f"Push: {self.pushes_survived}/{self.total_pushes} | {reason}"
            )

            ep += 1
            next_push_step = np.random.randint(PUSH_INTERVAL_MIN, PUSH_INTERVAL_MAX)


def main():
    rclpy.init()
    rclpy.spin(ReactiveBalanceNode())


if __name__ == '__main__':
    main()
