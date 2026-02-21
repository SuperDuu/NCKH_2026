#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REACTIVE BALANCE TRAINING (PPO)
================================
Mục tiêu: Robot đứng yên. Khi bị đẩy (400N theo Y) → bước chân lăng để 
phục hồi cân bằng → thu chân về vị trí chuẩn khi ổn định.

Logic khác với CPG walking:
- KHÔNG dùng pha thời gian cố định để đổi chân
- AI quyết định KHI NÀO bước và bước ĐẾN ĐÂU dựa trên roll/pitch
- Khi đã ổn định → từ từ thu chân về vị trí đứng chuẩn

STATE_DIM = 7: [roll, pitch, d_roll, d_pitch, yaw, sin(phase), cos(phase)]
ACTION_DIM = 4: [dy_shift, dz, step_y, step_x]
  - dy_shift: Dịch trọng tâm sang bên (chân trụ lấn vào giữa)
  - dz: Điều chỉnh chiều cao
  - step_y: Bước chân lăng theo Y (hướng nghiêng)
  - step_x: Bước chân lăng theo X (trước/sau)
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
DT = 0.02                 # 50Hz control loop
MAX_STEPS = 5000          # ~100 giây mỗi episode (50Hz * 100s)
MAX_LEG_LENGTH = 0.2204
SAFE_LIMIT = MAX_LEG_LENGTH * 0.98

STD_Z = 0.185             # Chiều cao thấp hơn (khụy gối nhiều hơn) để hạ trọng tâm, giúp khớp yếu bám trụ tốt hơn
STD_Y = 0.01              # Khoảng cách Y chuẩn mỗi chân
LIFT_H = 0.04             # Nhấc chân cao hơn (4cm) để tránh vấp móng

ACTION_DIM = 4
STATE_DIM = 7
ACTION_STD_INIT = 0.2     # Giảm biên độ random ban đầu để robot đỡ giật cục (vì khớp đang yếu)
ACTION_STD_MIN = 0.05
DECAY_FACTOR = 0.995
DECAY_INTERVAL_EP = 20

# Ngưỡng cân bằng
TILT_THRESHOLD = 0.1     # >0.05 rad (~2.8°) → Nhạy cảm hơn, kích hoạt bước sớm
STABLE_THRESHOLD = 0.08   # <0.02 rad (~1.1°) → coi là ổn định, thu chân về
STABLE_COUNT_NEEDED = 30  # Ổn định liên tục lâu hơn mới thu chân

# Lực đẩy
PUSH_FORCE_MIN = 600.0
PUSH_FORCE_MAX = 900.0
PUSH_INTERVAL_MIN = 60    # Bước tối thiểu giữa các lần đẩy
PUSH_INTERVAL_MAX = 150

# ==============================================================================
# PPO NETWORK
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init):
        super(ActorCritic, self).__init__()
        self.action_var = torch.full((action_dim,), action_std_init * action_std_init).to(DEVICE)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(),
            nn.Linear(256, action_dim), nn.Tanh()
        )
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(),
            nn.Linear(256, 1)
        )

    def set_action_std(self, new_action_std):
        self.action_var = torch.full((ACTION_DIM,), new_action_std * new_action_std).to(DEVICE)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        return action.detach(), dist.log_prob(action).sum(dim=-1).detach(), self.critic(state).detach()

    def evaluate(self, state, action):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var.expand_as(action_mean)))
        return dist.log_prob(action).sum(dim=-1), self.critic(state), dist.entropy().sum(dim=-1)


class PPO:
    def __init__(self, state_dim, action_dim):
        self.gamma, self.eps_clip, self.K_epochs = 0.99, 0.2, 20
        self.buffer_states, self.buffer_actions, self.buffer_logprobs = [], [], []
        self.buffer_rewards, self.buffer_is_terminals = [], []
        self.policy = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': 3e-4},
            {'params': self.policy.critic.parameters(), 'lr': 1e-3}
        ])
        self.policy_old = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()

    def select_action(self, state):
        with torch.no_grad():
            state_t = torch.FloatTensor(state).to(DEVICE)
            action, logprob, _ = self.policy_old.act(state_t)
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
        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards_norm - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards_norm) - 0.01 * dist_entropy
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear()
        self.buffer_actions.clear()
        self.buffer_logprobs.clear()
        self.buffer_rewards.clear()
        self.buffer_is_terminals.clear()
        print("\033[1;32m[PPO UPDATE] >>> Thành công!\033[0m\n")


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
        force_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        self.force_pub = self.create_publisher(
            Wrench, '/model/humanoid_robot/link/base_footprint/wrench', force_qos)
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # --- Subscribers ---
        # Subscribe thẳng /imu (sensor_msgs/Imu) với QoS BEST_EFFORT để match với Gazebo bridge
        imu_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, imu_qos)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # --- Service Clients ---
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # --- State ---
        self.roll = self.pitch = self.yaw = 0.0
        self.prev_roll = self.prev_pitch = 0.0
        self.smooth_roll = self.smooth_pitch = 0.0
        self.current_sim_time = 0.0
        self.prev_action = np.zeros(ACTION_DIM)

        # --- Trạng thái cân bằng ---
        self.balance_state = "STANDING"  # STANDING → STEPPING → RECOVERING
        self.stable_counter = 0
        self.step_direction = 0          # +1: nghiêng trái, -1: nghiêng phải
        self.stepping_foot = "none"      # "left" hoặc "right"

        # --- Training state ---
        self.best_reward = -float('inf')
        self.start_episode = 0
        self.step_in_episode = 0
        self.pushes_survived = 0
        self.total_pushes = 0
        self.imu_call_count = 0  # Đếm số lần imu_callback được gọi

        self.current_std = ACTION_STD_INIT
        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()

        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f:
                csv.writer(f).writerow([
                    "Episode", "Reward", "Steps", "Pushes_Survived",
                    "Total_Pushes", "Reason", "STD", "Time"
                ])

        self.get_logger().info("=" * 60)
        self.get_logger().info("  REACTIVE BALANCE TRAINING")
        self.get_logger().info(f"  STD_INIT={ACTION_STD_INIT} | LIFT_H={LIFT_H}m")
        self.get_logger().info(f"  TILT_THRESH={np.degrees(TILT_THRESHOLD):.1f}°")
        self.get_logger().info(f"  Push: {PUSH_FORCE_MIN}-{PUSH_FORCE_MAX}N (Y axis)")
        self.get_logger().info("=" * 60)

        threading.Thread(target=self.train_loop, daemon=True).start()

    # ---------- CALLBACKS ----------
    def imu_callback(self, msg):
        """Convert quaternion IMU → roll/pitch/yaw (radians) trực tiếp."""
        self.imu_call_count += 1
        self.prev_roll, self.prev_pitch = self.smooth_roll, self.smooth_pitch
        # Quaternion → Euler
        q = msg.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.roll = np.arctan2(sinr_cosp, cosr_cosp)
        # Pitch
        sinp = 2.0 * (w * y - z * x)
        self.pitch = np.copysign(np.pi / 2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)
        # Smooth
        self.smooth_roll = 0.3 * self.smooth_roll + 0.7 * self.roll
        self.smooth_pitch = 0.3 * self.smooth_pitch + 0.7 * self.pitch

    def clock_callback(self, msg):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def sim_sleep(self, duration):
        """Chờ thời gian mô phỏng (sys_time) trôi qua một khoảng duration.
        Cách này không block Python thread theo clock thật, cho phép Fast-Forward."""
        start = self.current_sim_time
        while self.current_sim_time - start < duration:
            # Ngủ 1ms thật để nhường CPU cho thread spin nhận clock
            time.sleep(0.001)

    # ---------- OBSERVATION ----------
    def get_observation(self):
        d_roll = (self.smooth_roll - self.prev_roll) / DT
        d_pitch = (self.smooth_pitch - self.prev_pitch) / DT
        # Phase dựa trên sim_time, dùng cho timing cơ bản
        phase = (self.current_sim_time % 1.5) * (2 * np.pi / 1.5)
        return np.array([
            self.smooth_roll, self.smooth_pitch,
            d_roll, d_pitch,
            self.yaw,
            np.sin(phase), np.cos(phase)
        ])

    # ---------- REACTIVE BALANCE LOGIC ----------
    def compute_leg_command(self, action):
        """
        Logic cân bằng phản ứng (Asymmetric Stepping):
        - STANDING: 2 chân đứng thẳng.
        - STEPPING: Chân Trụ (Stance) đứng im dồn trọng tâm, Chân Lăng (Swing) vung ra đỡ thân.
        - RECOVERING: Thu chân lăng về.
        """
        # Smooth action
        smooth_act = 0.7 * self.prev_action + 0.3 * action
        self.prev_action = smooth_act

        # Giải mã action: Biên độ lớn hơn hẵn để phản ứng triệt để
        dy_shift = smooth_act[0] * 0.03    # Dịch trọng tâm chân trụ (max ±4cm)
        dz = smooth_act[1] * 0.02          # Điều chỉnh Z (max ±2cm)
        step_y = smooth_act[2] * 0.05      # Lăng ngang cực đại (max ±12cm)
        step_x = smooth_act[3] * 0.05      # Lăng dọc cực đại (max ±12cm)

        tilt_magnitude = np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2)

        # Mặc định: đứng yên
        if self.balance_state == "STANDING":
            # Triệt tiêu hoàn toàn RL noise khi đang đứng yên để servo không co giật
            tz = STD_Z
            dy_shift = 0.0
            step_x = 0.0
            step_y = 0.0
        else:
            tz = STD_Z + dz

        lx, ly, lz, l_lift = 0.0, STD_Y, tz, 0.0
        rx, ry, rz, r_lift = 0.0, -STD_Y, tz, 0.0

        # --- Cập nhật trạng thái cân bằng ---
        if self.balance_state == "STANDING":
            if tilt_magnitude > TILT_THRESHOLD:
                self.balance_state = "STEPPING"
                self.stable_counter = 0
                
                # --- Xác định Chân Lăng (Swing Leg) ---
                # Phân tích khuynh hướng ngã
                if abs(self.smooth_roll) > abs(self.smooth_pitch) * 0.5:
                    # Ngã vật ngang ưu tiên
                    if self.smooth_roll > 0:
                        self.stepping_foot = "left"   # Ngã trái, tung chân trái đỡ
                    else:
                        self.stepping_foot = "right"  # Ngã phải, tung chân phải đỡ
                else:
                    # Ngã sấp ngửa ưu tiên
                    if self.smooth_pitch > 0:
                        self.stepping_foot = "right"  # Ngã ra sau, tung chân phải (fix)
                    else:
                        self.stepping_foot = "left"   # Ngã tới trước, tung chân trái (fix)

        elif self.balance_state == "STEPPING":
            if tilt_magnitude < STABLE_THRESHOLD:
                self.stable_counter += 1
                if self.stable_counter >= STABLE_COUNT_NEEDED:
                    self.balance_state = "RECOVERING"
                    self.stable_counter = 0
            else:
                self.stable_counter = 0

        elif self.balance_state == "RECOVERING":
            if tilt_magnitude > TILT_THRESHOLD:
                self.balance_state = "STEPPING"
                self.stable_counter = 0
            elif self.stable_counter >= STABLE_COUNT_NEEDED:
                self.balance_state = "STANDING"
                self.stepping_foot = "none"
                self.stable_counter = 0
            else:
                self.stable_counter += 1

        # --- Trạng thái Bước Chân Cụ Thể ---
        # Tính toán reflex cơ sở (baseline) dựa trên góc nghiêng
        # X: bước tới/lui (pitch). Y: bước ngang (roll)
        base_step_x = 0.0
        base_step_y = 0.0
        
        # Hàm sigmoid hoặc tỉ lệ để quyết định độ dài bước cơ bản
        if self.stepping_foot != "none":
            # Hệ số tỉ lệ (tùy chỉnh)
            k_pitch = 0.5  # rad -> mét
            k_roll = 0.5   # rad -> mét
            
            # Cắt tín hiệu để tránh bước quá dài
            base_step_x = np.clip(self.smooth_pitch * k_pitch, -0.15, 0.15)
            base_step_y = np.clip(self.smooth_roll * k_roll, -0.15, 0.15)

        # --- Áp dụng action theo trạng thái ---
        if self.balance_state == "STEPPING":
            # Determine if it's primarily a pitch or roll disturbance
            is_pitch_dominant = abs(self.smooth_pitch) > abs(self.smooth_roll)

            if self.stepping_foot == "left":
                if is_pitch_dominant:
                    # Ngã tới/lui: Chân trái bước tới/lui (CHỈ THEO X)
                    ly = STD_Y + dy_shift # Giữ nguyên chiều rộng chuẩn + xê dịch nhẹ trọng tâm
                    lx = base_step_x + step_x
                else:
                    # Ngã ngang: Chân trái bước chéo (CHÉO THEO X VÀ Y)
                    ly = STD_Y + abs(base_step_y) + abs(step_y) if self.smooth_roll > 0 else STD_Y - abs(base_step_y) - abs(step_y)
                    lx = base_step_x + step_x

                l_lift = LIFT_H                # Nhấc chân lên
                
                # Chân phải làm TRỤ
                ry = -STD_Y - dy_shift         
                rx = 0.0
                r_lift = -0.01  # Nén chân trụ một chút bám đất
                
            else: # stepping_foot == "right"
                if is_pitch_dominant:
                    # Ngã tới/lui: Chân phải bước tới/lui (CHỈ THEO X)
                    ry = -STD_Y - dy_shift
                    rx = base_step_x + step_x
                else:
                    # Ngã ngang: Chân phải bước chéo (CHÉO THEO X VÀ Y)
                    ry = -STD_Y - abs(base_step_y) - abs(step_y) if self.smooth_roll < 0 else -STD_Y + abs(base_step_y) + abs(step_y)
                    rx = base_step_x + step_x

                r_lift = LIFT_H
                
                # Chân trái làm TRỤ
                ly = STD_Y + dy_shift
                lx = 0.0
                l_lift = -0.01

        elif self.balance_state == "RECOVERING":
            # Thu chân lăng DẦN VỀ vị trí chuẩn (smooth return)
            recover_rate = 0.02  # Tăng tốc độ thu chân (2% -> nhanh hơn xíu)
            if self.stepping_foot == "left":
                ly = STD_Y * (1 - recover_rate) + ly * recover_rate
                lx = lx * recover_rate
            else:
                ry = -STD_Y * (1 - recover_rate) + ry * recover_rate
                rx = rx * recover_rate

        # --- Safety check ---
        if (np.linalg.norm([lx, ly, lz]) > SAFE_LIMIT or
                np.linalg.norm([rx, ry, rz]) > SAFE_LIMIT):
            return None, -10.0

        msg = Float64MultiArray()
        msg.data = [lx, ly, lz, l_lift, rx, ry, rz, r_lift, DT]
        return msg, 0.0

    # ---------- FORCE PUSH ----------
    def apply_push(self, episode):
        """Đẩy robot theo trục Y với lực ngẫu nhiên"""
        max_f = min(PUSH_FORCE_MAX, PUSH_FORCE_MIN + (episode // 30) * 15.0)
        threading.Thread(target=self._push_worker, args=(max_f,), daemon=True).start()
        self.total_pushes += 1

    def _push_worker(self, max_f):
        """
        Áp dụng lực đúng cách trong Gazebo Harmonic:
        Topic: /world/empty/wrench
        Type:  gz.msgs.EntityWrench
        
        Cú pháp từ tài liệu chính thức gz-sim8:
          gz topic -t "/world/apply_link_wrench/wrench" -m gz.msgs.EntityWrench
            -p "entity: {name: 'model::link', type: LINK}, wrench: {force: {y: 1000}}"
        """
        import subprocess
        f_val = np.random.uniform(0.6 * max_f, max_f) * np.random.choice([-1, 1])
        # Random axis: Y = sang ngang (roll), X = trước/sau (pitch)
        axis = np.random.choice(['y', 'x'])
        
        # Tên entity: dùng MODEL type → tự tìm canonical link
        model_name = "humanoid_robot"

        if axis == 'y':
            force_str = f"{{y: {f_val:.1f}}}"
        else:
            force_str = f"{{x: {f_val:.1f}}}"

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
    def compute_reward(self, action, t, pen):
        tilt = np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2)

        # Thưởng sống sót
        r_alive = 2.0

        # Thưởng cân bằng (càng thẳng càng tốt)
        r_balance = 6.0 * np.exp(-15.0 * tilt)

        # Phạt yaw (xoay robot)
        r_yaw = -2.0 * max(0.0, abs(self.yaw) - 0.1)

        # Phạt action quá lớn (tiết kiệm năng lượng)
        r_energy = -0.3 * np.sum(np.square(action))

        # Thưởng thêm khi ở trạng thái STANDING (đứng yên ổn định)
        r_standing = 2.0 if self.balance_state == "STANDING" else 0.0

        # Thưởng phục hồi thành công (STEPPING → RECOVERING → STANDING)
        r_recovery = 3.0 if self.balance_state == "RECOVERING" else 0.0

        return r_alive + r_balance + r_yaw + r_energy + r_standing + r_recovery + pen

    # ---------- SIMULATION CONTROL ----------
    def stabilize_robot(self):
        """Gửi lệnh đứng thẳng liên tục và đợi robot ổn định."""
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(30):
            self.action_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(0.5)

    def reset_simulation(self):
        """Reset robot — copy exact logic từ ft_force.py (đã hoạt động)."""
        print("\033[93m>>> RESETTING SIMULATION...\033[0m")

        # 1. Báo LLC node dừng xử lý
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.5)

        # 2. Pause physics
        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--req", "pause: true"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        
        # 3. Dịch chuyển bằng native gz
        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--req", "name: 'humanoid_robot', position: {x: 0.0, y: 0.0, z: 0.255}, orientation: {w: 1.0, x: 0.00, y: 0.0, z: 0.0}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()
        time.sleep(0.5)

        # 4. Chuẩn bị tư thế: Bật lại LLC và nạp tư thế đứng thẳng TRƯỚC KHI rớt xuống
        self.reset_pub.publish(Bool(data=False))
        # Gửi lệnh nhún về tiêu chuẩn
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(2):
            self.action_pub.publish(msg)
            time.sleep(0.01)

        # 5. Unpause physics
        subprocess.Popen([
            "gz", "service", "-s", "/world/empty/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--req", "pause: false"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).wait()

        # 6. Đợi sim clock chạy lại
        sc = self.current_sim_time
        timeout = 0
        while self.current_sim_time - sc < 0.2:
            time.sleep(0.01)
            timeout += 1
            if timeout > 500:  # 5 giây timeout
                print("\033[91m>>> Clock timeout! Forcing continue.\033[0m")
                break

        # 7. Vòng lặp chờ robot ổn định hoàn toàn
        print("    \033[90mĐang đợi robot ổn định...\033[0m")
        stable_frames = 0
        max_wait_time = 4.0  # Chờ tối đa 4 giây mô phỏng
        wait_start = self.current_sim_time
        
        while rclpy.ok():
            # Điều kiện ổn định: nghiêng rất ít và ít dao động
            tilt_now = np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2)
            d_roll_now = abs(self.smooth_roll - self.prev_roll) / DT
            d_pitch_now = abs(self.smooth_pitch - self.prev_pitch) / DT
            
            is_stable = (tilt_now < 0.05) and (d_roll_now < 0.1) and (d_pitch_now < 0.1)

            if is_stable:
                stable_frames += 1
            else:
                stable_frames = 0
                
            if stable_frames >= 20: # Ổn định liên tục ~0.4s
                break
                
            if (self.current_sim_time - wait_start) > max_wait_time:
                print("\033[93m>>> Cảnh báo: Robot không thể ổn định hoàn toàn trong giới hạn thời gian.\033[0m")
                break
                
            self.sim_sleep(DT)

        # 8. Cập nhật state (KHÔNG gán 0 cho IMU, nếu gán 0 sẽ làm d_roll nhảy lên max vì mâu thuẫn IMU thật)
        self.step_in_episode = 0
        self.prev_action = np.zeros(ACTION_DIM)
        self.balance_state = "STANDING"
        self.stepping_foot = "none"
        self.stable_counter = 0
        self.pushes_survived = 0
        self.total_pushes = 0

        # 8. Stabilize
        self.stabilize_robot()
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
            self.current_std = torch.sqrt(self.ppo.policy.action_var[0]).item()
            self.get_logger().info(f"Loaded weights. STD={self.current_std:.4f}")

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

        while rclpy.ok():
            print("\033[94m>>> Bắt đầu Episode mới\033[0m")
            self.reset_simulation()
            
            # Kiểm tra trạng thái ngay sau khi reset
            initial_tilt = np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2)
            if initial_tilt > 0.5: # ~28 độ
                print("\033[91m>>> Lỗi rớt ngay sau reset! Thử lại...\033[0m")
                continue # Skip this episode and try reset again
            
            ep_reward = 0.0
            failed_at_start = False
            reason = "TIME_LIMIT"
            was_tilted_before_push = False

            for t in range(MAX_STEPS):
                self.step_in_episode = t

                # --- Đẩy robot ngẫu nhiên ---
                if t == next_push_step:
                    # Ghi nhận trạng thái trước khi đẩy
                    was_tilted_before_push = (
                        np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2) > TILT_THRESHOLD
                    )
                    self.apply_push(ep)
                    next_push_step = t + np.random.randint(PUSH_INTERVAL_MIN, PUSH_INTERVAL_MAX)

                # --- Observation & Action ---
                state = self.get_observation()
                action = self.ppo.select_action(state)
                msg, pen = self.compute_leg_command(action)

                if msg is None:
                    self.ppo.buffer_rewards.append(-50.0)
                    self.ppo.buffer_is_terminals.append(True)
                    reason = "IK_FAIL"
                    break

                self.action_pub.publish(msg)
                
                # Sleep mô phỏng thay vì block hệ thống bằng time.sleep(DT)
                self.sim_sleep(DT)

                # --- Debug IMU mỗi 50 step ---
                if t % 50 == 0:
                    print(
                        f"  [t={t:4d}] roll={np.degrees(self.smooth_roll):+6.1f}deg "
                        f"pitch={np.degrees(self.smooth_pitch):+6.1f}deg "
                        f"yaw={np.degrees(self.yaw):+6.1f}deg "
                        f"state={self.balance_state} imu_calls={self.imu_call_count}"
                    )
    
                # --- Check nga (45 do = 0.785 rad) ---
                done = abs(self.smooth_roll) > 0.785 or abs(self.smooth_pitch) > 0.785
                if done:
                    print(
                        f"  \033[91m[FALL t={t}] roll={np.degrees(self.smooth_roll):+.1f}deg "
                        f"pitch={np.degrees(self.smooth_pitch):+.1f}deg\033[0m"
                    )

                # --- Kiểm tra sống sót sau push ---
                tilt_now = np.sqrt(self.smooth_roll**2 + self.smooth_pitch**2)
                # Ensure it only counts as survived if it actually got pushed and then stabilized
                if self.total_pushes > 0 and tilt_now < STABLE_THRESHOLD and was_tilted_before_push:
                    self.pushes_survived = self.total_pushes
                    was_tilted_before_push = False # Đã hoàn thành phục hồi cho lần đẩy này

                # --- Reward ---
                reward = self.compute_reward(action, t, pen)
                if done:
                    reward = -100.0

                if t == 0 and done:
                    failed_at_start = True
                    break

                self.ppo.buffer_rewards.append(reward)
                self.ppo.buffer_is_terminals.append(done)
                ep_reward += reward

                # --- PPO Update mỗi 4000 steps ---
                if len(self.ppo.buffer_states) >= 4000:
                    self.ppo.update()

                if done:
                    reason = "FALL"
                    break

            # --- End of Episode ---
            if not failed_at_start:
                # STD Decay
                if ep > 0 and ep % DECAY_INTERVAL_EP == 0:
                    self.current_std = max(ACTION_STD_MIN, self.current_std * DECAY_FACTOR)
                    self.ppo.policy.set_action_std(self.current_std)
                    self.ppo.policy_old.set_action_std(self.current_std)

                # Save
                if ep_reward > self.best_reward:
                    self.best_reward = ep_reward
                    self.save_state(ep, ep_reward, True)
                else:
                    self.save_state(ep, ep_reward, False)

                # Log
                with open(self.history_path, 'a', newline='') as f:
                    csv.writer(f).writerow([
                        ep, f"{ep_reward:.2f}", t,
                        self.pushes_survived, self.total_pushes,
                        reason, f"{self.current_std:.4f}",
                        datetime.now().strftime("%H:%M:%S")
                    ])

                print(
                    f"RB-Ep {ep} | R: {ep_reward:.2f} | Best: {self.best_reward:.2f} | "
                    f"Steps: {t} | Push: {self.pushes_survived}/{self.total_pushes} | "
                    f"STD: {self.current_std:.4f} | {reason}"
                )
                ep += 1
                next_push_step = np.random.randint(PUSH_INTERVAL_MIN, PUSH_INTERVAL_MAX)
            else:
                print("\033[91m⚠️ Reset Lag. Skipping.\033[0m")


def main():
    rclpy.init()
    rclpy.spin(ReactiveBalanceNode())


if __name__ == '__main__':
    main()
