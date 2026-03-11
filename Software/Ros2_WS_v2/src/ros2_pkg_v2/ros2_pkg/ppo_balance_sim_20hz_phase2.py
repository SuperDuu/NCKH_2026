#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Phase 2: PPO Static Balance — 70D History Buffer + Production Hardening
# =============================================================================
# Fixes tích hợp từ phản biện kỹ thuật:
#   - IMU Watchdog: cảnh báo nếu IMU stale > 0.1s
#   - Gyro chuẩn hóa: clip [-10, 10] / 10.0 → [-1, 1]
#   - NaN protection: np.nan_to_num trước khi nạp buffer
#   - SCALE_DX=0.04, SCALE_DY=0.02 tránh IK reject ở cực trị
#   - EMA_ALPHA=0.85 giảm trễ do lọc kép với LLC (α=0.6)
#   - R_jitter=-1.5 tránh policy collapse (cũ -5.0 quá mạnh vs R_balance=1.0)
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool, Float64
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from ros_gz_interfaces.msg import WorldControl, Entity
from geometry_msgs.msg import Pose

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import collections
import time
import os
import json
import csv
import subprocess
import threading
import math
from datetime import datetime

# =============================================================================
# CẤU HÌNH PHẦN CỨNG & HUẤN LUYỆN
# =============================================================================
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

DT = 0.05                      # 20Hz control loop
MOVE_DURATION = 0.15            # 150ms — LLC có 15 iterations để nội suy mượt
MAX_STEPS = 9999

# --- Thông số IK (LLC V2: L3=0.08, L4=0.15, L5=0.065) ---
MAX_LEG_LENGTH = 0.293          # L3+L4+L5 - 0.002mm an toàn
SAFE_LIMIT = MAX_LEG_LENGTH * 0.999
STD_Z = 0.265                  # Chiều cao đứng chuẩn (LLC V2 default Z)
STD_Y = 0.01
LIFT_H = 0.07                  # Giữ lại (không dùng Phase 1)

# --- Kích thước mạng (70D Architecture) ---
HISTORY_LEN = 7
FRAME_DIM = 12                  # [qw,qx,qy,qz, wx_norm,wy_norm,wz_norm, prev_dx,prev_dy,prev_dz, prev_lift_l, prev_lift_r]
STATE_DIM = HISTORY_LEN * FRAME_DIM  # 84
ACTION_DIM = 5                  # [dx, dy, dz, lift_l, lift_r]

# --- Thông số PPO ---
GAMMA = 0.99
EPS_CLIP = 0.2
K_EPOCHS = 5
LR_ACTOR = 3e-4
LR_CRITIC = 1e-3
ENTROPY_COEFF = 0.01
MIN_UPDATE_STEPS = 1000
MAX_UPDATE_STEPS = 9999

# --- Action scaling (Đã TĂNG TỐI ĐA để đỡ đẩy mạnh) ---
# Worst-case IK check: sqrt(0.05² + 0.03² + 0.285²) = 0.2909 < SAFE_LIMIT 0.2927 ✓
# --- Action scaling (TĂNG ĐỂ SẢI BƯỚC MẠNH MẼ HƠN) ---
SCALE_DX = 0.05                 # ±8.0cm (Mở rộng để bước phục hồi xa hơn)
SCALE_DY = 0.03                 # ±5.0cm (Mở rộng chân bám trụ sang hai bên)
SCALE_DZ = 0.02                 # ÉP KHÓA LẠI ĐỂ CHỐNG NHÚN!
SCALE_LIFT = 0.025               # Max nhấc chân 5cm

# --- EMA Action Smoothing ---
# α=0.85: Phản ứng nhanh hơn với các cú đẩy mạnh (Phase 2).
# Effective response qua 2 tầng lọc: 0.85 * 0.6^(5 iterations) ≈ nhanh hơn.
EMA_ALPHA = 0.9                # Nâng lên để robot có thể nhấc chân gắt/dứt khoát

# --- Exploration STD decay ---
ACTION_STD_INIT = 0.6         # Giảm xuống vì có trọng số cũ, đủ để khám phá 2 chiều mới
ACTION_STD_MIN = 0.02           # Nâng lên chút cho an toàn
DECAY_FACTOR = 0.98
DECAY_INTERVAL_EP = 20

# --- Fall detection ---
FALL_ROLL_THRESHOLD = 0.6       # rad (~34°)
FALL_PITCH_THRESHOLD = 0.6

# --- Push force curriculum ---
PUSH_START_EPISODE = 1
PUSH_INTERVAL_STEPS = (150, 230)  # Khoảng ngẫu nhiên giữa 5-9s (20Hz → 100-180 steps)
PUSH_BASE_FORCE = 200.0
PUSH_MAX_FORCE =300.0  # Reduced initially to not overwhelm Phase 2 learning
PUSH_FORCE_INCREMENT = 4.0
PUSH_DURATION_SIM = 2.0  # Thời gian đẩy trên Gazebo (thực tế sẽ ngắn hơn do hình sin)

# --- Curriculum Reward ---
CURRICULUM_THRESHOLD = 400.0
W_RESTORE_MAX = 0.8
W_RESTORE_INCREMENT = 0.05

# --- Gyro chuẩn hóa ---
# BNO055 gyro max ~2000°/s ≈ 34.9 rad/s, nhưng khi ngã thường < 10 rad/s.
# Clip ±10 rad/s rồi chia 10 → gyro ∈ [-1, 1], cùng scale với quat và action.
MAX_GYRO = 10.0

# --- IMU Watchdog ---
IMU_TIMEOUT = 0.1               # 100ms không có IMU → cảnh báo


# =============================================================================
# HISTORY BUFFER: 7 bước × 12D = 84D Observation
# =============================================================================
class HistoryBuffer:
    """
    Buffer lịch sử 7 bước, mỗi bước 12 chiều.
    MG996R trễ >100ms → PPO cần 7 frames (350ms) để ước lượng trạng thái ngầm.
    """
    def __init__(self, history_len=HISTORY_LEN, frame_dim=FRAME_DIM):
        self.history_len = history_len
        self.frame_dim = frame_dim
        self.buffer = collections.deque(maxlen=history_len)
        self.reset()

    def reset(self):
        self.buffer.clear()
        for _ in range(self.history_len):
            self.buffer.append(np.zeros(self.frame_dim))

    def push(self, frame):
        assert len(frame) == self.frame_dim
        self.buffer.append(frame.copy())

    def get_observation(self):
        return np.concatenate(list(self.buffer))


# =============================================================================
# MẠNG NEURAL: Actor-Critic
# =============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init):
        super(ActorCritic, self).__init__()
        self.action_var = torch.full(
            (action_dim,), action_std_init * action_std_init).to(DEVICE)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ELU(),
            nn.Linear(256, 256), nn.ELU(),
            nn.Linear(256, action_dim), nn.Tanh())
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256), nn.ELU(),
            nn.Linear(256, 256), nn.ELU(),
            nn.Linear(256, 1))

    def set_action_std(self, new_std):
        self.action_var = torch.full(
            (ACTION_DIM,), new_std * new_std).to(DEVICE)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        value = self.critic(state)
        return action.detach(), log_prob.detach(), value.detach()

    def evaluate(self, state, action):
        action_mean = self.actor(state)
        action_var_expanded = self.action_var.expand_as(action_mean)
        dist = Normal(action_mean, torch.sqrt(action_var_expanded))
        log_probs = dist.log_prob(action).sum(dim=-1)
        state_values = self.critic(state)
        dist_entropy = dist.entropy().sum(dim=-1)
        return log_probs, state_values, dist_entropy


# =============================================================================
# PPO ALGORITHM
# =============================================================================
class PPO:
    def __init__(self, state_dim, action_dim):
        self.policy = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': LR_ACTOR},
            {'params': self.policy.critic.parameters(), 'lr': LR_CRITIC}])
        self.policy_old = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logprobs = []
        self.buffer_rewards = []
        self.buffer_is_terminals = []

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
        if min_size < 1:
            self.clear_buffer()
            return
        print(f"\n\033[1;32m[PPO UPDATE] >>> {min_size} steps, K={K_EPOCHS}\033[0m")
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(
            reversed(self.buffer_rewards[:min_size]),
            reversed(self.buffer_is_terminals[:min_size])):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + GAMMA * discounted_reward
            rewards.insert(0, discounted_reward)
        rewards_t = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        std = rewards_t.std()
        if std > 1e-6:
            rewards_t = (rewards_t - rewards_t.mean()) / (std + 1e-7)
        else:
            rewards_t = rewards_t - rewards_t.mean()
        old_states = torch.stack(self.buffer_states[:min_size]).detach()
        old_actions = torch.stack(self.buffer_actions[:min_size]).detach()
        old_logprobs = torch.stack(self.buffer_logprobs[:min_size]).detach()
        for _ in range(K_EPOCHS):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards_t - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1 - EPS_CLIP, 1 + EPS_CLIP) * advantages
            loss = (-torch.min(surr1, surr2)
                    + 0.5 * self.MseLoss(state_values.squeeze(), rewards_t)
                    - ENTROPY_COEFF * dist_entropy)
            self.optimizer.zero_grad()
            loss.mean().backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=0.5)
            self.optimizer.step()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.clear_buffer()

    def clear_buffer(self):
        self.buffer_states.clear()
        self.buffer_actions.clear()
        self.buffer_logprobs.clear()
        self.buffer_rewards.clear()
        self.buffer_is_terminals.clear()


# =============================================================================
# MAIN NODE: PPO Balance Training
# =============================================================================
class PPOBalanceNode(Node):
    def __init__(self):
        super().__init__('ppo_balance_node')

        # --- Paths ---
        self.base_dir = "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/weights"
        self.weight_dir = os.path.join(self.base_dir, "phase2_balance")
        os.makedirs(self.weight_dir, exist_ok=True)
        self.latest_model_path = os.path.join(self.weight_dir, "latest_model.pth")
        self.best_model_path = os.path.join(self.weight_dir, "best_model.pth")
        self.meta_path = os.path.join(self.weight_dir, "training_meta.json")
        self.history_path = os.path.join(self.weight_dir, "training_history.csv")

        # --- QoS ---
        force_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Publishers ---
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # Array of publishers for all 17 joints to force loosening during reset
        # Cố định Thân và Tay trên Gazebo Harmonic vì không nằm trong controller_config.yaml
        self.upper_joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in
                           ['base_hip_middle',
                            'hip_shoulder_left', 'shoulder_shoulder_left', 'shoulder_elbow_left',
                            'hip_shoulder_right', 'shoulder_shoulder_right', 'shoulder_elbow_right']}
        self.leg_joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in
                           ['base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
                            'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right']}

        # --- Subscribers ---
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, force_qos)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, force_qos)

        # --- Service clients ---
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # --- IMU state ---
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])    # [qw, qx, qy, qz]
        self.gyro = np.zeros(3)                          # [wx, wy, wz] rad/s (RAW)

        # --- IMU Watchdog ---
        # Nếu IMU không publish > 0.1s, dữ liệu cũ là rác → KHÔNG nạp buffer
        self.last_imu_time = 0.0
        self.imu_stale = True  # Bắt đầu = stale, chờ frame đầu tiên

        # --- IMU Offset (Tare) ---
        self.roll_offset = 0.0
        self.pitch_offset = 0.0

        # --- History Buffer ---
        self.history_buffer = HistoryBuffer(HISTORY_LEN, FRAME_DIM)

        # --- EMA Action Smoothing ---
        self.smoothed_action = np.zeros(ACTION_DIM)
        self.prev_raw_action = np.zeros(ACTION_DIM)

        # --- Curriculum ---
        self.w_restore = 0.0
        self.reward_history_100 = collections.deque(maxlen=100)

        # --- Training state ---
        self.current_sim_time = 0.0
        self.best_reward = -float('inf')
        self.start_episode = 0
        self.current_episode = 0
        self.step_in_episode = 0
        self.ep_reward = 0.0
        self.episode_reason = "TIME_LIMIT"
        self.current_std = ACTION_STD_INIT
        self.next_push_step = -1
        
        # --- BIẾN TRẠNG THÁI CHO BÀN TAY ĐẨY TỪ TỪ ---
        self.is_pushing = False
        self.push_current_step = 0
        self.push_total_steps = 0
        self.push_max_f = 0.0
        self.push_axis = 'y'
        self.push_dir = 1

        # --- PPO ---
        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()

        # --- CSV ---
        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f:
                csv.writer(f).writerow(["Episode", "Reward", "Steps", "Reason", "Time"])

        self.get_logger().info(
            f">>> PPO 70D Ready | STATE={STATE_DIM} ACTION={ACTION_DIM} "
            f"EMA={EMA_ALPHA} SCALE_DX={SCALE_DX} SCALE_DY={SCALE_DY} "
            f"R_jitter=-1.5 | STD={self.current_std:.4f}")

        self.state_lock = threading.Lock()
        threading.Thread(target=self.train_loop, daemon=True).start()

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def imu_callback(self, msg: Imu):
        with self.state_lock:
            self.quat[0] = msg.orientation.w
            self.quat[1] = msg.orientation.x
            self.quat[2] = msg.orientation.y
            self.quat[3] = msg.orientation.z
            self.gyro[0] = msg.angular_velocity.x
            self.gyro[1] = msg.angular_velocity.y
            self.gyro[2] = msg.angular_velocity.z
            # Cập nhật timestamp cho watchdog
            self.last_imu_time = time.time()
            self.imu_stale = False

        # Trực tiếp cố định các khớp tay và eo ở 0 để tay không rũ xuống
        if hasattr(self, 'upper_joint_pubs'):
            for pub in self.upper_joint_pubs.values():
                pub.publish(Float64(data=0.0))

    def clock_callback(self, msg: Clock):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    # =========================================================================
    # OBSERVATION: 70D (với gyro chuẩn hóa + NaN protection + watchdog)
    # =========================================================================
    def get_observation(self):
        """
        Tạo frame 10D, chuẩn hóa gyro, lọc NaN, push vào buffer.
        Nếu IMU stale > 0.1s → WARN và KHÔNG push frame mới (giữ frame cũ).
        """
        is_timeout = time.time() - self.last_imu_time > IMU_TIMEOUT
        if is_timeout or self.imu_stale:
            if not self.imu_stale:
                self.get_logger().warn(
                    f"⚠️ IMU STALE! Không nhận data > {IMU_TIMEOUT}s. Giữ frame cũ.")
                self.imu_stale = True
            # Không push frame mới → buffer giữ nguyên frame cuối cùng hợp lệ
            return self.history_buffer.get_observation()

        with self.state_lock:
            quat_copy = self.quat.copy()
            gyro_raw = self.gyro.copy()

        # --- Chuẩn hóa Gyro: clip ±10 rad/s rồi chia 10 → [-1, 1] ---
        # Đảm bảo cùng scale với quaternion [-1,1] và action [-1,1],
        # tránh gradient explosion khi robot ngã (gyro spike).
        gyro_norm = np.clip(gyro_raw, -MAX_GYRO, MAX_GYRO) / MAX_GYRO

        # --- Tạo frame 12D ---
        frame = np.concatenate([
            quat_copy,                  # 4D: [qw, qx, qy, qz] ∈ [-1, 1]
            gyro_norm,                  # 3D: [wx, wy, wz] normalized ∈ [-1, 1]
            self.smoothed_action.copy() # 5D: [dx, dy, dz, lift_l, lift_r] ∈ [-1, 1]
        ])

        # --- NaN Protection ---
        # Nếu sensor trả NaN (chia zero, lỗi phần cứng), thay bằng 0.
        # Tránh nhiễm NaN vào network → phá hủy weights .pth vĩnh viễn.
        frame = np.nan_to_num(frame, nan=0.0, posinf=1.0, neginf=-1.0)

        self.history_buffer.push(frame)
        return self.history_buffer.get_observation()

    # =========================================================================
    # REWARD: Gaussian + Curriculum (R_jitter GIẢM xuống -1.5)
    # =========================================================================
    def compute_reward(self, action, prev_smoothed):
        """
        R_balance = exp(-10 * (pitch² + roll²))     — Gaussian, max 1.0
        R_jitter  = -1.5 * sum((a - a_prev)²)       — giảm từ -5.0 tránh collapse
        R_yaw     = -3.0 * wz²                      — phạt xoay quanh trục Z
        R_restore = exp(-||ω||²) * exp(-2*||a||²)   — curriculum w_restore
        """
        with self.state_lock:
            qw, qx, qy, qz = self.quat.copy()
            gyro_copy = self.gyro.copy()
            
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        raw_roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            raw_pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            raw_pitch = math.asin(sinp)
            
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Áp dụng IMU offset cho pitch và roll
        roll = raw_roll - self.roll_offset
        pitch = raw_pitch - self.pitch_offset
        
        # UW TIÊN SỐ 1: Thưởng sống sót cố định mỗi khung hình. 
        # Giảm xuống 0.5 để ép robot mạo hiểm nhấc chân thay vì đứng im chịu trận
        r_alive = 5

        # TĂNG phạt tư thế để ưu tiên thăng bằng tuyệt đối
        r_balance = 3.0 * math.exp(-15.0 * (pitch**2 + roll**2))

        # R_jitter: Giữ ở -2.5 để action không bị rung (MG996R sẽ nóng nếu rung)
        action_diff = self.smoothed_action - prev_smoothed
        r_jitter = -0.5 * float(np.sum(action_diff ** 2))

        # R_yaw: BỎ QUA - Không phạt vặn mình
        r_yaw = 0.0

        # R_restore: Thưởng khích lệ đứng yên khi đã vào phom
        w_res = 0.01
        omega_sq = float(np.sum(gyro_copy ** 2))
        action_sq = float(np.sum(action ** 2))
        r_restore = math.exp(-1.0 * omega_sq) * math.exp(-2.0 * action_sq)
        
        # R_lift penalty: Tránh nhấc chân cao vô tội vạ nếu không cần thiết
        # lift_l = max(0.0, float(self.smoothed_action[3] * SCALE_LIFT))
        # lift_r = max(0.0, float(self.smoothed_action[4] * SCALE_LIFT))
        # r_lift = -0.2 * (lift_l**2 + lift_r**2)
        r_lift = 0.0
        if abs(pitch) > 0.2 or abs(roll) > 0.2:
            lift_val = np.sum(np.maximum(0, self.smoothed_action[3:5]))
            r_lift = 1.0 * lift_val # Thưởng nhấc chân khi mất thăng bằng

        # total_reward = r_alive + r_balance + r_jitter + self.w_restore * r_restore + r_lift
        total_reward = r_alive + r_balance + r_jitter + (w_res * r_restore) + r_lift
        return total_reward, roll, pitch

    # =========================================================================
    # CHECK SAFETY & PREPARE IK COMMAND
    # =========================================================================
    def check_safety_and_prepare_cmd(self, raw_action):
        # Cartesian Clamping
        clamped_action = np.clip(raw_action, -1.0, 1.0)

        # EMA Smoothing (α=0.85)
        self.smoothed_action = EMA_ALPHA * clamped_action + (1.0 - EMA_ALPHA) * self.smoothed_action

        # Scale → giá trị vật lý
        dx = float(self.smoothed_action[0]) * SCALE_DX
        dy = float(self.smoothed_action[1]) * SCALE_DY
        dz = float(self.smoothed_action[2]) * SCALE_DZ
        tz = STD_Z + dz  # BIẾN TZ QUAN TRỌNG ĐỂ TÍNH ĐỘ DÀI CHÂN

        stance = 0.0
        tx_l = dx
        tx_r = dx
        target_y_l = STD_Y + dy + stance
        target_y_r = -STD_Y + dy - stance
        
        # === NHẤC CHÂN LOGIC (PHASE 2) ===
        # Lấy từ action space 4 và 5, giới hạn chỉ được dương
        lift_l_raw = float(self.smoothed_action[3]) * SCALE_LIFT
        lift_r_raw = float(self.smoothed_action[4]) * SCALE_LIFT
        lift_l = max(0.0, lift_l_raw)
        lift_r = max(0.0, lift_r_raw)
        
        # Chiều dài Z thực tế chân phải vươn tới (sau khi bị nhấc)
        # BẢO VỆ GẬP CHÂN mềm (soft clip tz >= 0.15) để robot không bị phạt chết oan
        tz_left = max(0.15, min(0.285, tz - lift_l))
        tz_right = max(0.15, min(0.285, tz - lift_r))

        # IK limits check (GIỮ NGUYÊN)
        len_l = np.linalg.norm([tx_l, target_y_l, tz_left])
        len_r = np.linalg.norm([tx_r, target_y_r, tz_right])

        if len_l > SAFE_LIMIT or len_r > SAFE_LIMIT:
            return False, None, -5.0

        msg = Float64MultiArray()
        msg.data = [tx_l, target_y_l, tz_left, lift_l,
                    tx_r, target_y_r, tz_right, lift_r, MOVE_DURATION]
        return True, msg, tz

    # =========================================================================
    # FALL DETECTION
    # =========================================================================
    def is_fallen(self):
        with self.state_lock:
            qw, qx, qy, qz = self.quat.copy()
            
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        raw_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (qw * qy - qz * qx)
        raw_pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        # So sánh theo ngưỡng hiệu dụng: Dùng góc raw trừ đi offset
        roll = raw_roll - self.roll_offset
        pitch = raw_pitch - self.pitch_offset
        
        # Lấy tz hiện tại từ smoothed action để check kịch trần
        tz = STD_Z + float(self.smoothed_action[2]) * SCALE_DZ
        
        return abs(roll) > FALL_ROLL_THRESHOLD or abs(pitch) > FALL_PITCH_THRESHOLD or tz > 0.288

    # =========================================================================
    # PUSH FORCE (ĐÃ NÂNG CẤP: BÀN TAY NGƯỜI ĐẨY TỪ TỪ - HALF-SINE WAVE)
    # =========================================================================
    def trigger_push_event(self):
        """Hàm này chỉ khởi tạo kịch bản đẩy, chưa bắn lực ngay"""
        min_f = PUSH_BASE_FORCE
        max_f = min(PUSH_MAX_FORCE, PUSH_BASE_FORCE + (self.current_episode // 40) * PUSH_FORCE_INCREMENT)

        self.push_max_f = np.random.uniform(min_f, max_f)
        self.push_dir = np.random.choice([-1, 1])
        self.push_axis = np.random.choice(['y', 'x', 'xy'], p=[0.7, 0.1, 0.2])
        
        self.is_pushing = True
        self.push_current_step = 0
        # Số frame cần thiết để đẩy xong (VD: 1.0s / 0.05s = 20 frames)
        self.push_total_steps = max(1, int(PUSH_DURATION_SIM / DT))
        
        self.get_logger().info(f"🖐️ BẮT ĐẦU ĐẨY TỪ TỪ: Max {self.push_max_f:.0f}N theo trục {self.push_axis.upper()} trong {PUSH_DURATION_SIM}s")

    def process_continuous_push(self):
        """Hàm này được gọi mỗi 50ms để bơm lực theo hình Sin"""
        if not self.is_pushing:
            return

        # Tính tỷ lệ phần trăm chu kỳ đẩy (từ 0.0 đến 1.0)
        progress = self.push_current_step / float(self.push_total_steps)
        # Hệ số Sine tạo sự êm ái: bắt đầu 0 -> đỉnh 1 -> kết thúc 0
        sine_multiplier = math.sin(math.pi * progress)
        
        # Lực tịnh tiến tại mili-giây hiện tại
        current_f = self.push_max_f * sine_multiplier * self.push_dir
        
        model_name = "humanoid_robot"
        if self.push_axis == 'y':
            force_str = f"{{y: {current_f:.1f}}}"
        elif self.push_axis == 'x':
            force_str = f"{{x: {current_f:.1f}}}"
        else:
            f_val_x = current_f * 0.707
            f_val_y = current_f * 0.707 * np.random.choice([-1, 1])
            force_str = f"{{x: {f_val_x:.1f}, y: {f_val_y:.1f}}}"

        msg_str = (f"entity: {{name: '{model_name}', type: MODEL}}, wrench: {{force: {force_str}}}")

        # Bắn lệnh cập nhật lực liên tục xuống Gazebo
        cmd_on = ["gz", "topic", "-t", "/world/empty/wrench",
                  "-m", "gz.msgs.EntityWrench", "-p", msg_str]
        subprocess.Popen(cmd_on, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.push_current_step += 1

        # Nếu đã hoàn thành chu kỳ đẩy -> Buông tay
        if self.push_current_step > self.push_total_steps:
            self.is_pushing = False
            msg_zero = (f"entity: {{name: '{model_name}', type: MODEL}}, wrench: {{force: {{x: 0, y: 0, z: 0}}}}")
            cmd_off = ["gz", "topic", "-t", "/world/empty/wrench",
                      "-m", "gz.msgs.EntityWrench", "-p", msg_zero]
            subprocess.Popen(cmd_off, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("🛑 BUÔNG TAY (Kết thúc lực đẩy)")

    # =========================================================================
    # SIM SLEEP & RESET (GIỮ NGUYÊN logic)
    # =========================================================================
    def sim_sleep(self, duration):
        start = self.current_sim_time
        while self.current_sim_time - start < duration:
            time.sleep(0.001)

    def reset_simulation(self):
        while not self.w_cli.wait_for_service(timeout_sec=1.0):
            pass
        # === DỌN DẸP LỰC ĐẨY CŨ TRƯỚC ===
        # Nếu robot ngã ngay trong lúc bị đẩy, lực đẩy vẫn còn tồn tại mãi trong Gazebo
        # Ta phải xóa (zero) lực đẩy này ngay lập tức.
        msg_zero = "entity: {name: 'humanoid_robot', type: MODEL}, wrench: {force: {x: 0, y: 0, z: 0}}"
        cmd_off = ["gz", "topic", "-t", "/world/empty/wrench",
                   "-m", "gz.msgs.EntityWrench", "-p", msg_zero]
        subprocess.Popen(cmd_off, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if hasattr(self, 'is_pushing'):
            self.is_pushing = False

        # === RESET FLOW CẬP NHẬT THEO YÊU CẦU MỚI ===
        # Robot ngã -> Nằm dưới sân -> Thu chân về vị trí thẳng -> Chờ rụt xong -> Teleport

        # 1. Đảm bảo physics đang chạy (UNPAUSE) để rụt chân được
        req_u = ControlWorld.Request()
        req_u.world_control = WorldControl(pause=False)
        self.w_cli.call_async(req_u)
        time.sleep(0.1)

        # 2. Phát lệnh đứng thẳng (Action trung lập) tới LLC để kéo chân về chuẩn
        self.get_logger().info(">>> Retracting legs on the floor...")
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(3):
            self.action_pub.publish(msg)
            time.sleep(0.1)
            
        # Chờ 1.2 giây để đôi chân kéo giãn về từ từ nhờ bộ lọc Alpha của LLC
        time.sleep(1.2)

        # 3. Báo LLC tạm ngất (Dừng nhận lệnh) rỗng để teleport không bị lỗi
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.2)

        # 4. PAUSE PHYSICS để teleport chuẩn xác (Gazebo Harmonic yêu cầu)
        req_p = ControlWorld.Request()
        req_p.world_control = WorldControl(pause=True)
        self.w_cli.call_async(req_p)
        time.sleep(0.2)

        # 5. Teleport robot về tâm (Z=0.28 - vừa đủ chạm sàn)
        req_s = SetEntityPose.Request()
        req_s.entity = Entity(name="humanoid_robot")
        req_s.pose = Pose()
        req_s.pose.position.z = 0.28
        req_s.pose.orientation.w = 1.0
        self.p_cli.call_async(req_s)
        time.sleep(0.5)

        # 6. UNPAUSE PHYSICS lại để bắt đầu vòng lặp mới
        req_u = ControlWorld.Request()
        req_u.world_control = WorldControl(pause=False)
        self.w_cli.call_async(req_u)
        
        # 7. Chờ sim clock chạy lại
        sc = self.current_sim_time
        while self.current_sim_time - sc < 0.05:
            time.sleep(0.01)

        # 8. Báo LLC bắt đầu nhận lệnh lại

        self.reset_pub.publish(Bool(data=False))

        # Reset states
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.zeros(3)
        self.history_buffer.reset()
        self.smoothed_action = np.zeros(ACTION_DIM)
        self.prev_raw_action = np.zeros(ACTION_DIM)
        self.last_imu_time = time.time()
        self.imu_stale = True
        self.step_in_episode = 0
        self.ep_reward = 0.0
        self.episode_reason = "TIME_LIMIT"
        self.is_pushing = False
        if self.current_episode >= PUSH_START_EPISODE:
            self.next_push_step = np.random.randint(*PUSH_INTERVAL_STEPS)
        else:
            self.next_push_step = -1
        return self.stabilize_robot()

    def stabilize_robot(self):
        """Stabilize giống ft_force.py: dùng gyro để xác định trạng thái tĩnh thay vì chờ thời gian cứng."""
        self.get_logger().info(">>> Stabilizing...")
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        # Publish 3 lần liên tiếp để đảm bảo LLC nhận được (giống ft_force.py)
        for _ in range(3):
            self.action_pub.publish(msg)
            time.sleep(0.1)

        # BƯỚC 1: Chờ ít nhất 1 frame IMU thật đến (kiên nhẫn chờ imu_stale = False)
        deadline_imu = time.time() + 3.0
        while self.imu_stale and time.time() < deadline_imu:
            self.action_pub.publish(msg)
            time.sleep(0.05)

        # BƯỚC 2: Chờ gyro về gần 0 (dựa trên data IMU thật)
        deadline = time.time() + 5.0   # Timeout tối đa 5s
        while time.time() < deadline:
            with self.state_lock:
                gyro_mag = np.linalg.norm(self.gyro)
            if gyro_mag < 0.05:
                break
            self.action_pub.publish(msg)
            time.sleep(0.1)
            
        # BƯỚC 3: Lấy trung bình 10 frame IMU để tare chính xác hơn
        samples = []
        for _ in range(10):
            with self.state_lock:
                samples.append(self.quat.copy())
            time.sleep(0.05)
            
        avg_quat = np.mean(samples, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)
        qw, qx, qy, qz = avg_quat
        
        roll_offset = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
        pitch_offset = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
        
        # Ngăn chặn ảo tưởng sức mạnh: Nếu robot đang nằm trên sàn (góc quá lớn), không được phép lấy offset!
        if abs(roll_offset) > 0.4 or abs(pitch_offset) > 0.4:
            self.get_logger().warn(f"⚠️ Reset thất bại do robot đổ gục (roll={roll_offset:.2f}, pitch={pitch_offset:.2f}). Thử lại...")
            return False
            
        self.roll_offset = roll_offset
        self.pitch_offset = pitch_offset
            
        self.smoothed_action = np.zeros(ACTION_DIM)
        self.prev_raw_action = np.zeros(ACTION_DIM)
        return True

    # =========================================================================
    # SAVE / LOAD (w_restore persist)
    # =========================================================================
    def transfer_weights(self, old_model_path, new_model):
        self.get_logger().info(f">>> Performing Surgery: Transferring weights from {old_model_path}")
        old_state = torch.load(old_model_path, map_location=DEVICE, weights_only=True)
        new_state = new_model.state_dict()
        
        for name, param in old_state.items():
            if name in new_state:
                if param.shape == new_state[name].shape:
                    new_state[name].copy_(param)
                else:
                    self.get_logger().info(f"Surgery on layer: {name}. Old: {param.shape}, New: {new_state[name].shape}")
                    # Đây là nơi xử lý lớp Output cuối cùng (lệch shape)
                    # Chỉ copy 3 dòng đầu (cho dx, dy, dz)
                    if len(param.shape) == 2:
                        new_state[name][:3, :] = param[:3, :] # Nếu là weight
                    elif len(param.shape) == 1:
                        new_state[name][:3] = param[:3] # Nếu là bias
        new_model.load_state_dict(new_state)

    def load_training_state(self):
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                data = json.load(f)
                self.best_reward = data.get('best_reward', -float('inf'))
                self.start_episode = data.get('current_episode', 0) + 1
                self.current_std = data.get('action_std', ACTION_STD_INIT)
                self.w_restore = data.get('w_restore', 0.0)
                
                # Trong Phase 2, lúc đầu nên giảm W_RESTORE về 0.1 để nó chịu khó khám phá nhấc chân
                if self.w_restore > 0.1:
                    self.get_logger().info(f"Forcing W_RESTORE to 0.1 for Phase 2 start. (Old was: {self.w_restore:.2f})")
                    self.w_restore = 0.1

        if os.path.exists(self.latest_model_path):
            self.transfer_weights(self.latest_model_path, self.ppo.policy)
            self.ppo.policy_old.load_state_dict(self.ppo.policy.state_dict())
            self.ppo.policy.set_action_std(self.current_std)
            self.ppo.policy_old.set_action_std(self.current_std)
            self.get_logger().info(
                f">>> Surgery Loaded: STD={self.current_std:.4f} best={self.best_reward:.2f} "
                f"w_restore={self.w_restore:.2f}")
        self.current_episode = self.start_episode

    def save_state(self, ep, reward, is_best=False):
        data = {"best_reward": float(self.best_reward),
                "current_episode": int(ep),
                "action_std": float(self.current_std),
                "w_restore": float(self.w_restore)}
        with open(self.meta_path, 'w') as f:
            json.dump(data, f, indent=4)
        torch.save(self.ppo.policy.state_dict(), self.latest_model_path)
        if is_best:
            torch.save(self.ppo.policy.state_dict(), self.best_model_path)

    def log_episode(self, ep, reward, steps, reason):
        with open(self.history_path, 'a', newline='') as f:
            csv.writer(f).writerow([ep, f"{reward:.2f}", steps, reason,
                                    datetime.now().strftime("%H:%M:%S")])

    # =========================================================================
    # CURRICULUM UPDATE
    # =========================================================================
    def update_curriculum(self):
        if len(self.reward_history_100) >= 100:
            mean_r = np.mean(self.reward_history_100)
            if mean_r > CURRICULUM_THRESHOLD and self.w_restore < W_RESTORE_MAX:
                old_w = self.w_restore
                self.w_restore = min(W_RESTORE_MAX, self.w_restore + W_RESTORE_INCREMENT)
                if self.w_restore != old_w:
                    self.get_logger().info(
                        f"📈 CURRICULUM: w_restore {old_w:.2f} → {self.w_restore:.2f} "
                        f"(mean_100={mean_r:.1f})")

    # =========================================================================
    # MAIN TRAIN LOOP
    # =========================================================================
    def train_loop(self):
        while self.current_sim_time == 0.0:
            time.sleep(0.1)
        self.get_logger().info(">>> Sim clock detected. Training start (70D + hardened).")

        # 0. Đợi cho đến khi hệ thống nhận được IMU và ổn định xong
        self.get_logger().info(">>> Waiting for IMU stabilization...")
        while self.imu_stale and rclpy.ok():
            time.sleep(0.1)

        while rclpy.ok():
            if not self.reset_simulation():
                time.sleep(0.5)
                continue
            ep = self.current_episode

            while rclpy.ok():
                state = self.get_observation()
                raw_action = self.ppo.select_action(state)

                # Đầu inner loop, trước khi check_safety update smoothed_action
                prev_smoothed_snapshot = self.smoothed_action.copy()

                ok, msg, tz = self.check_safety_and_prepare_cmd(raw_action)
                if not ok:
                    self.ppo.buffer_rewards.append(-100.0)
                    self.ppo.buffer_is_terminals.append(True)
                    self.episode_reason = "IK_FAIL"
                    break

                self.action_pub.publish(msg)
                
                # --- THỰC THI LỰC ĐẨY TỪ TỪ NẾU ĐANG CÓ SỰ KIỆN ---
                self.process_continuous_push()

                # --- KÍCH HOẠT SỰ KIỆN ĐẨY MỚI ---
                if (self.current_episode >= PUSH_START_EPISODE and
                        self.next_push_step > 0 and
                        self.step_in_episode >= self.next_push_step and 
                        not self.is_pushing):  # Tránh đè sự kiện đẩy khi đang đẩy dở
                    
                    self.trigger_push_event()
                    self.next_push_step = (self.step_in_episode +
                                           np.random.randint(*PUSH_INTERVAL_STEPS))

                self.sim_sleep(DT)

                done = self.is_fallen()
                reward, roll, pitch = self.compute_reward(raw_action, prev_smoothed_snapshot)
                if done:
                    reward = -100.0

                self.ppo.buffer_rewards.append(reward)
                self.ppo.buffer_is_terminals.append(done)
                self.ep_reward += reward
                self.prev_raw_action = raw_action.copy()
                self.step_in_episode += 1

                # Mid-episode update nếu buffer quá lớn (tránh bloat memory và thay đổi policy quá gắt)
                if len(self.ppo.buffer_states) >= MAX_UPDATE_STEPS:
                    self.ppo.update()

                if done:
                    self.episode_reason = "FALL"
                    break
                elif self.step_in_episode >= MAX_STEPS:
                    self.episode_reason = "TIME_LIMIT"
                    break

            # --- Post-episode ---
            # === SKIP GARBAGE EPISODES ===
            # Nếu robot ngã ngay ≤1 step → do reset không ổn định, KHÔNG phải
            # do policy tệ. Nạp data này vào mạng sẽ dạy nó "luôn ngã" → collapse.
            # Xóa buffer entries của episode rác này ra khỏi PPO buffer.
            if self.step_in_episode <= 1:
                # Sửa lỗi: Chỉ pop đúng số lượng step đã chạy trong episode này
                n_pop = self.step_in_episode
                for _ in range(n_pop):
                    if self.ppo.buffer_states:
                        self.ppo.buffer_states.pop()
                        self.ppo.buffer_actions.pop()
                        self.ppo.buffer_logprobs.pop()
                        self.ppo.buffer_rewards.pop()
                        self.ppo.buffer_is_terminals.pop()
                self.get_logger().warn(
                    f"🗑️ Ep {ep} SKIPPED (steps={self.step_in_episode}) — "
                    f"reset instability, not updating network.")
                self.log_episode(ep, self.ep_reward, self.step_in_episode,
                                 "SKIP_RESET")
                self.current_episode += 1
                continue  # Bỏ qua episode này hoàn toàn

            is_best = self.ep_reward > self.best_reward
            if is_best:
                self.best_reward = self.ep_reward
                self.get_logger().info(f"⭐ NEW BEST: {self.ep_reward:.2f}")

            # Đảm bảo record reward vào history trước khi tính trung bình (mean_recent)
            self.reward_history_100.append(self.ep_reward)
            self.update_curriculum()

            # Xử lý decay STD: Thay đổi dựa trên average reward thay vì chỉ khi có best
            if ep > 0 and ep % DECAY_INTERVAL_EP == 0:
                # mean_recent = np.mean(self.reward_history_100) if self.reward_history_100 else -999.0
                # if mean_recent > CURRICULUM_THRESHOLD:
                self.current_std = max(ACTION_STD_MIN, self.current_std * DECAY_FACTOR)
                self.ppo.policy.set_action_std(self.current_std)
                self.ppo.policy_old.set_action_std(self.current_std)

            # Cập nhật network cuối episode (Chỉ khi tích đủ MIN data)
            # Nếu chưa đủ MIN_UPDATE_STEPS, giữ nguyên buffer để tích lũy sang các episode ngắn tiếp theo.
            if len(self.ppo.buffer_states) >= MIN_UPDATE_STEPS:
                self.ppo.update()

            self.save_state(ep, self.ep_reward, is_best)
            self.log_episode(ep, self.ep_reward, self.step_in_episode, self.episode_reason)
            self.get_logger().info(
                f"Ep {ep} | R:{self.ep_reward:.2f} | Best:{self.best_reward:.2f} | "
                f"S:{self.step_in_episode} | STD:{self.current_std:.4f} | "
                f"w:{self.w_restore:.2f} | {self.episode_reason}")
            self.current_episode += 1


# =============================================================================
# ENTRY POINT
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PPOBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(">>> Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
