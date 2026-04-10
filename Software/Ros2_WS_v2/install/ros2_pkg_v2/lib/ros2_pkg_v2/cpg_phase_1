#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Phase 3: PPO + CPG Walking — 126D History Buffer + BNO055 Contact Estimation
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool, Float64
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
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

# --- Thông số IK ---
MAX_LEG_LENGTH = 0.293          # L3+L4+L5 - 0.002mm an toàn
SAFE_LIMIT = MAX_LEG_LENGTH * 0.999
STD_Z = 0.265                  # Chiều cao đứng chuẩn
STD_Y = 0.01

# ─── Dimensions ───────────────────────────────────────────────
FRAME_DIM      = 18    # [quat(4), gyro(3), lin_acc(3), action_cũ(6), sin_phi, cos_phi]
STATE_DIM      = 126   # 7 frames × 18
ACTION_DIM     = 6     # [res_dx_l, res_dx_r, res_dy, res_dz, res_lift_l, res_lift_r]

# ─── CPG ──────────────────────────────────────────────────────
CPG_PERIOD     = 1.0   # giây — 1 chu kỳ bước
CPG_AMP_DX     = 0.04  # mét — biên độ bước chân trên trục X
CPG_AMP_LIFT   = 0.03  # mét — độ nhấc chân
CPG_AMP_DY     = 0.01  # mét — lắc hông nhẹ

# ─── PPO Residual ─────────────────────────────────────────────
RESIDUAL_SCALE = 1.0   # Full can thiệp cho stepping balance
EMA_ALPHA      = 0.85  # Smoothing nhanh cho stepping

# ─── BNO055 Contact Estimation ────────────────────────────────
LIN_ACC_Z_IMPACT_THRESHOLD = 1.5   # m/s² — ngưỡng phát hiện chân chạm đất
LIN_ACC_Z_FLIGHT_THRESHOLD = 0.4   # m/s² — ngưỡng phát hiện cả 2 chân bay
CONTACT_WINDOW             = 5     # frames — cửa sổ averaging để tránh nhiễu

CMD_VEL_X = 0.05  # m/s — mục tiêu vận tốc thực tế của thân robot cho Phase 3

# --- Kích thước mạng (History Buffer) ---
HISTORY_LEN = 7

# --- Thông số PPO ---
GAMMA = 0.99
EPS_CLIP = 0.2
K_EPOCHS = 5
LR_ACTOR = 3e-4
LR_CRITIC = 1e-3
ENTROPY_COEFF = 0.01
MIN_UPDATE_STEPS = 1000
MAX_UPDATE_STEPS = 9999

# --- Exploration STD decay ---
ACTION_STD_INIT = 0.6
ACTION_STD_MIN = 0.02
DECAY_FACTOR = 0.98
DECAY_INTERVAL_EP = 20

# --- Fall detection ---
FALL_ROLL_THRESHOLD = 0.6       # rad (~34°)
FALL_PITCH_THRESHOLD = 0.6

# --- Gyro chuẩn hóa ---
MAX_GYRO = 10.0

# --- IMU Watchdog ---
IMU_TIMEOUT = 0.1               # 100ms không có IMU → cảnh báo

# LỘ TRÌNH TRAIN: Stepping Balance Recovery (Fixed Height)
STEPPING_BALANCE_MODE = True

# =============================================================================
# MODULE CPG & CONTACT ESTIMATOR
# =============================================================================
class GaitCPG:
    def __init__(self, dt=DT):
        self.dt    = dt
        self.T     = CPG_PERIOD
        self.phase = 0.0

    def step(self):
        """
        GỌI ĐÚNG 1 LẦN DUY NHẤT mỗi timestep — ở đầu vòng lặp train_loop.
        """
        self.phase = (self.phase + 2 * math.pi * self.dt / self.T) % (2 * math.pi)
        phi = self.phase

        # CPG disabled for Step Balance Recovery
        dx_l   =  0.0
        dx_r   =  0.0   
        dy     =  0.0              
        dz     =  0.0                                        
        lift_l =  0.0
        lift_r =  0.0

        return np.array([dx_l, dx_r, dy, dz, lift_l, lift_r], dtype=np.float32)

    def get_phase_encoding(self):
        """Đưa pha vào observation để PPO biết mình đang ở đâu trong chu kỳ."""
        return math.sin(self.phase), math.cos(self.phase)


class BNO055ContactEstimator:
    """
    Ước tính tiếp xúc chân dựa trên linear_acceleration của BNO055.
    """
    def __init__(self, window=CONTACT_WINDOW):
        self.window     = window
        self.acc_z_buf  = collections.deque(maxlen=window)
        self.both_flight_count = 0

    def reset(self):
        self.acc_z_buf.clear()
        self.both_flight_count = 0

    def update(self, lin_acc_z: float, lift_l_cmd: float, lift_r_cmd: float) -> dict:
        self.acc_z_buf.append(abs(lin_acc_z))

        # ── Tầng 1: BNO055 freefall detection ──────────────────────────
        if len(self.acc_z_buf) < self.window:
            avg_acc_z = sum(self.acc_z_buf) / len(self.acc_z_buf) if len(self.acc_z_buf) > 0 else 0.0
            imu_flight = False # Buffer chưa đủ -> không đánh giá flight
        else:
            avg_acc_z = sum(self.acc_z_buf) / len(self.acc_z_buf)
            imu_flight = (len(self.acc_z_buf) >= self.window) and (avg_acc_z < LIN_ACC_Z_FLIGHT_THRESHOLD)

        # ── Tầng 2: CPG logic guard ─────────────────────────────────────
        # Cả 2 chân đều được lệnh nhấc cao cùng lúc → vi phạm gait pattern
        cpg_flight = (lift_l_cmd > 0.005) and (lift_r_cmd > 0.005)

        # ── Tầng 3: Impact detection ─────────
        # Spike lớn sau phase swing → xác nhận chân đã hạ cánh
        impact_detected = avg_acc_z > LIN_ACC_Z_IMPACT_THRESHOLD

        return {
            'imu_flight'      : imu_flight,
            'cpg_flight'      : cpg_flight,
            'impact_detected' : impact_detected,
            'avg_acc_z'       : avg_acc_z
        }


# =============================================================================
# HISTORY BUFFER
# =============================================================================
class HistoryBuffer:
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
# MẠNG NEURAL
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
# MAIN NODE
# =============================================================================
class PPOWalkingNode(Node):
    def __init__(self):
        super().__init__('ppo_walking_node')

        # --- Paths ---
        self.base_dir = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/weights"
        self.weight_dir = os.path.join(self.base_dir, "balancing_step_ppo")
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

        self.upper_joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in
                           ['base_hip_middle',
                            'hip_shoulder_left', 'shoulder_shoulder_left', 'shoulder_elbow_left',
                            'hip_shoulder_right', 'shoulder_shoulder_right', 'shoulder_elbow_right']}

        # --- Subscribers ---
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, force_qos)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, force_qos)
        self.odom_sub = self.create_subscription(Odometry, '/model/humanoid_robot/odometry', self.odom_callback, force_qos)

        # --- Service clients ---
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # --- IMU & Odom state ---
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.zeros(3)
        self.lin_acc = np.zeros(3)
        self.real_vel_x = 0.0

        self.last_imu_time = 0.0
        self.imu_stale = True

        self.roll_offset = 0.0
        self.pitch_offset = 0.0

        # --- CPG & Contact Estimator ---
        self.cpg = GaitCPG(dt=DT)
        self.contact_est = BNO055ContactEstimator(window=CONTACT_WINDOW)

        # --- History Buffer ---
        self.history_buffer = HistoryBuffer(HISTORY_LEN, FRAME_DIM)

        # --- Action Pipeline ---
        self.smoothed_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.prev_smoothed = np.zeros(ACTION_DIM, dtype=np.float32)
        
        # CPG Scale array (Modified for Step Balance)
        self.scale_array = np.array([
            0.05,    # res_dx_l -> mét
            0.05,    # res_dx_r -> mét
            0.03,    # res_dy   -> mét
            0.0,     # res_dz   TẮT DZ ĐỂ CẤM NHÚN CỔ CHÂN
            0.04,    # res_lift_l -> mét
            0.04,    # res_lift_r -> mét
        ], dtype=np.float32)

        # Trạng thái episode
        self.current_sim_time = 0.0
        self.best_reward = -float('inf')
        self.start_episode = 0
        self.current_episode = 0
        self.step_in_episode = 0
        self.ep_reward = 0.0
        self.episode_reason = "TIME_LIMIT"
        self.current_std = ACTION_STD_INIT
        
        self.cpg_ref = np.zeros(6, dtype=np.float32)

        self.is_pushing = False
        self.push_current_step = 0
        self.push_total_steps = 0
        self.push_max_f = 0.0
        self.push_axis = 'y'
        self.push_dir = 1
        self.next_push_step = -1

        # --- PPO ---
        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()

        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f:
                csv.writer(f).writerow(["Episode", "Reward", "Steps", "Reason", "Time"])

        self.get_logger().info(f">>> Stepping Balance Mode Active | EMA={EMA_ALPHA} | RESIDUAL_SCALE={RESIDUAL_SCALE}")

        self.state_lock = threading.Lock()
        threading.Thread(target=self.train_loop, daemon=True).start()

    def imu_callback(self, msg: Imu):
        with self.state_lock:
            self.quat[0] = msg.orientation.w
            self.quat[1] = msg.orientation.x
            self.quat[2] = msg.orientation.y
            self.quat[3] = msg.orientation.z
            self.gyro[0] = msg.angular_velocity.x
            self.gyro[1] = msg.angular_velocity.y
            self.gyro[2] = msg.angular_velocity.z
            self.lin_acc[0] = msg.linear_acceleration.x
            self.lin_acc[1] = msg.linear_acceleration.y
            self.lin_acc[2] = msg.linear_acceleration.z
            
            self.last_imu_time = time.time()
            self.imu_stale = False

        if hasattr(self, 'upper_joint_pubs'):
            for pub in self.upper_joint_pubs.values():
                pub.publish(Float64(data=0.0))

    def clock_callback(self, msg: Clock):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def odom_callback(self, msg: Odometry):
        # Ground truth vx từ odometry
        with self.state_lock:
            self.real_vel_x = msg.twist.twist.linear.x

    def build_frame(self, raw_action_prev):
        is_timeout = time.time() - self.last_imu_time > IMU_TIMEOUT
        if is_timeout or self.imu_stale:
            if not self.imu_stale:
                self.get_logger().warn(f"⚠️ IMU STALE! Giữ frame cũ.")
                self.imu_stale = True
            return self.history_buffer.get_observation()

        with self.state_lock:
            quat_copy = self.quat.copy()
            gyro_raw = self.gyro.copy()
            lin_acc_raw = self.lin_acc.copy()
            
        gyro_norm = np.clip(gyro_raw, -MAX_GYRO, MAX_GYRO) / MAX_GYRO
        
        # Scaling lin_acc roughly to [-1, 1] assuming max approx 20 m/s^2 
        lin_acc_norm = np.clip(lin_acc_raw, -20.0, 20.0) / 20.0

        sin_phi, cos_phi = self.cpg.get_phase_encoding()

        frame = np.concatenate([
            quat_copy,                  # 4
            gyro_norm,                  # 3
            lin_acc_norm,               # 3
            raw_action_prev,            # 6
            [sin_phi, cos_phi]          # 2
        ])  # = 18

        frame = np.nan_to_num(frame, nan=0.0, posinf=1.0, neginf=-1.0)
        self.history_buffer.push(frame)
        return self.history_buffer.get_observation()

    def train_loop_step(self, raw_ppo_action):
        """
        Tự động điều chỉnh Scale và CPG
        """
        cpg_ref = self.cpg_ref 

        # Full PPO can thiệp tối đa với scale_array cho Stepping Balance
        res = np.clip(raw_ppo_action, -1.0, 1.0) * RESIDUAL_SCALE * self.scale_array
        final_raw = cpg_ref + res

        # Cập nhật hành động mượt (Smoothing)
        self.prev_smoothed = self.smoothed_action.copy()
        
        ema = EMA_ALPHA 
        self.smoothed_action = ema * final_raw + (1 - ema) * self.smoothed_action

        cmd = self._to_joint_angles(self.smoothed_action)
        return cmd, res
        
    def _to_joint_angles(self, action):
        """Mapping to LLC Cartesian array"""
        dx_l, dx_r, dy, dz, lift_l, lift_r = action
        
        tz = STD_Z + dz
        stance = 0.0
        
        tx_l = dx_l
        tx_r = dx_r
        target_y_l = STD_Y + dy + stance
        target_y_r = -STD_Y + dy - stance

        # FIX LỖI ĐẦU GỐI: Phải trừ đi lift để Inverse Kinematics gập đầu gối lại!
        tz_left = max(0.12, min(0.285, tz - lift_l))
        tz_right = max(0.12, min(0.285, tz - lift_r))

        len_l = np.linalg.norm([tx_l, target_y_l, tz_left])
        len_r = np.linalg.norm([tx_r, target_y_r, tz_right])

        if len_l > SAFE_LIMIT or len_r > SAFE_LIMIT:
            return None

        msg = Float64MultiArray()
        msg.data = [tx_l, target_y_l, tz_left, lift_l,
                    tx_r, target_y_r, tz_right, lift_r, MOVE_DURATION]
        return msg

    def compute_reward(self, cpg_ref, res_action, raw_ppo_action, cmd_vel_x, real_vel_x, lin_acc_z, pitch, roll):
        contact_info = self.contact_est.update(
            lin_acc_z   = lin_acc_z,
            lift_l_cmd  = self.smoothed_action[4],
            lift_r_cmd  = self.smoothed_action[5]
        )

        r_alive = 3.0
        r_balance = -5.0 * (pitch**2 + roll**2)
        
        # Hình phạt rung lắc (Jitter) luôn bật để bảo vệ phần cứng
        jerk = float(np.sum((self.smoothed_action - self.prev_smoothed)**2))
        r_jitter = -0.5 * jerk

        # Stepping Balance Reward
        r_step = 0.0
        if abs(pitch) > 0.15 or abs(roll) > 0.15:
            # Khen thưởng lấy lại thăng bằng bằng cách nhấc chân và di chuyển (stepping)
            lift_val = np.sum(np.maximum(0, self.smoothed_action[4:6]))
            dx_dy_val = abs(self.smoothed_action[0]) + abs(self.smoothed_action[1]) + abs(self.smoothed_action[2])
            r_step = 2.0 * (lift_val + dx_dy_val)
        else:
            # Đang ổn định -> Khen thưởng khi thu chân về vị trí nghỉ (0)
            action_sq = float(np.sum(self.smoothed_action**2))
            r_step = -1.0 * action_sq

        # Hình phạt "bay" (cả 2 chân rời đất)
        r_fly = 0.0
        if contact_info['imu_flight'] and contact_info['cpg_flight']:
            r_fly = -3.0
        elif contact_info['imu_flight'] or contact_info['cpg_flight']:
            r_fly = -1.5

        return r_alive + r_balance + r_step + r_fly + r_jitter

    def is_fallen(self):
        with self.state_lock:
            qw, qx, qy, qz = self.quat.copy()
            
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        raw_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2.0 * (qw * qy - qz * qx)
        raw_pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        roll = raw_roll - self.roll_offset
        pitch = raw_pitch - self.pitch_offset
        
        tz = STD_Z + float(self.smoothed_action[3]) # approx
        
        return abs(roll) > FALL_ROLL_THRESHOLD or abs(pitch) > FALL_PITCH_THRESHOLD or tz > 0.288

    def trigger_push_event(self):
        min_f = 300.0
        max_f = min(500.0, 300.0 + (self.current_episode // 40) * 4.0)

        self.push_max_f = np.random.uniform(min_f, max_f)
        self.push_dir = np.random.choice([-1, 1])
        self.push_axis = np.random.choice(['y', 'x', 'xy'], p=[0.7, 0.1, 0.2])
        
        self.is_pushing = True
        self.push_current_step = 0
        self.push_total_steps = max(1, int(2.0 / DT))  # 2.0s duration
        
        self.get_logger().info(f"🖐️ BẮT ĐẦU ĐẨY TỪ TỪ: Max {self.push_max_f:.0f}N theo trục {self.push_axis.upper()} trong 2.0s")

    def process_continuous_push(self):
        if not self.is_pushing:
            return

        progress = self.push_current_step / float(self.push_total_steps)
        sine_multiplier = math.sin(math.pi * progress)
        current_f = self.push_max_f * sine_multiplier * self.push_dir
        
        model_name = "humanoid_robot"
        if self.push_axis == 'y':
            force_str = f"{{y: {current_f:.1f}}}"
        elif self.push_axis == 'x':
            force_str = f"{{x: {current_f:.1f}}}"
        else:
            f_val_x = current_f * 0.707
            f_val_y = current_f * 0.707 * float(np.random.choice([-1, 1]))
            force_str = f"{{x: {f_val_x:.1f}, y: {f_val_y:.1f}}}"

        msg_str = (f"entity: {{name: '{model_name}', type: MODEL}}, wrench: {{force: {force_str}}}")

        cmd_on = ["gz", "topic", "-t", "/world/empty/wrench", "-m", "gz.msgs.EntityWrench", "-p", msg_str]
        subprocess.Popen(cmd_on, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        self.push_current_step += 1

        if self.push_current_step > self.push_total_steps:
            self.is_pushing = False
            msg_zero = (f"entity: {{name: '{model_name}', type: MODEL}}, wrench: {{force: {{x: 0, y: 0, z: 0}}}}")
            cmd_off = ["gz", "topic", "-t", "/world/empty/wrench", "-m", "gz.msgs.EntityWrench", "-p", msg_zero]
            subprocess.Popen(cmd_off, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("🛑 BUÔNG TAY (Kết thúc lực đẩy)")

    def sim_sleep(self, duration):
        start = self.current_sim_time
        while self.current_sim_time - start < duration:
            time.sleep(0.001)

    def reset_simulation(self):
        while not self.w_cli.wait_for_service(timeout_sec=1.0):
            pass

        msg_zero = "entity: {name: 'humanoid_robot', type: MODEL}, wrench: {force: {x: 0, y: 0, z: 0}}"
        cmd_off = ["gz", "topic", "-t", "/world/empty/wrench", "-m", "gz.msgs.EntityWrench", "-p", msg_zero]
        subprocess.Popen(cmd_off, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.is_pushing = False

        req_u = ControlWorld.Request()
        req_u.world_control = WorldControl(pause=False)
        self.w_cli.call_async(req_u)
        time.sleep(0.1)

        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(3):
            self.action_pub.publish(msg)
            time.sleep(0.1)
            
        time.sleep(1.2)

        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.2)

        req_p = ControlWorld.Request()
        req_p.world_control = WorldControl(pause=True)
        self.w_cli.call_async(req_p)
        time.sleep(0.2)

        req_s = SetEntityPose.Request()
        req_s.entity = Entity(name="humanoid_robot")
        req_s.pose = Pose()
        req_s.pose.position.z = 0.28
        req_s.pose.orientation.w = 1.0
        self.p_cli.call_async(req_s)
        time.sleep(0.5)

        req_u = ControlWorld.Request()
        req_u.world_control = WorldControl(pause=False)
        self.w_cli.call_async(req_u)
        
        sc = self.current_sim_time
        while self.current_sim_time - sc < 0.05:
            time.sleep(0.01)

        self.reset_pub.publish(Bool(data=False))

        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.zeros(3)
        self.lin_acc = np.zeros(3)
        self.history_buffer.reset()
        self.smoothed_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.prev_smoothed = np.zeros(ACTION_DIM, dtype=np.float32)
        self.last_imu_time = time.time()
        self.imu_stale = True
        self.step_in_episode = 0
        self.ep_reward = 0.0
        self.episode_reason = "TIME_LIMIT"
        self.cpg.phase = 0.0
        self.contact_est.reset()
        
        self.is_pushing = False
        self.next_push_step = np.random.randint(150, 230)
        
        return self.stabilize_robot()

    def stabilize_robot(self):
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(3):
            self.action_pub.publish(msg)
            time.sleep(0.1)

        deadline_imu = time.time() + 3.0
        while self.imu_stale and time.time() < deadline_imu:
            self.action_pub.publish(msg)
            time.sleep(0.05)

        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self.state_lock:
                gyro_mag = np.linalg.norm(self.gyro)
            if gyro_mag < 0.05:
                break
            self.action_pub.publish(msg)
            time.sleep(0.1)
            
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
        
        if abs(roll_offset) > 0.4 or abs(pitch_offset) > 0.4:
            return False
            
        self.roll_offset = roll_offset
        self.pitch_offset = pitch_offset
            
        self.smoothed_action = np.zeros(ACTION_DIM, dtype=np.float32)
        return True

    def transfer_weights(self, old_model_path, new_model):
        old_state = torch.load(old_model_path, map_location=DEVICE, weights_only=True)
        new_state = new_model.state_dict()
        for name, param in old_state.items():
            if name in new_state:
                if param.shape == new_state[name].shape:
                    new_state[name].copy_(param)
                else:
                    self.get_logger().info(f"Phẫu thuật cắt ghép layer: {name}")
                    # Kế thừa các action cũ, random các action mới bị dư ra
                    if len(param.shape) == 2:
                        min_dim = min(param.shape[0], new_state[name].shape[0])
                        new_state[name][:min_dim, :] = param[:min_dim, :]
                    elif len(param.shape) == 1:
                        min_dim = min(param.shape[0], new_state[name].shape[0])
                        new_state[name][:min_dim] = param[:min_dim]
        new_model.load_state_dict(new_state)

    def load_training_state(self):
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                data = json.load(f)
                self.best_reward = data.get('best_reward', -float('inf'))
                self.start_episode = data.get('current_episode', 0) + 1
                self.current_std = data.get('action_std', ACTION_STD_INIT)

        if os.path.exists(self.latest_model_path):
            self.transfer_weights(self.latest_model_path, self.ppo.policy)
            self.ppo.policy_old.load_state_dict(self.ppo.policy.state_dict())
            self.ppo.policy.set_action_std(self.current_std)
            self.ppo.policy_old.set_action_std(self.current_std)
        self.current_episode = self.start_episode

    def save_state(self, ep, reward, is_best=False):
        data = {"best_reward": float(self.best_reward),
                "current_episode": int(ep),
                "action_std": float(self.current_std)}
        with open(self.meta_path, 'w') as f:
            json.dump(data, f, indent=4)
        torch.save(self.ppo.policy.state_dict(), self.latest_model_path)
        if is_best:
            torch.save(self.ppo.policy.state_dict(), self.best_model_path)

    def log_episode(self, ep, reward, steps, reason):
        with open(self.history_path, 'a', newline='') as f:
            csv.writer(f).writerow([ep, f"{reward:.2f}", steps, reason,
                                    datetime.now().strftime("%H:%M:%S")])

    def train_loop(self):
        while self.current_sim_time == 0.0:
            time.sleep(0.1)
        self.get_logger().info(">>> Sim clock detected. Training start.")

        while self.imu_stale and rclpy.ok():
            time.sleep(0.1)

        while rclpy.ok():
            if not self.reset_simulation():
                time.sleep(0.5)
                continue
            ep = self.current_episode

            prev_raw_action = np.zeros(ACTION_DIM, dtype=np.float32)

            while rclpy.ok():
                self.cpg_ref = self.cpg.step()

                if self.step_in_episode % 20 == 0:
                    self.get_logger().info(
                        f"odom vel_x={self.real_vel_x:.4f} | "
                        f"CPG dx_l={self.cpg_ref[0]:.3f} | "
                        f"Smoothed dx_l={self.smoothed_action[0]:.3f}"
                    )

                state = self.build_frame(prev_raw_action)
                raw_action = self.ppo.select_action(state)

                msg, res = self.train_loop_step(raw_action)
                if msg is None:
                    self.ppo.buffer_rewards.append(-100.0)
                    self.ppo.buffer_is_terminals.append(True)
                    self.episode_reason = "IK_FAIL"
                    break

                self.action_pub.publish(msg)

                self.process_continuous_push()
                if self.next_push_step > 0 and self.step_in_episode >= self.next_push_step and not self.is_pushing:
                    self.trigger_push_event()
                    self.next_push_step = self.step_in_episode + np.random.randint(150, 230)

                self.sim_sleep(DT)

                done = self.is_fallen()
                
                with self.state_lock:
                    lz = self.lin_acc[2]
                    qw, qx, qy, qz = self.quat.copy()
                
                sinr_cosp = 2.0 * (qw * qx + qy * qz)
                cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                raw_roll = math.atan2(sinr_cosp, cosr_cosp)
                sinp = 2.0 * (qw * qy - qz * qx)
                raw_pitch = math.asin(max(-1.0, min(1.0, sinp)))
                roll = raw_roll - self.roll_offset
                pitch = raw_pitch - self.pitch_offset
                
                reward = self.compute_reward(
                    cpg_ref=self.cpg_ref,
                    res_action=res,
                    raw_ppo_action=raw_action,
                    cmd_vel_x=CMD_VEL_X,
                    real_vel_x=self.real_vel_x,
                    lin_acc_z=lz,
                    pitch=pitch,
                    roll=roll
                )

                if done:
                    reward -= 100.0

                # Cập nhật buffer (PPO Step Balance)
                self.ppo.buffer_rewards.append(reward)
                self.ppo.buffer_is_terminals.append(done)
                if len(self.ppo.buffer_states) >= MAX_UPDATE_STEPS:
                    self.ppo.update()

                self.ep_reward += reward
                prev_raw_action = raw_action.copy()
                self.step_in_episode += 1

                if done:
                    self.episode_reason = "FALL"
                    break
                elif self.step_in_episode >= MAX_STEPS:
                    self.episode_reason = "TIME_LIMIT"
                    break

            if self.step_in_episode <= 1:
                n_pop = self.step_in_episode
                for _ in range(n_pop):
                    if self.ppo.buffer_states:
                        self.ppo.buffer_states.pop()
                        self.ppo.buffer_actions.pop()
                        self.ppo.buffer_logprobs.pop()
                        self.ppo.buffer_rewards.pop()
                        self.ppo.buffer_is_terminals.pop()
                self.log_episode(ep, self.ep_reward, self.step_in_episode, "SKIP_RESET")
                self.current_episode += 1
                continue

            is_best = self.ep_reward > self.best_reward
            if is_best:
                self.best_reward = self.ep_reward

            if ep > 0 and ep % DECAY_INTERVAL_EP == 0:
                self.current_std = max(ACTION_STD_MIN, self.current_std * DECAY_FACTOR)
                self.ppo.policy.set_action_std(self.current_std)
                self.ppo.policy_old.set_action_std(self.current_std)

            if len(self.ppo.buffer_states) >= MIN_UPDATE_STEPS:
                self.ppo.update()

            self.save_state(ep, self.ep_reward, is_best)
            self.log_episode(ep, self.ep_reward, self.step_in_episode, self.episode_reason)
            self.get_logger().info(
                f"Ep {ep} | R:{self.ep_reward:.2f} | Best:{self.best_reward:.2f} | "
                f"S:{self.step_in_episode} | STD:{self.current_std:.4f} | "
                f"{self.episode_reason}")
            self.current_episode += 1


def main(args=None):
    rclpy.init(args=args)
    node = PPOWalkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(">>> Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
