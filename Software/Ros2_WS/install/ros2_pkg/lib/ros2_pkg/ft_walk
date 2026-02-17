#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3, Pose, Wrench 
from nav_msgs.msg import Odometry
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
from datetime import datetime

# ==============================================================================
# CẤU HÌNH HỆ THỐNG & VẬT LÝ - GIỮ NGUYÊN
# ==============================================================================
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT = 0.05 
MAX_STEPS = 9999 
MAX_LEG_LENGTH = 0.2204
SAFE_LIMIT = MAX_LEG_LENGTH * 0.98
STD_Z, STD_Y, LIFT_H = 0.195, 0.01, 0.05 
ACTION_DIM, STATE_DIM, STEPS_PER_PHASE = 4, 7, 15 

# --- THÔNG SỐ STD DECAY ---
ACTION_STD_INIT = 0.05      
ACTION_STD_MIN = 0.01       
DECAY_FACTOR = 0.99         
DECAY_INTERVAL_EP = 30      

# ==============================================================================
# MẠNG NEURAL (PPO) - ĐÃ FIX LỖI ATTRIBUTEERROR
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
        # FIX: Trỏ trực tiếp vào action_dim thay vì lấy từ lớp Tanh cuối cùng
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
        self.buffer_states, self.buffer_actions, self.buffer_logprobs, self.buffer_rewards, self.buffer_is_terminals = [], [], [], [], []
        self.policy = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.optimizer = torch.optim.Adam([{'params': self.policy.actor.parameters(), 'lr': 3e-5}, {'params': self.policy.critic.parameters(), 'lr': 1e-4}])
        self.policy_old = ActorCritic(state_dim, action_dim, ACTION_STD_INIT).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()
    def select_action(self, state):
        with torch.no_grad():
            state_t = torch.FloatTensor(state).to(DEVICE)
            action, logprob, _ = self.policy_old.act(state_t)
        self.buffer_states.append(state_t); self.buffer_actions.append(action); self.buffer_logprobs.append(logprob)
        return action.cpu().numpy().flatten()
    def update(self):
        min_size = min(len(self.buffer_states), len(self.buffer_rewards))
        if min_size == 0: return 
        
        # LOG IN RA KHI UPDATE [Yêu cầu bổ sung]
        print(f"\n\033[1;32m[PPO UPDATE] >>> Đang tối ưu hóa Policy với {min_size} bước dữ liệu...\033[0m")
        
        rewards, discounted_reward = [], 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards[:min_size]), reversed(self.buffer_is_terminals[:min_size])):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        rew_t = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        rewards = (rew_t - rew_t.mean()) / (rew_t.std() + 1e-7)
        old_states, old_actions, old_logprobs = torch.stack(self.buffer_states[:min_size]).detach(), torch.stack(self.buffer_actions[:min_size]).detach(), torch.stack(self.buffer_logprobs[:min_size]).detach()
        for i in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.squeeze().detach()
            surr1, surr2 = ratios * advantages, torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards) - 0.01 * dist_entropy
            self.optimizer.zero_grad(); loss.mean().backward(); self.optimizer.step()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear(); self.buffer_actions.clear(); self.buffer_logprobs.clear(); self.buffer_rewards.clear(); self.buffer_is_terminals.clear()
        print("\033[1;32m[PPO UPDATE] >>> Thành công. Hệ thống sẵn sàng cho chu kỳ mới.\033[0m\n")

# ==============================================================================
# MAIN NODE - WALKING FINE-TUNING
# ==============================================================================
class HumanoidWalkNode(Node):
    def __init__(self):
        super().__init__('humanoid_walk_node')
        
        self.base_dir = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights"
        self.walk_dir = os.path.join(self.base_dir, "finetunning_walk")
        os.makedirs(self.walk_dir, exist_ok=True)
        self.latest_model_path = os.path.join(self.walk_dir, "latest_model.pth")
        self.best_model_path = os.path.join(self.walk_dir, "best_model.pth")
        self.meta_path = os.path.join(self.walk_dir, "training_meta_walk.json")
        self.history_path = os.path.join(self.walk_dir, "training_history_walk.csv")

        force_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.force_pub = self.create_publisher(Wrench, '/model/humanoid_robot/link/base_footprint/wrench', force_qos)
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10) 
        self.imu_sub = self.create_subscription(Vector3, '/robot_orientation', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/humanoid_robot/odometry', self.odom_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')
        self.joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in 
                          ['base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
                           'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right']}

        self.roll = self.pitch = self.yaw = self.prev_roll = self.prev_pitch = 0.0
        self.vx = self.vy = 0.0
        self.smooth_roll = self.smooth_pitch = 0.0
        self.prev_action = np.zeros(ACTION_DIM)
        self.is_left_support = True; self.current_sim_time = 0.0
        self.best_reward = -float('inf'); self.start_episode = 0; self.step_in_episode = 0
        
        self.target_v_x = 0.4 
        self.current_std = ACTION_STD_INIT
        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()
        
        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f: 
                csv.writer(f).writerow(["Episode", "Reward", "Steps", "Vx_Avg", "Time"])

        print(f"\033[94m>>> Node Ready. WALKING Fine-tuning. Target: {self.target_v_x} m/s\033[0m")
        threading.Thread(target=self.train_loop, daemon=True).start()

    def odom_callback(self, msg):
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y

    def load_training_state(self):
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                data = json.load(f); self.best_reward = data.get('best_reward', -float('inf'))
                self.start_episode = data.get('current_episode', 0) + 1
        if os.path.exists(self.latest_model_path):
            self.ppo.policy.load_state_dict(torch.load(self.latest_model_path, map_location=DEVICE))
            self.ppo.policy_old.load_state_dict(self.ppo.policy.state_dict())
            self.current_std = torch.sqrt(self.ppo.policy.action_var[0]).item()

    def save_state(self, ep, reward, is_best=False):
        data = {"best_reward": float(self.best_reward), "current_episode": int(ep)}
        with open(self.meta_path, 'w') as f: json.dump(data, f, indent=4)
        torch.save(self.ppo.policy.state_dict(), self.latest_model_path)
        if is_best: 
            torch.save(self.ppo.policy.state_dict(), self.best_model_path)
            print(f"\033[1;33m⭐ NEW BEST WALK: {reward:.2f} | Episode: {ep}\033[0m")

    def imu_callback(self, msg):
        self.prev_roll, self.prev_pitch = self.roll, self.pitch
        self.pitch, self.roll, self.yaw = msg.x * (np.pi/180), msg.y * (np.pi/180), msg.z * (np.pi/180)
        self.smooth_roll = 0.3 * self.smooth_roll + 0.7 * self.roll
        self.smooth_pitch = 0.3 * self.smooth_pitch + 0.7 * self.pitch

    def clock_callback(self, msg): self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def get_observation(self):
        d_roll, d_pitch = (self.smooth_roll - self.prev_roll) / DT, (self.smooth_pitch - self.prev_pitch) / DT
        obs = [self.smooth_roll, self.smooth_pitch, d_roll, d_pitch, self.yaw]
        if not self.is_left_support: obs[0] *= -1; obs[2] *= -1
        phase = (self.current_sim_time % (STEPS_PER_PHASE*DT*2)) * np.pi
        return np.array(obs + [np.sin(phase), np.cos(phase)])

    def check_safety_and_prepare_cmd(self, action):
        smooth_act = 0.7 * self.prev_action + 0.3 * action
        self.prev_action = smooth_act
        dx_s, dy_s, dz_s, dst_s = smooth_act[0]*0.10, smooth_act[1]*0.02, smooth_act[2]*0.02, smooth_act[3]*0.02
        tz = STD_Z + dz_s
        progress = (self.step_in_episode % (STEPS_PER_PHASE*2)) / (STEPS_PER_PHASE*2)
        weight_shift = np.sin(progress * 2 * np.pi)
        lift_l = LIFT_H * (((-weight_shift - 0.4) / 0.6) ** 0.3) if weight_shift < -0.4 else 0.0
        lift_r = LIFT_H * (((weight_shift - 0.4) / 0.6) ** 0.3) if weight_shift > 0.4 else 0.0
        target_y_l, target_y_r = 0.01 + dy_s, -0.01 - dy_s
        if weight_shift > 0: target_y_l, target_y_r = 0.01 - 0.012 * weight_shift + dy_s, -0.01 - dst_s 
        elif weight_shift < 0: target_y_r, target_y_l = -0.01 + 0.012 * abs(weight_shift) - dy_s, 0.01 + dst_s 
        tx_l, tx_r = (dx_s if lift_l > 0.01 else 0.0), (dx_s if lift_r > 0.01 else 0.0)
        if np.linalg.norm([tx_l, target_y_l, tz]) > SAFE_LIMIT or np.linalg.norm([tx_r, target_y_r, tz]) > SAFE_LIMIT: return False, None, -5.0 
        msg = Float64MultiArray(); msg.data = [tx_l, target_y_l, tz, lift_l, tx_r, target_y_r, tz, lift_r, DT]
        return True, msg, 0.0

    def stabilize_robot(self):
        msg = Float64MultiArray(); msg.data = [0.0, 0.01, STD_Z, 0.0, 0.0, -0.01, STD_Z, 0.0, 1.0]
        for _ in range(3): self.action_pub.publish(msg); time.sleep(0.1)
        time.sleep(1.5)

    def reset_simulation(self): 
        while not self.w_cli.wait_for_service(timeout_sec=1.0): pass
        self.reset_pub.publish(Bool(data=True)); time.sleep(1.0)  
        req_p = ControlWorld.Request(); req_p.world_control = WorldControl(pause=True)
        self.w_cli.call_async(req_p); time.sleep(0.2)
        req_s = SetEntityPose.Request(); req_s.entity = Entity(name="humanoid_robot")
        req_s.pose = Pose(); req_s.pose.position.z = 0.3; req_s.pose.orientation.w = 1.0
        self.p_cli.call_async(req_s); time.sleep(0.5)  
        for pub in self.joint_pubs.values(): pub.publish(Float64(data=0.0))
        time.sleep(0.5)
        req_u = ControlWorld.Request(); req_u.world_control = WorldControl(pause=False)
        self.w_cli.call_async(req_u) 
        sc = self.current_sim_time 
        while self.current_sim_time - sc < 0.05: time.sleep(0.01) 
        self.reset_pub.publish(Bool(data=False))
        self.roll = self.pitch = self.yaw = 0.0
        self.vx = self.vy = 0.0
        self.is_left_support = True; self.step_in_episode = 0
        self.prev_action = np.zeros(ACTION_DIM); self.stabilize_robot()

    def train_loop(self):
        while self.current_sim_time == 0: time.sleep(1.0)
        ep = self.start_episode
        while rclpy.ok():
            self.reset_simulation()
            ep_reward = 0; failed_at_start = False; ep_vx = []
            for t in range(MAX_STEPS):
                self.step_in_episode = t
                state = self.get_observation()
                action = self.ppo.select_action(state)
                ok, msg, pen = self.check_safety_and_prepare_cmd(action)
                if not ok: 
                    self.ppo.buffer_rewards.append(-100.0); self.ppo.buffer_is_terminals.append(True); break
                self.action_pub.publish(msg); time.sleep(DT)
                
                # --- REWARD ĐI BỘ CHỐNG HỌC VẸT ---
                done = abs(self.smooth_roll) > 0.6 or abs(self.smooth_pitch) > 0.6
                
                if self.vx > 0.05:
                    r_alive = 2.0
                    r_balance = 5.0 * np.exp(-10.0 * np.sqrt(self.smooth_pitch**2 + self.smooth_roll**2))
                    r_vel = 15.0 * np.exp(-5.0 * abs(self.vx - self.target_v_x))
                else:
                    r_alive = 0.0 
                    r_balance = 2.0 * np.exp(-10.0 * np.sqrt(self.smooth_pitch**2 + self.smooth_roll**2))
                    r_vel = -5.0 
                
                r_yaw = -2.0 * max(0.0, abs(self.yaw) - 0.1) 
                r_action = -0.3 * np.sum(np.square(action))
                reward = r_alive + r_balance + r_vel + r_yaw + r_action + pen

                if done: reward = -100.0 
                if t == 0 and done: failed_at_start = True; break
                self.ppo.buffer_rewards.append(reward); self.ppo.buffer_is_terminals.append(done); ep_reward += reward
                ep_vx.append(self.vx)

                # Update mỗi 4000 steps [Yêu cầu bổ sung]
                if len(self.ppo.buffer_states) >= 4000: self.ppo.update()
                
                self.is_left_support = (np.sin((t % (STEPS_PER_PHASE*2)) / (STEPS_PER_PHASE*2) * 2 * np.pi) > 0)
                if done: break

            if not failed_at_start:
                if ep > 0 and ep % DECAY_INTERVAL_EP == 0:
                    self.current_std = max(ACTION_STD_MIN, self.current_std * DECAY_FACTOR)
                    self.ppo.policy.set_action_std(self.current_std)
                    self.ppo.policy_old.set_action_std(self.current_std)

                avg_vx = np.mean(ep_vx) if ep_vx else 0.0
                if ep_reward > self.best_reward: self.best_reward = ep_reward; self.save_state(ep, ep_reward, True)
                else: self.save_state(ep, ep_reward, False)

                with open(self.history_path, 'a', newline='') as f:
                    csv.writer(f).writerow([ep, f"{ep_reward:.2f}", t, f"{avg_vx:.3f}", datetime.now().strftime("%H:%M:%S")])
                
                # LOG CHI TIẾT KÈM BEST REWARD [Yêu cầu bổ sung]
                print(f"WALK-Ep {ep} | R: {ep_reward:.2f} | Best: {self.best_reward:.2f} | Steps: {t} | Vx_Avg: {avg_vx:.3f} | STD: {self.current_std:.4f}")
                ep += 1
            else: print("\033[91m⚠️ Reset Lag Detected. Skipping.\033[0m")

def main(): rclpy.init(); rclpy.spin(HumanoidWalkNode())
if __name__ == '__main__': main()