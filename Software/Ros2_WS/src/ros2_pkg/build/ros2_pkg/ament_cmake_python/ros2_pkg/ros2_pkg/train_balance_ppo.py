#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose

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
# CẤU HÌNH VẬT LÝ & THỜI GIAN
# ==============================================================================
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT = 0.05 
MAX_STEPS = 3000 
MAX_LEG_LENGTH = 0.2204
SAFE_LIMIT = MAX_LEG_LENGTH * 0.98 # Chừa 2% dư địa an toàn tuyệt đối

STD_Z = 0.195 # Hạ xuống để cứu trục Z
STD_Y = 0.01  
LIFT_H = 0.03 # 3cm là mức lý tưởng cho Z=0.195
ACTION_DIM = 4 
STATE_DIM = 6
STEPS_PER_PHASE = 15 

# ==============================================================================
# THUẬT TOÁN PPO (GIỮ NGUYÊN KIẾN TRÚC)
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.3):
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

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        return action.detach(), action_logprob.detach(), self.critic(state).detach()

    def evaluate(self, state, action):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var.expand_as(action_mean)))
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        return action_logprobs, state_values, dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim):
        self.gamma = 0.99
        self.eps_clip = 0.2
        self.K_epochs = 10
        self.buffer_states, self.buffer_actions, self.buffer_logprobs = [], [], []
        self.buffer_rewards, self.buffer_is_terminals = [], []
        self.policy = ActorCritic(state_dim, action_dim).to(DEVICE)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': 0.0003},
            {'params': self.policy.critic.parameters(), 'lr': 0.001}
        ])
        self.policy_old = ActorCritic(state_dim, action_dim).to(DEVICE)
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
        rewards, discounted_reward = [], 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards[:min_size]), reversed(self.buffer_is_terminals[:min_size])):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        rewards = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)
        old_states = torch.stack(self.buffer_states[:min_size]).detach()
        old_actions = torch.stack(self.buffer_actions[:min_size]).detach()
        old_logprobs = torch.stack(self.buffer_logprobs[:min_size]).detach()
        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards) - 0.01 * dist_entropy
            self.optimizer.zero_grad(); loss.mean().backward(); self.optimizer.step()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear(); self.buffer_actions.clear(); self.buffer_logprobs.clear(); self.buffer_rewards.clear(); self.buffer_is_terminals.clear()

# ==============================================================================
# ROS2 TRAINING NODE
# ==============================================================================
class HumanoidTrainNode(Node):
    def __init__(self):
        super().__init__('humanoid_train_node')
        self.weights_dir = "/home/nckh/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights"
        os.makedirs(self.weights_dir, exist_ok=True)
        self.best_model_path = os.path.join(self.weights_dir, "best_model.pth")
        self.latest_model_path = os.path.join(self.weights_dir, "latest_model.pth")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")
        self.history_path = os.path.join(self.weights_dir, "training_history.csv")
        
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10) 
        self.imu_sub = self.create_subscription(Vector3, '/robot_orientation', self.imu_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.w_cli = self.create_client(ControlWorld, '/world/empty/control')
        self.p_cli = self.create_client(SetEntityPose, '/world/empty/set_pose')

        self.joint_pubs = {j: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10) for j in 
                          ['base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
                           'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right']}

        # Khởi tạo các bộ lọc
        self.roll = self.pitch = self.prev_roll = self.prev_pitch = 0.0
        self.smooth_roll = self.smooth_pitch = 0.0
        self.prev_action = np.zeros(ACTION_DIM)
        
        self.is_left_support = True; self.current_sim_time = 0.0
        self.best_reward = -float('inf'); self.start_episode = 0
        self.step_in_episode = 0

        self.ppo = PPO(STATE_DIM, ACTION_DIM)
        self.load_training_state()
        
        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w', newline='') as f:
                csv.writer(f).writerow(["Episode", "Reward", "Steps", "Reason", "Time"])

        threading.Thread(target=self.train_loop, daemon=True).start()

    def imu_callback(self, msg):
        self.prev_roll, self.prev_pitch = self.roll, self.pitch
        self.pitch, self.roll = msg.x * (np.pi/180), msg.y * (np.pi/180)
        # Lọc thông thấp IMU (Alpha = 0.7) để tránh AI bị sốc bởi rung động chạm đất
        self.smooth_roll = 0.3 * self.smooth_roll + 0.7 * self.roll
        self.smooth_pitch = 0.3 * self.smooth_pitch + 0.7 * self.pitch

    def clock_callback(self, msg): self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def call_pause_physics(self, p):
        req = ControlWorld.Request(); req.world_control.pause = p
        self.w_cli.call_async(req)

    def call_set_pose(self, z):
        req = SetEntityPose.Request(); req.entity.name = "humanoid_robot"
        req.pose.position.z = z; req.pose.orientation.w = 1.0 
        self.p_cli.call_async(req)

    def load_training_state(self):
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                data = json.load(f); self.best_reward = data.get('best_reward', -float('inf'))
                self.start_episode = data.get('current_episode', 0) + 1
        if os.path.exists(self.latest_model_path):
            self.ppo.policy.load_state_dict(torch.load(self.latest_model_path, map_location=DEVICE))
            self.ppo.policy_old.load_state_dict(self.ppo.policy.state_dict())

    def save_state(self, ep, reward, is_best=False):
        data = {"best_reward": float(self.best_reward), "current_episode": int(ep)}
        with open(self.meta_path, 'w') as f: json.dump(data, f, indent=4)
        torch.save(self.ppo.policy.state_dict(), self.latest_model_path)
        if is_best:
            torch.save(self.ppo.policy.state_dict(), self.best_model_path)
            print(f"\n\033[1;33m⭐ NEW BEST: {reward:.2f} | Ep: {ep}\033[0m\n", flush=True)

    def get_observation(self):
        d_roll = (self.smooth_roll - self.prev_roll) / DT
        d_pitch = (self.smooth_pitch - self.prev_pitch) / DT
        obs = [self.smooth_roll, self.smooth_pitch, d_roll, d_pitch]
        if not self.is_left_support: obs[0] *= -1; obs[2] *= -1
        phase = (self.current_sim_time % (STEPS_PER_PHASE*DT*2)) * np.pi
        return np.array(obs + [np.sin(phase), np.cos(phase)])

    def check_safety_and_prepare_cmd(self, action):
        # 1. Smoothing Action
        smooth_act = 0.7 * self.prev_action + 0.3 * action
        self.prev_action = smooth_act

        dx_s, dy_s, dz_s, dst_s = smooth_act[0]*0.06, smooth_act[1]*0.02, smooth_act[2]*0.02, smooth_act[3]*0.02
        tz = STD_Z + dz_s

        total_steps = STEPS_PER_PHASE * 2
        progress = (self.step_in_episode % total_steps) / total_steps
        weight_shift = np.sin(progress * 2 * np.pi)
        
        # 2. Logic nhấc chân (Lift) dứt khoát
        lift_l = lift_r = 0.0
        if weight_shift < -0.4:
            norm_l = (-weight_shift - 0.4) / 0.6
            lift_l = LIFT_H * (norm_l ** 0.3) 
        elif weight_shift > 0.4:
            norm_r = (weight_shift - 0.4) / 0.6
            lift_r = LIFT_H * (norm_r ** 0.3)

        # 3. ĐƯA dst_s VÀO LOGIC Y (Quan trọng!)
        # Chân trụ: Chỉ lấn tâm (dy_s)
        # Chân lăng: Đá ra ngoài (mặc định) + xoạc thêm (dst_s)
        
        # Mặc định ban đầu
        target_y_l = 0.01 + dy_s
        target_y_r = -0.01 - dy_s

        if weight_shift > 0: # Chân trái trụ, chân phải lăng
            target_y_l = 0.01 - 0.012 * weight_shift + dy_s # Lấn tâm
            target_y_r = -0.01 - dst_s # Chân phải xoạc ngang theo AI
        elif weight_shift < 0: # Chân phải trụ, chân trái lăng
            target_y_r = -0.01 + 0.012 * abs(weight_shift) - dy_s # Lấn tâm
            target_y_l = 0.01 + dst_s # Chân trái xoạc ngang theo AI
            
        # 4. Logic X (Tiến tới)
        tx_l = dx_s if lift_l > 0.01 else 0.0
        tx_r = dx_s if lift_r > 0.01 else 0.0

        if np.linalg.norm([tx_l, target_y_l, tz]) > SAFE_LIMIT or \
           np.linalg.norm([tx_r, target_y_r, tz]) > SAFE_LIMIT:
            return True, self.get_safe_msg(), -5.0 

        msg = Float64MultiArray()
        msg.data = [tx_l, target_y_l, tz, lift_l, tx_r, target_y_r, tz, lift_r, DT]
        return True, msg, 0.0

    def stabilize_robot(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.01, STD_Z, 0.0, 0.0, -0.01, STD_Z, 0.0, 1.0]
        for _ in range(3): self.action_pub.publish(msg); time.sleep(0.1)
        time.sleep(1.5)

    def reset_simulation(self): 
         self.reset_pub.publish(Bool(data=True)); time.sleep(1.0)  
         self.call_pause_physics(True); time.sleep(0.2); self.call_set_pose(0.3); time.sleep(0.5)  
         safe_pose = { 'hip_hip_left_joint': 0, 'hip_hip_right_joint': 0, 'hip_knee_left_joint': 0, 
                      'hip_knee_right_joint': 0, 'knee_ankle_left_joint': 0, 'knee_ankle_right_joint': 0, 
                      'ankle_ankle_left_joint': 0.0, 'ankle_ankle_right_joint': 0.0, 'base_hip_left_joint': 0.0, 'base_hip_right_joint': 0.0 } 
         for _ in range(15):  
             for name, val in safe_pose.items(): 
                 if name in self.joint_pubs: self.joint_pubs[name].publish(Float64(data=val)) 
         time.sleep(0.5); self.call_pause_physics(False) 
         sc = self.current_sim_time 
         while self.current_sim_time - sc < 0.02: time.sleep(0.01) 
         self.reset_pub.publish(Bool(data=False)); self.roll = 0.0; self.pitch = 0.0; self.is_left_support = True; self.step_in_episode = 0
         self.prev_action = np.zeros(ACTION_DIM); self.stabilize_robot()

    def train_loop(self):
        time.sleep(3); ep = self.start_episode
        while rclpy.ok():
            self.reset_simulation(); ep_reward = 0; failed_at_start = False; reason = "TIME_LIMIT"
            for t in range(MAX_STEPS):
                self.step_in_episode = t
                state = self.get_observation()
                action = self.ppo.select_action(state)
                ok, msg, pen = self.check_safety_and_prepare_cmd(action)
                
                if not ok:
                    self.ppo.buffer_rewards.append(-300.0); self.ppo.buffer_is_terminals.append(True); reason = "IK_FAIL"; break
                
                self.action_pub.publish(msg); time.sleep(DT)
                done = abs(self.smooth_roll) > 0.6 or abs(self.smooth_pitch) > 0.6
                
                # Reward: Cân bằng giữa giữ thẳng và tiết kiệm Action (Energy)
                reward = (1.5 + 3.5*np.exp(-6.0*np.sqrt(self.smooth_pitch**2+self.smooth_roll**2))) + pen
                
                if done: reward = -15.0
                if t == 0 and done: failed_at_start = True; break
                self.ppo.buffer_rewards.append(reward); self.ppo.buffer_is_terminals.append(done); ep_reward += reward
                
                if len(self.ppo.buffer_rewards) >= 2000:
                    print("\033[92m>>> UPDATING POLICY <<<\033[0m"); self.ppo.update()
                
                self.is_left_support = (np.sin((t % (STEPS_PER_PHASE*2)) / (STEPS_PER_PHASE*2) * 2 * np.pi) > 0)
                if done: reason = "FALL"; break
            
            if not failed_at_start:
                is_nb = ep_reward > self.best_reward
                if is_nb: self.best_reward = ep_reward
                self.save_state(ep, ep_reward, is_nb)
                with open(self.history_path, 'a', newline='') as f:
                    csv.writer(f).writerow([ep, f"{ep_reward:.2f}", t, reason, datetime.now().strftime("%H:%M:%S")])
                print(f"Ep {ep} | Reward: {ep_reward:.2f} | Steps: {t} | {reason}")
                ep += 1
            else: print("\033[91m⚠️ Reset Lag Detected. Ignoring episode.\033[0m")

def main(): rclpy.init(); rclpy.spin(HumanoidTrainNode())
if __name__ == '__main__': main()