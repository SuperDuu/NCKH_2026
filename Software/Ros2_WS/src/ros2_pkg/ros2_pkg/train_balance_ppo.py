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
# CẤU HÌNH VẬT LÝ
# ==============================================================================
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT = 0.05 
MAX_STEPS = 1000 
MAX_LEG_LENGTH = 0.220

# [FIX 1] Nới lỏng giới hạn an toàn lên 1.02 để tránh lỗi IK oan uổng
SAFE_LIMIT = MAX_LEG_LENGTH * 1.02 

STD_Z = 0.20   
STD_Y = 0.02    
LIFT_H = 0.06  
ACTION_DIM = 4 
STATE_DIM = 6

# ==============================================================================
# PPO ALGORITHM 
# ==============================================================================
class ActorCritic(nn.Module):
    # [FIX 2] Giảm độ nhiễu khởi tạo (0.5 -> 0.3) để robot bớt giật cục lúc đầu
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

    def set_action_std(self, new_action_std):
        self.action_var = torch.full((self.actor[-2].out_features,), new_action_std * new_action_std).to(DEVICE)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        return action.detach(), action_logprob.detach(), self.critic(state).detach()

    def evaluate(self, state, action):
        action_mean = self.actor(state)
        action_var = self.action_var.expand_as(action_mean)
        dist = Normal(action_mean, torch.sqrt(action_var))
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        return action_logprobs, state_values, dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        self.buffer_states, self.buffer_actions = [], []
        self.buffer_logprobs, self.buffer_rewards, self.buffer_is_terminals = [], [], []
        self.policy = ActorCritic(state_dim, action_dim).to(DEVICE)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])
        self.policy_old = ActorCritic(state_dim, action_dim).to(DEVICE)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss()

    def select_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state).to(DEVICE)
            action, action_logprob, _ = self.policy_old.act(state)
        self.buffer_states.append(state)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(action_logprob)
        return action.cpu().numpy().flatten()

    def update(self):
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        rewards = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        if len(rewards) > 1: rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)
        old_states = torch.squeeze(torch.stack(self.buffer_states, dim=0)).detach().to(DEVICE)
        old_actions = torch.squeeze(torch.stack(self.buffer_actions, dim=0)).detach().to(DEVICE)
        old_logprobs = torch.squeeze(torch.stack(self.buffer_logprobs, dim=0)).detach().to(DEVICE)
        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            state_values = torch.squeeze(state_values)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear(); self.buffer_actions.clear(); self.buffer_logprobs.clear()
        self.buffer_rewards.clear(); self.buffer_is_terminals.clear()
    
    def save(self, path): torch.save(self.policy_old.state_dict(), path)
    def load(self, path): 
        self.policy_old.load_state_dict(torch.load(path, map_location=DEVICE))
        self.policy.load_state_dict(torch.load(path, map_location=DEVICE))

# ==============================================================================
# ROS2 NODE
# ==============================================================================
class HumanoidTrainNode(Node):
    def __init__(self):
        super().__init__('humanoid_train_node')
        
        self.weights_dir = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights"
        os.makedirs(self.weights_dir, exist_ok=True)
        self.best_model_path = os.path.join(self.weights_dir, "best_model.pth")
        self.latest_model_path = os.path.join(self.weights_dir, "latest_model.pth")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")
        self.history_csv_path = os.path.join(self.weights_dir, "training_history.csv")
        
        self.action_pub = self.create_publisher(Float64MultiArray, '/rl/leg_command', 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10) 
        self.imu_sub = self.create_subscription(Vector3, '/robot_orientation', self.imu_callback, 10)
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        self.world_control_client = self.create_client(ControlWorld, '/world/empty/control')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')
        
        print("⏳ Waiting for Gazebo Bridge services...", flush=True)
        while not self.world_control_client.wait_for_service(timeout_sec=2.0):
            print("... waiting for /world/empty/control", flush=True)
        print("✅ Gazebo Services Connected!", flush=True)

        self.joint_pubs = {}
        joints = ['base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
                  'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right']
        for j in joints:
            self.joint_pubs[j] = self.create_publisher(Float64, f'/model/humanoid_robot/joint/{j}_joint/cmd_pos', 10)

        self.roll = 0.0; self.pitch = 0.0
        self.prev_roll = 0.0; self.prev_pitch = 0.0
        self.is_left_support = True
        self.imu_received = False
        self.current_sim_time = 0.0
        
        self.best_reward = -float('inf')
        self.best_episode = 0
        self.start_episode = 0
        
        self.ppo = PPO(STATE_DIM, ACTION_DIM, 0.0003, 0.001, 0.99, 10, 0.2)
        self.load_training_state()
        
        if not os.path.exists(self.history_csv_path):
            with open(self.history_csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["Episode", "Reward", "Steps", "Reason", "Timestamp"])

        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def call_pause_physics(self, paused=True):
        req = ControlWorld.Request()
        req.world_control.pause = paused
        future = self.world_control_client.call_async(req)
        while not future.done():
            time.sleep(0.005)
        return future.result().success

    def call_set_pose(self, z_height=0.3):
        req = SetEntityPose.Request()
        req.entity.name = "humanoid_robot"
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = z_height
        req.pose.orientation.w = 1.0 
        future = self.set_pose_client.call_async(req)
        while not future.done():
            time.sleep(0.005)
        return future.result().success

    def load_training_state(self):
        if os.path.exists(self.meta_path):
            try:
                with open(self.meta_path, 'r') as f:
                    data = json.load(f)
                    self.best_reward = data.get('best_reward', -float('inf'))
                    self.best_episode = data.get('best_episode', 0)
                    self.start_episode = data.get('current_episode', 0) + 1
                    print(f"🔄 RESUMING from Ep {self.start_episode} (Best: {self.best_reward:.2f})", flush=True)
            except Exception as e:
                print(f"⚠️ Error loading meta file: {e}", flush=True)

        if os.path.exists(self.latest_model_path):
            self.ppo.load(self.latest_model_path)
        elif os.path.exists(self.best_model_path):
            self.ppo.load(self.best_model_path)

    def save_training_state(self, current_ep, is_best=False):
        data = {
            "best_reward": float(self.best_reward),
            "best_episode": int(self.best_episode),
            "current_episode": int(current_ep),
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        with open(self.meta_path, 'w') as f:
            json.dump(data, f, indent=4)
        self.ppo.save(self.latest_model_path)
        if is_best: self.ppo.save(self.best_model_path)

    def log_episode_to_csv(self, ep, reward, steps, reason):
        with open(self.history_csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([ep, f"{reward:.2f}", steps, reason, datetime.now().strftime("%H:%M:%S")])

    def clock_callback(self, msg):
        self.current_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def imu_callback(self, msg):
        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.pitch = msg.x * (np.pi / 180.0) 
        self.roll = msg.y * (np.pi / 180.0)
        self.imu_received = True

    def get_observation(self):
        obs_pitch = self.pitch
        obs_roll = self.roll
        obs_d_pitch = (self.pitch - self.prev_pitch) / DT
        obs_d_roll = (self.roll - self.prev_roll) / DT
        if not self.is_left_support:
            obs_roll *= -1       
            obs_d_roll *= -1
        phase_val = (self.current_sim_time % 1.0) * 2 * np.pi 
        return np.array([obs_roll, obs_pitch, obs_d_roll, obs_d_pitch, np.sin(phase_val), np.cos(phase_val)])

    def check_safety_and_prepare_cmd(self, action):
        dx_s = np.clip(action[0], -1, 1) * 0.03
        dy_s = np.clip(action[1], -1, 1) * 0.02
        raw_dz = np.clip(action[2], -1, 1)
        if raw_dz > 0: dz_s = raw_dz * 0.01 
        else:          dz_s = raw_dz * 0.03 
        raw_st = np.clip(action[3], -1, 1)
        if raw_st > 0: dz_st = raw_st * 0.005
        else:          dz_st = raw_st * 0.02

        if self.is_left_support:
            t_xl, t_yl, t_zl, t_hl = 0.0, STD_Y, STD_Z + dz_st, 0.0
            t_xr, t_yr, t_zr, t_hr = dx_s, -STD_Y + dy_s, STD_Z + dz_s, LIFT_H
        else:
            t_xr, t_yr, t_zr, t_hr = 0.0, -STD_Y, STD_Z + dz_st, 0.0
            t_xl, t_yl, t_zl, t_hl = dx_s, STD_Y - dy_s, STD_Z + dz_s, LIFT_H

        dist_L = np.sqrt(t_xl**2 + t_yl**2 + t_zl**2)
        dist_R = np.sqrt(t_xr**2 + t_yr**2 + t_zr**2)
        
        penalty = 0.0
        is_valid = True
        
        if dist_L > SAFE_LIMIT:
            penalty -= 100.0 + (dist_L - SAFE_LIMIT)*1000
            is_valid = False
            
        if dist_R > SAFE_LIMIT:
            penalty -= 100.0 + (dist_R - SAFE_LIMIT)*1000
            is_valid = False
            
        if self.is_left_support:
             if t_yr > -0.005: 
                 penalty -= 50.0
                 is_valid = False
        else:
             if t_yl < 0.005: 
                 penalty -= 50.0
                 is_valid = False

        if not is_valid: return False, None, penalty

        msg = Float64MultiArray()
        msg.data = [t_xl, t_yl, t_zl, t_hl, t_xr, t_yr, t_zr, t_hr, 0.5] 
        return True, msg, 0.0

    def calculate_reward(self, penalty_ik):
        if penalty_ik < -1.0: return penalty_ik 
        r_alive = 1.0
        tilt = np.sqrt(self.pitch**2 + self.roll**2)
        r_upright = np.exp(-5.0 * tilt) 
        return r_alive + 3.0 * r_upright

    def reset_simulation(self):
        # 1. Stop C++
        self.reset_pub.publish(Bool(data=True))
        time.sleep(1.0) 
        
        # 2. PAUSE (API Call)
        self.call_pause_physics(True)
        time.sleep(0.2)

        # 3. TELEPORT (API Call)
        self.call_set_pose(0.3)
        time.sleep(0.5) 
        
        # 4. RESET KHỚP
        safe_pose = {
            'hip_hip_left_joint': 0,   'hip_hip_right_joint': 0,
            'hip_knee_left_joint': 0,  'hip_knee_right_joint': 0,
            'knee_ankle_left_joint': 0, 'knee_ankle_right_joint': 0,
            'ankle_ankle_left_joint': 0.0, 'ankle_ankle_right_joint': 0.0,
            'base_hip_left_joint': 0.0, 'base_hip_right_joint': 0.0
        }
        for _ in range(15): 
            for name, val in safe_pose.items():
                if f"{name}" in self.joint_pubs:
                   self.joint_pubs[name].publish(Float64(data=val))
        
        time.sleep(0.5)

        # 5. UNPAUSE (API Call)
        self.call_pause_physics(False)
        
        start_check = self.current_sim_time
        while self.current_sim_time - start_check < 0.05:
            time.sleep(0.01)
        
        # 6. START
        self.reset_pub.publish(Bool(data=False))
        self.roll = 0.0; self.pitch = 0.0; self.is_left_support = True
        self.stabilize_robot()

    def stabilize_robot(self):
        msg = Float64MultiArray()
        msg.data = [0.0, STD_Y, STD_Z, 0.0, 0.0, -STD_Y, STD_Z, 0.0, 1.0]
        for _ in range(3): 
            self.action_pub.publish(msg)
            time.sleep(0.1)
        time.sleep(1.5)

    def train_loop(self):
        print("--- TRAINING STARTING ---", flush=True)
        time.sleep(3)
        
        ep = self.start_episode
        
        while ep < 100000:
            self.reset_simulation()
            ep_reward = 0
            state = self.get_observation()
            reset_reason = "TIME_LIMIT"
            episode_failed_at_start = False 
            
            for t in range(MAX_STEPS):
                action = self.ppo.select_action(state)
                is_valid, cmd_msg, penalty = self.check_safety_and_prepare_cmd(action)
                
                if not is_valid:
                    heavy_penalty = -300.0 
                    self.ppo.buffer_rewards.append(heavy_penalty)
                    self.ppo.buffer_is_terminals.append(True)
                    reset_reason = "IK_FAIL"
                    time.sleep(0.5)
                    break 
                
                self.action_pub.publish(cmd_msg)
                time.sleep(DT)
                
                reward = self.calculate_reward(penalty)
                state = self.get_observation()
                
                done = False
                if abs(self.pitch) > 0.6 or abs(self.roll) > 0.6: 
                    done = True
                    reward = -10.0
                    deg_pitch = self.pitch * 180.0 / np.pi
                    deg_roll = self.roll * 180.0 / np.pi
                    reset_reason = f"FALL (Pitch:{deg_pitch:.1f}°, Roll:{deg_roll:.1f}°)"
                
                if t == 0 and done:
                    print(f"⚠️ RESET POSE ERROR: Fall at Step 0 ({reset_reason}). Retrying...", flush=True)
                    self.ppo.buffer_states.pop()
                    self.ppo.buffer_actions.pop()
                    self.ppo.buffer_logprobs.pop()
                    episode_failed_at_start = True
                    break

                self.ppo.buffer_rewards.append(reward)
                self.ppo.buffer_is_terminals.append(done)
                ep_reward += reward
                
                if len(self.ppo.buffer_rewards) >= 2000:
                    print(f">>> UPDATING POLICY (Ep {ep}) <<<", flush=True)
                    self.ppo.update()
                    self.save_training_state(ep, is_best=False)
                
                if t % 10 == 0: self.is_left_support = not self.is_left_support
                
                if done: break
            
            if episode_failed_at_start:
                continue
            
            print(f"Ep {ep} | Reward: {ep_reward:.2f} | Steps: {t} | Reason: {reset_reason}", flush=True)
            self.log_episode_to_csv(ep, ep_reward, t, reset_reason)
            
            is_new_best = False
            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                self.best_episode = ep
                is_new_best = True
                print(f"⭐ NEW BEST! Reward: {ep_reward:.2f} (Saved best_model.pth)", flush=True)
            
            if is_new_best or ep % 10 == 0:
                self.save_training_state(ep, is_best=is_new_best)
            
            ep += 1

def main():
    rclpy.init()
    node = HumanoidTrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()