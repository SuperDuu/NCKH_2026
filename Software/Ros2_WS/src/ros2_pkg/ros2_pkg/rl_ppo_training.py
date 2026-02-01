#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
import json

# ==============================================================================
# CONSTANTS
# ==============================================================================
L5 = 57.0
ANKLE_HEIGHT = 18.0
RESET_HEIGHT = 205.0

# ==============================================================================
# PPO - FIXED VERSION
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
        # FIX 1: Đúng cách register buffer
        self.register_buffer('action_var', 
                           torch.full((action_dim,), action_std_init * action_std_init))
        
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
        """FIX 2: Cập nhật đúng cách cho buffer"""
        # Sử dụng .data để cập nhật buffer mà không phá vỡ state_dict
        self.action_var.data = torch.full_like(self.action_var, 
                                             new_action_std * new_action_std)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        # FIX 3: Clip action về [-1, 1] trước khi return
        action_clipped = torch.clamp(action, -1.0, 1.0)
        return action_clipped.detach(), dist.log_prob(action).sum(dim=-1).detach(), self.critic(state).detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var.expand_as(action_mean)))
        return dist.log_prob(action).sum(dim=-1), self.critic(state), dist.entropy().sum(dim=-1)

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        self.device = torch.device("cpu")
        print(f"✅ Device: {self.device}", flush=True)

        self.gamma, self.eps_clip, self.K_epochs = gamma, eps_clip, K_epochs
        self.buffer_states, self.buffer_actions, self.buffer_logprobs = [], [], []
        self.buffer_rewards, self.buffer_is_terminals = [], []
        
        self.policy = ActorCritic(state_dim, action_dim).float().to(self.device)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])
        self.policy_old = ActorCritic(state_dim, action_dim).float().to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss().to(self.device)

    def select_action(self, state):
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).to(self.device).unsqueeze(0)
            action, logprob, _ = self.policy_old.act(state_tensor)
        
        self.buffer_states.append(state_tensor.cpu())
        self.buffer_actions.append(action.cpu())
        self.buffer_logprobs.append(logprob.cpu())
        return action.cpu().numpy().flatten()

    def update(self):
        if len(self.buffer_states) < 4:
            return
            
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal: 
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
        if len(rewards) > 1:
            rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        old_states = torch.cat(self.buffer_states, dim=0).to(self.device).detach()
        old_actions = torch.cat(self.buffer_actions, dim=0).to(self.device).detach()
        old_logprobs = torch.cat(self.buffer_logprobs, dim=0).to(self.device).detach()

        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs.squeeze())
            advantages = rewards - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards) - 0.01 * dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5)
            self.optimizer.step()
            
        # FIX: Đảm bảo copy cả buffers (action_var)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        del self.buffer_states[:], self.buffer_actions[:], self.buffer_logprobs[:]
        del self.buffer_rewards[:], self.buffer_is_terminals[:]
        
        print(f"✅ Updated with {len(old_states)} samples", flush=True)

# ==============================================================================
# RL NODE - FIXED VERSION
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # Setup directories
        user_home = os.path.expanduser("~")
        self.weights_dir = os.path.join(user_home, "Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights")
        os.makedirs(self.weights_dir, exist_ok=True)
        self.model_path = os.path.join(self.weights_dir, "best.pt")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")
        self.history_path = os.path.join(self.weights_dir, "training_history.csv")
        
        if not os.path.exists(self.history_path):
            with open(self.history_path, 'w') as f:
                f.write("episode,reward,steps,std\n")
        
        # ROS communications
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        
        self.joint_names = [
            'base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 
            'knee_ankle_left_joint', 'ankle_ankle_left_joint',
            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 
            'knee_ankle_right_joint', 'ankle_ankle_right_joint'
        ]
        self.joint_pubs = {
            name: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{name}/cmd_pos', 10) 
            for name in self.joint_names
        }

        # RL Parameters
        self.state_dim = 6
        self.action_dim = 7
        self.best_ep = 0
        
        self.ppo_agent = PPOAgent(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            lr_actor=0.0001,
            lr_critic=0.0005,
            gamma=0.99,
            K_epochs=8,
            eps_clip=0.2
        )
        
        # Training State
        self.current_episode = 0
        self.best_reward = -float('inf')
        self.best_episode = 0
        self.current_std = 0.25
        
        # IMU Calibration
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        self.calibration_done = False
        
        # Current State
        self.current_obs = np.zeros(3)
        self.prev_obs = np.zeros(3)
        self.data_received = False
        self.is_falling = False
        self.ep_reward = 0
        self.t_final = 0
        
        # Parameter bounds với CLIPPING
        self.param_mins = np.array([0.10, 0.08, 40.0, 6.0, 20.0, 8.0, 0.06])
        self.param_maxs = np.array([0.22, 0.18, 80.0, 12.0, 35.0, 18.0, 0.14])
        
        # STD control: 10 ep giảm 0.001
        self.std_reduction_counter = 0
        self.std_reduction_interval = 10  # 10 episodes mới giảm 1 lần
        self.std_reduction_amount = 0.001
        
        # Load previous state
        self.load_training_state()
        
        # Start training thread
        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()
        
        print(f"\n{'='*70}", flush=True)
        print(f"🤖 RL TRAINING - FIXED VERSION", flush=True)
        print(f"🎯 STD Schedule: -{self.std_reduction_amount} every {self.std_reduction_interval} episodes", flush=True)
        print(f"✅ Fixes applied: Buffer update, Action clipping, Stable STD decay", flush=True)
        print(f"{'='*70}\n", flush=True)

    # ==============================================================================
    # IMU CALIBRATION
    # ==============================================================================
    def calibrate_imu_offset(self):
        """Quick calibration"""
        print("🔧 Calibrating IMU...", flush=True)
        
        samples = []
        for _ in range(30):
            if self.data_received:
                samples.append(self.current_obs[:2])
            time.sleep(0.02)
        
        if len(samples) > 10:
            samples = np.array(samples)
            self.pitch_offset = np.median(samples[:, 0])
            self.roll_offset = np.median(samples[:, 1])
            self.calibration_done = True
            
            print(f"✅ Calibrated: P={self.pitch_offset:.1f}°, R={self.roll_offset:.1f}°", flush=True)
            return True
        return False
    
    def get_calibrated_obs(self):
        if self.calibration_done:
            return np.array([
                self.current_obs[0] - self.pitch_offset,
                self.current_obs[1] - self.roll_offset,
                self.current_obs[2]
            ])
        return self.current_obs.copy()

    # ==============================================================================
    # STD CONTROL: 10 EPISODES GIẢM 0.001
    # ==============================================================================
    def update_std_slow(self, episode):
        """10 episodes giảm 0.001 - giữ nguyên strategy của bạn"""
        old_std = self.current_std
        
        # Tăng counter
        self.std_reduction_counter += 1
        
        # Mỗi 10 episodes giảm 0.001
        if self.std_reduction_counter >= self.std_reduction_interval:
            self.current_std = max(0.08, self.current_std - self.std_reduction_amount)
            self.std_reduction_counter = 0
            
            print(f"📉 STD reduction: {old_std:.4f} → {self.current_std:.4f}", flush=True)
            
            # Cập nhật cho cả policy và policy_old
            self.ppo_agent.policy.set_action_std(self.current_std)
            self.ppo_agent.policy_old.set_action_std(self.current_std)

    # ==============================================================================
    # STATE & REWARD
    # ==============================================================================
    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True
        
        if abs(msg.x) > 50.0 or abs(msg.y) > 50.0:
            self.is_falling = True

    def get_state_vector(self):
        current_calibrated = self.get_calibrated_obs()
        prev_calibrated = np.array([
            self.prev_obs[0] - self.pitch_offset if self.calibration_done else self.prev_obs[0],
            self.prev_obs[1] - self.roll_offset if self.calibration_done else self.prev_obs[1],
            self.prev_obs[2]
        ])
        
        delta = current_calibrated - prev_calibrated
        norm_obs = current_calibrated / np.array([45.0, 45.0, 1.0])
        norm_delta = delta / np.array([10.0, 10.0, 0.2])
        
        return np.concatenate([norm_obs, norm_delta], axis=0).astype(np.float32)
    
    def calculate_reward(self, pitch, roll, step_count, is_falling):
        if is_falling:
            return -1.0
        
        tilt_mag = np.sqrt(pitch**2 + roll**2)
        
        r_survival = 0.3
        r_tilt = -0.05 * (tilt_mag / 45.0)
        r_time = 0.15 * np.log(step_count + 1)
        r_upright = 0.8 * np.exp(-tilt_mag / 20.0)
        
        total = r_survival + r_tilt + r_time + r_upright
        
        if step_count < 20:
            total += 0.2 * np.exp(-step_count / 10.0)
        
        if step_count % 50 == 0 and step_count > 0:
            total += 0.5
        
        return np.clip(total, -2.0, 10.0)

    # ==============================================================================
    # TRAINING LOOP - FIXED
    # ==============================================================================
    def train_loop(self):
        time.sleep(3)
        
        update_interval = 2000
        time_step = 0
        warmup_params = [0.18, 0.15, 50.0, 8.0, 25.0, 10.0, 0.1]
        
        print("🚀 Starting training with FIXED PPO...", flush=True)
        
        for episode in range(self.current_episode, 100000):
            if not self.running:
                break
                
            self.current_episode = episode
            
            print(f"\n{'='*50}", flush=True)
            print(f"EPISODE {episode}", flush=True)
            print(f"{'='*50}", flush=True)
            
            # Reset
            self.reset_pub.publish(Bool(data=True))
            self.reset_simulation_physics()
            time.sleep(0.8)
            
            self.is_falling = False
            self.data_received = False
            self.reset_pub.publish(Bool(data=False))
            time.sleep(0.5)
            
            # Calibration
            if episode % 10 == 0:
                self.calibrate_imu_offset()
            
            # Warm-up
            print("🔥 Warm-up...", flush=True)
            for _ in range(3):
                self.param_pub.publish(Float64MultiArray(data=warmup_params))
                time.sleep(0.1)
            
            # Training
            state = self.get_state_vector()
            ep_reward = 0
            ep_steps = 0
            
            for t in range(20000):  # Tăng max steps
                if not self.running or self.is_falling:
                    break
                
                while not self.data_received and not self.is_falling:
                    time.sleep(0.001)
                
                if self.is_falling:
                    break
                    
                self.data_received = False
                
                # Select action (đã được clip trong PPO)
                action = self.ppo_agent.select_action(state)
                
                # FIX: Double clip để đảm bảo action trong [-1, 1]
                action = np.clip(action, -1.0, 1.0)
                
                # Convert to parameters với clipping
                phys_act = self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                phys_act = np.clip(phys_act, self.param_mins, self.param_maxs)
                
                self.param_pub.publish(Float64MultiArray(data=list(phys_act)))
                
                # Next state
                next_state = self.get_state_vector()
                calibrated = self.get_calibrated_obs()
                pitch, roll = calibrated[0], calibrated[1]
                
                # Check termination
                done = False
                if self.is_falling:
                    done = True
                elif np.sqrt(pitch**2 + roll**2) > 40.0:
                    done = True
                elif t >= 19999:
                    done = True
                
                # Calculate reward
                reward = self.calculate_reward(pitch, roll, t, self.is_falling)
                
                # Store
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                # Update
                state = next_state
                ep_reward += reward
                ep_steps = t + 1
                time_step += 1
                
                # Periodic update
                if time_step % update_interval == 0 and len(self.ppo_agent.buffer_states) >= 8:
                    self.ppo_agent.update()
                
                if done:
                    break
            
            # End-of-episode update
            if len(self.ppo_agent.buffer_states) >= 4:
                self.ppo_agent.update()
            
            # ============ STD CONTROL: 10 EPISODES GIẢM 0.001 ============
            self.update_std_slow(episode)
            
            # Save results
            if ep_reward > self.best_reward:
                self.best_ep = episode
                self.best_reward = ep_reward
                self.best_episode = episode
                self.save_training_state(ep_reward, ep_steps)
                print(f"🌟 NEW BEST: {ep_reward:.1f} ({ep_steps} steps)", flush=True)
            else:
                with open(self.history_path, 'a') as f:
                    f.write(f"{episode},{ep_reward:.2f},{ep_steps},{self.current_std:.4f}\n")
            
            # Episode summary
            print(f"{'-'*50}", flush=True)
            print(f"SUMMARY | Reward: {ep_reward:.2f} | Steps: {ep_steps}", flush=True)
            print(f"STD: {self.current_std:.4f} | Best: {self.best_reward:.1f} (Ep {self.best_ep})", flush=True)
            print(f"STD counter: {self.std_reduction_counter}/{self.std_reduction_interval}", flush=True)
            if self.calibration_done:
                print(f"Calibration: P={self.pitch_offset:.1f}°, R={self.roll_offset:.1f}°", flush=True)
            print(f"{'='*50}\n", flush=True)

    # ==============================================================================
    # UTILITY FUNCTIONS
    # ==============================================================================
    def reset_simulation_physics(self):
        self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true')
        time.sleep(0.2)
        
        safe_pose = {
            'hip_knee_left_joint': 0.25,
            'knee_ankle_left_joint': -0.12,
            'hip_knee_right_joint': 0.25,
            'knee_ankle_right_joint': -0.12,
        }
        
        for name, val in safe_pose.items():
            if name in self.joint_pubs:
                self.joint_pubs[name].publish(Float64(data=val))
        
        time.sleep(0.2)
        
        z_reset = (RESET_HEIGHT + L5 + ANKLE_HEIGHT) / 1000.0
        pose_msg = (
            f'name: "humanoid_robot" '
            f'position {{ x: 0.0 y: 0.0 z: {z_reset:.3f} }} '
            f'orientation {{ x: 0.0 y: 0.0 z: 0.0 w: 1.0 }}'
        )
        self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', pose_msg)
        time.sleep(0.2)
        
        self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false')
        time.sleep(0.1)
    
    def run_gz_command(self, service, req_type, req_data):
        cmd = [
            'gz', 'service', 
            '-s', service, 
            '--reqtype', req_type, 
            '--reptype', 'gz.msgs.Boolean', 
            '--timeout', '1000', 
            '--req', req_data
        ]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    
    def save_training_state(self, ep_reward=None, steps=None):
        torch.save(self.ppo_agent.policy_old.state_dict(), self.model_path)
        
        meta = {
            'best_reward': float(self.best_reward),
            'current_std': float(self.current_std),
            'episode': self.current_episode,
            'best_episode': self.best_episode,
            'pitch_offset': float(self.pitch_offset),
            'roll_offset': float(self.roll_offset),
            'calibration_done': self.calibration_done,
            'std_reduction_counter': self.std_reduction_counter
        }
        
        with open(self.meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        
        if ep_reward is not None and steps is not None:
            with open(self.history_path, 'a') as f:
                f.write(f"{self.current_episode},{ep_reward:.2f},{steps},{self.current_std:.4f}\n")
        
        print(f"💾 Saved training state", flush=True)
    
    def load_training_state(self):
        if os.path.exists(self.model_path):
            self.ppo_agent.policy.load_state_dict(
                torch.load(self.model_path, map_location='cpu'), 
                strict=False
            )
            self.ppo_agent.policy_old.load_state_dict(self.ppo_agent.policy.state_dict())
            print("✅ Loaded model weights", flush=True)
        
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                meta = json.load(f)
                self.best_reward = meta.get('best_reward', -float('inf'))
                self.current_episode = meta.get('episode', 0)
                self.best_episode = meta.get('best_episode', 0)
                self.current_std = meta.get('current_std', 0.25)
                self.pitch_offset = meta.get('pitch_offset', 0.0)
                self.roll_offset = meta.get('roll_offset', 0.0)
                self.calibration_done = meta.get('calibration_done', False)
                self.std_reduction_counter = meta.get('std_reduction_counter', 0)
                
                print(f"📊 Loaded: Ep {self.current_episode}, Best: {self.best_reward:.1f}", flush=True)
                if self.calibration_done:
                    print(f"   Calibration: P={self.pitch_offset:.1f}°, R={self.roll_offset:.1f}°", flush=True)
        
        self.ppo_agent.policy.set_action_std(self.current_std)
        self.ppo_agent.policy_old.set_action_std(self.current_std)
        print(f"🎯 Initial STD: {self.current_std}", flush=True)

def main():
    rclpy.init()
    node = RLTrainingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Training stopped")
    finally:
        node.running = False
        node.save_training_state()
        rclpy.shutdown()

if __name__ == '__main__':
    main()