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
# 1. MẠNG PPO (Actor - Critic)
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
        # Sử dụng register_buffer để action_var tự động chuyển đổi giữa CPU/GPU theo model
        self.register_buffer('action_var', torch.full((action_dim,), action_std_init * action_std_init))
        
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
        device = self.action_var.device
        self.action_var = torch.full((self.action_dim,), new_action_std * new_action_std).to(device)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        return action.detach(), dist.log_prob(action).sum(dim=-1).detach(), self.critic(state).detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var.expand_as(action_mean)))
        return dist.log_prob(action).sum(dim=-1), self.critic(state), dist.entropy().sum(dim=-1)

# ==============================================================================
# 2. PPO AGENT
# ==============================================================================
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
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
            state = torch.FloatTensor(state).to(self.device)
            action, logprob, _ = self.policy_old.act(state)
        self.buffer_states.append(state.cpu())
        self.buffer_actions.append(action.cpu())
        self.buffer_logprobs.append(logprob.cpu())
        return action.cpu().numpy().flatten()

    def update(self):
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        old_states = torch.stack(self.buffer_states, dim=0).to(self.device).detach()
        old_actions = torch.stack(self.buffer_actions, dim=0).to(self.device).detach()
        old_logprobs = torch.stack(self.buffer_logprobs, dim=0).to(self.device).detach()

        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            ratios = torch.exp(logprobs - old_logprobs.squeeze())
            advantages = rewards - state_values.squeeze().detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values.squeeze(), rewards) - 0.01 * dist_entropy
            
            self.optimizer.zero_grad(); loss.mean().backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5); self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())
        del self.buffer_states[:], self.buffer_actions[:], self.buffer_logprobs[:], self.buffer_rewards[:], self.buffer_is_terminals[:]

# ==============================================================================
# 3. NODE HUẤN LUYỆN ROS2
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # --- PATH SETUP (Theo yêu cầu của bạn) ---
        self.weights_dir = "/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights"
        os.makedirs(self.weights_dir, exist_ok=True)
        self.model_path = os.path.join(self.weights_dir, "best.pt")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")

        # --- ROS COMM ---
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        
        # Joint Publishers để Reset robot
        self.joint_names = ['base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 'knee_ankle_left_joint', 'ankle_ankle_left_joint',
                            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 'knee_ankle_right_joint', 'ankle_ankle_right_joint']
        self.joint_pubs = {name: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{name}/cmd_pos', 10) for name in self.joint_names}

        # --- HYPERPARAMS & STATE ---
        self.state_dim, self.action_dim = 6, 7
        self.ppo_agent = PPOAgent(self.state_dim, self.action_dim, 0.0001, 0.0005, 0.99, 10, 0.2)
        
        self.current_episode = 0
        self.best_reward = -float('inf')
        self.best_episode = 0
        self.current_std = 0.3
        self.load_training_state()

        self.current_obs = np.zeros(3)
        self.prev_obs = np.zeros(3)
        self.data_received = False
        self.is_falling = False
        self.param_mins = np.array([0.01, 0.01, 40.0, 4.0, 10.0, 1.0, 0.001])
        self.param_maxs = np.array([0.50, 0.50, 100.0, 10.0, 40.0, 20.0, 0.25])

        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    # --- HÀM GAZEBO SERVICE ---
    def run_gz_command(self, service, req_type, req_data):
        cmd = ['gz', 'service', '-s', service, '--reqtype', req_type, '--reptype', 'gz.msgs.Boolean', '--timeout', '1000', '--req', req_data]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def reset_simulation_physics(self):
        # 1. Pause
        self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true')
        # 2. Reset Pose
        pose_msg = 'name: "humanoid_robot" position { x: 0.0 y: 0.0 z: 0.28 } orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }'
        self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', pose_msg)
        # 3. Reset Joints
        msg = Float64(data=0.0)
        for pub in self.joint_pubs.values(): pub.publish(msg)
        time.sleep(0.1)
        # 4. Unpause
        self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false')

    def load_training_state(self):
        if os.path.exists(self.model_path):
            self.ppo_agent.policy.load_state_dict(torch.load(self.model_path, map_location=self.ppo_agent.device), strict=False)
            self.ppo_agent.policy_old.load_state_dict(self.ppo_agent.policy.state_dict())
            self.get_logger().info("✅ Loaded weights.")
        
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                meta = json.load(f)
                self.best_reward = meta.get('best_reward', -float('inf'))
                self.current_std = meta.get('current_std', 0.6)
                self.current_episode = meta.get('episode', 0)
                self.best_episode = meta.get('best_episode', 0)
            self.ppo_agent.policy.set_action_std(self.current_std)
            self.ppo_agent.policy_old.set_action_std(self.current_std)

    def save_training_state(self):
        torch.save(self.ppo_agent.policy_old.state_dict(), self.model_path)
        meta = {
            'best_reward': self.best_reward,
            'current_std': self.current_std,
            'episode': self.current_episode,
            'best_episode': self.best_episode
        }
        with open(self.meta_path, 'w') as f: json.dump(meta, f)

    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True 
        if abs(msg.x) > 45.0 or abs(msg.y) > 45.0: self.is_falling = True

    def get_state_vector(self):
        delta = self.current_obs - self.prev_obs
        norm_obs = self.current_obs / np.array([45.0, 45.0, 1.0])
        norm_delta = delta / np.array([5.0, 5.0, 0.1]) 
        return np.concatenate([norm_obs, norm_delta], axis=0).astype(np.float32)

    def train_loop(self):
        time.sleep(2)
        update_timestep = 2000
        time_step = 0

        for episode in range(self.current_episode, 100000):
            if not self.running: break
            self.current_episode = episode
            
            # --- RESET LOGIC ---
            self.reset_pub.publish(Bool(data=True))
            self.reset_simulation_physics()
            time.sleep(0.5)
            self.is_falling, self.data_received = False, False
            self.reset_pub.publish(Bool(data=False))
            
            state = self.get_state_vector()
            ep_reward = 0
            
            # Episodes Loop (Interaction)
            for t in range(6000):
                if not self.running or self.is_falling: break
                while not self.data_received: time.sleep(0.001)
                self.data_received = False

                action = self.ppo_agent.select_action(state)
                phys_act = self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                
                self.param_pub.publish(Float64MultiArray(data=list(phys_act)))

                next_state = self.get_state_vector()
                pitch, roll = self.current_obs[0], self.current_obs[1]
                tilt_mag = np.sqrt(pitch**2 + roll**2)
                
                reward = np.exp(-(tilt_mag/15.0)**2) + 0.1
                done = self.is_falling or tilt_mag > 40.0
                if done: reward = -10.0
                
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                state, ep_reward, time_step = next_state, ep_reward + reward, time_step + 1
                if time_step % update_timestep == 0: 
                    print(f"🔄 Updating Model...", flush=True)
                    self.ppo_agent.update()
                if done: break
            
            # --- POST-EPISODE LOGIC ---
            if episode > 0 and episode % 50 == 0:
                self.current_std = max(0.05, self.current_std - 0.001)
                self.ppo_agent.policy.set_action_std(self.current_std)
                self.ppo_agent.policy_old.set_action_std(self.current_std)

            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                self.best_episode = episode
                self.save_training_state()
                print(f"🌟 NEW BEST: {ep_reward:.1f}", flush=True)

            print(f"Ep {episode} | Reward: {ep_reward:.1f} | Std: {self.current_std:.3f} | Best: {self.best_reward:.1f} (Ep {self.best_episode})", flush=True)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        rclpy.shutdown()

if __name__ == '__main__': main()