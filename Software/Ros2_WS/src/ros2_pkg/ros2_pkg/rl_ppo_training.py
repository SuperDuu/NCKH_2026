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
from torch.optim.lr_scheduler import StepLR

# ==============================================================================
# CONSTANTS
# ==============================================================================
L5 = 57.0
ANKLE_HEIGHT = 18.0
RESET_HEIGHT = 200.0

# ==============================================================================
# PPO - GPU OPTIMIZED VERSION
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
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
        self.action_var.data = torch.full_like(self.action_var, 
                                             new_action_std * new_action_std)

    def act(self, state):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        action_clipped = torch.clamp(action, -1.0, 1.0)
        return action_clipped.detach(), dist.log_prob(action).sum(dim=-1).detach(), self.critic(state).detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        dist = Normal(action_mean, torch.sqrt(self.action_var.expand_as(action_mean)))
        return dist.log_prob(action).sum(dim=-1), self.critic(state), dist.entropy().sum(dim=-1)

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        self.gamma, self.eps_clip, self.K_epochs = gamma, eps_clip, K_epochs
        self.buffer_states, self.buffer_actions, self.buffer_logprobs = [], [], []
        self.buffer_rewards, self.buffer_is_terminals = [], []
        
        self.policy = ActorCritic(state_dim, action_dim).float().to(self.device)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])
        
        self.scheduler = StepLR(self.optimizer, step_size=1, gamma=0.98)
        
        self.policy_old = ActorCritic(state_dim, action_dim).float().to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.MseLoss = nn.MSELoss().to(self.device)

    def select_action(self, state):
        with torch.no_grad():
            state_tensor = torch.as_tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
            action, logprob, _ = self.policy_old.act(state_tensor)
        
        self.buffer_states.append(state_tensor.cpu())
        self.buffer_actions.append(action.cpu())
        self.buffer_logprobs.append(logprob.cpu())
        return action.cpu().numpy().flatten()

    def update(self):
        if len(self.buffer_states) < 10: return
        try:
            rewards = []
            discounted_reward = 0
            for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
                if is_terminal: discounted_reward = 0
                discounted_reward = reward + (self.gamma * discounted_reward)
                rewards.insert(0, discounted_reward)
                
            rewards = torch.tensor(rewards, dtype=torch.float32).to(self.device)
            if len(rewards) > 1:
                rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

            old_states = torch.stack(self.buffer_states, dim=0).to(self.device).detach().squeeze()
            old_actions = torch.stack(self.buffer_actions, dim=0).to(self.device).detach().squeeze()
            old_logprobs = torch.stack(self.buffer_logprobs, dim=0).to(self.device).detach().squeeze()

            for _ in range(self.K_epochs):
                logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
                state_values = state_values.squeeze()
                ratios = torch.exp(logprobs - old_logprobs)
                advantages = rewards - state_values.detach()
                
                surr1 = ratios * advantages
                surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
                loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
                
                self.optimizer.zero_grad()
                loss.mean().backward()
                self.optimizer.step()
                
            self.policy_old.load_state_dict(self.policy.state_dict())
        except Exception as e:
            print(f"⚠️ Update Skip: {e}", flush=True)
        finally:
            self.clear_buffer()

    def clear_buffer(self):
        del self.buffer_states[:], self.buffer_actions[:], self.buffer_logprobs[:]
        del self.buffer_rewards[:], self.buffer_is_terminals[:]

# ==============================================================================
# RL NODE
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        user_home = os.path.expanduser("~")
        self.weights_dir = os.path.join(user_home, "Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/weights")
        os.makedirs(self.weights_dir, exist_ok=True)
        self.model_path = os.path.join(self.weights_dir, "best.pt")
        self.meta_path = os.path.join(self.weights_dir, "training_meta.json")
        self.history_path = os.path.join(self.weights_dir, "training_history.csv")
        
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        
        self.joint_names = [
            'base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 
            'knee_ankle_left_joint', 'ankle_ankle_left_joint',
            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 
            'knee_ankle_right_joint', 'ankle_ankle_right_joint'
        ]
        self.joint_pubs = {name: self.create_publisher(Float64, f'/model/humanoid_robot/joint/{name}/cmd_pos', 10) for name in self.joint_names}

        self.state_dim = 6
        self.action_dim = 10 
        
        self.ppo_agent = PPOAgent(self.state_dim, self.action_dim, 0.0003, 0.001, 0.99, 10, 0.2)
        
        self.current_episode = 0
        self.best_reward = -float('inf')
        self.best_episode = 0
        self.current_std = 0.25
        self.std_decay_interval = 50 
        
        self.current_obs = np.zeros(3)
        self.prev_obs = np.zeros(3)
        self.data_received = False
        self.is_falling = False
        self.prev_action = np.zeros(self.action_dim)
        
        # Mapping 10 tham số: 
        # [gain, scale_base, fwctEnd, landing_phase, stance_width, fhMax, rr, ankle_gain, gyro_gain, roll_step_gain]
        self.param_mins = np.array([0.05, 0.10, 30.0, 3.0, 20.0, 10.0, 0.01, 0.1, 0.05, 50.0])
        self.param_maxs = np.array([0.30, 0.40, 60.0, 10.0, 35.0, 30.0, 0.2, 0.6, 0.40, 250.0])
        
        self.load_training_state()
        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z]) # [pitch, roll, phase]
        self.data_received = True
        if abs(msg.x) > 40.0 or abs(msg.y) > 40.0: self.is_falling = True

    def get_state_vector(self):
        curr = self.current_obs
        prev = self.prev_obs
        delta = curr - prev
        return np.concatenate([curr/np.array([40.0, 40.0, 1.0]), delta/np.array([10.0, 10.0, 0.2])]).astype(np.float32)

    def calculate_reward(self, pitch, roll, t, is_falling, action):
        if is_falling: return -50.0 
        
        tilt_mag = np.sqrt(pitch**2 + roll**2)
        
        r_survival = 1.0 
        r_upright = 5.0 * np.exp(-tilt_mag / 8.0) # Thưởng lớn nếu đứng thẳng
        
        # Phạt nếu hành động thay đổi quá đột ngột (tránh rung lắc)
        r_stability = -0.5 * np.mean(np.square(action - self.prev_action))
        self.prev_action = action.copy()
        
        # Thưởng theo thời gian sống sót
        r_time = t / 1000.0
        
        return r_survival + r_upright + r_stability + r_time

    def train_loop(self):
        time.sleep(2)
        update_interval = 4000
        total_steps = 0
        
        for episode in range(self.current_episode, 200000):
            if not self.running: break
            self.current_episode = episode
            
            # Cập nhật STD
            if episode % self.std_decay_interval == 0 and episode > 0:
                self.current_std = max(0.05, self.current_std - 0.01)
                self.ppo_agent.policy.set_action_std(self.current_std)
                self.ppo_agent.policy_old.set_action_std(self.current_std)

            print(f"\n--- Episode {episode} | STD: {self.current_std:.3f} ---", flush=True)
            
            self.reset_pub.publish(Bool(data=True))
            if not self.reset_simulation_physics(): continue
            
            time.sleep(0.5)
            self.is_falling, self.data_received = False, False
            self.reset_pub.publish(Bool(data=False))
            
            state = self.get_state_vector()
            ep_reward, ep_steps = 0, 0
            
            for t in range(15000): # Max steps per episode
                if not self.running or self.is_falling: break
                
                action = self.ppo_agent.select_action(state)
                # Denormalize action sang dải vật lý
                phys_act = self.param_mins + (np.clip(action, -1, 1) + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
                
                self.param_pub.publish(Float64MultiArray(data=list(phys_act)))
                
                # Chờ feedback từ C++
                start_wait = time.time()
                while not self.data_received and (time.time() - start_wait) < 0.1:
                    time.sleep(0.001)
                
                next_state = self.get_state_vector()
                reward = self.calculate_reward(self.current_obs[0], self.current_obs[1], t, self.is_falling, action)
                done = self.is_falling
                
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                state = next_state
                ep_reward += reward
                ep_steps = t
                total_steps += 1
                
                if total_steps % update_interval == 0:
                    self.ppo_agent.update()
                
                if done: break

            # Lưu model nếu đạt best reward
            if ep_reward > self.best_reward and ep_steps > 50:
                self.best_reward = ep_reward
                self.best_episode = episode
                self.save_training_state(is_best=True)
                print(f"⭐ New Best! Reward: {ep_reward:.2f}", flush=True)
            
            if episode % 10 == 0: self.save_training_state()
            print(f"End Ep {episode} | Steps: {ep_steps} | Reward: {ep_reward:.2f}")

    def run_gz_command(self, service, req_type, req_data):
        cmd = ['gz', 'service', '-s', service, '--reqtype', req_type, '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', req_data]
        try:
            result = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2.0)
            return result.returncode == 0
        except: return False

    # def reset_simulation_physics(self):
    #     self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true')
    #     z_reset = (RESET_HEIGHT + L5 + ANKLE_HEIGHT + 20.0) / 1000.0
    #     pose_msg = f'name: "humanoid_robot" position {{ x: 0.0 y: 0.0 z: {z_reset:.3f} }} orientation {{ x: 0.0 y: 0.0 z: 0.0 w: 1.0 }}'
    #     self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', pose_msg)
        
    #     # Tư thế khuỵu gối nhẹ để IK C++ tiếp quản mượt hơn
    #     safe_pose = {
    #         'hip_knee_left_joint': 0.1, 
    #         'hip_knee_right_joint': 0.1, 
    #         'knee_ankle_left_joint': -0.05, 
    #         'knee_ankle_right_joint': -0.05
    #     }
    #     for name, val in safe_pose.items():
    #         if name in self.joint_pubs: self.joint_pubs[name].publish(Float64(data=val))
            
    #     time.sleep(0.5)
    #     return self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false')
    def reset_simulation_physics(self):
        """Hàm Reset chuẩn cũ, lặp cho tới khi thành công"""
        # 1. Clear IPC (Dọn dẹp bộ nhớ chia sẻ để tránh xung đột Gazebo)
        
        os.system("ipcs -m | awk '{print $2}' | xargs -rn1 ipcrm -m > /dev/null 2>&1")
        time.sleep(2)
        # 2. Pause Simulation
        # Lặp cho tới khi Gazebo phản hồi đã Pause thành công
        while not self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: true'):
            time.sleep(0.5)
        time.sleep(2.0)
        # 3. Set Pose (Đưa robot về vị trí trên không để rơi xuống)
        # z: 0.42 tương ứng với khoảng 420mm
        pose_msg = 'name: "humanoid_robot" position { x: 0.0 y: 0.0 z: 0.30 } orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }'
        while not self.run_gz_command('/world/empty/set_pose', 'gz.msgs.Pose', pose_msg):
            time.sleep(0.5)
        time.sleep(2.0)
        # 4. Set Joints (Gập chân nhẹ để robot không bị khóa khớp khi bắt đầu)
        safe_pose = {
            'hip_knee_left_joint': 0.3, 
            'hip_knee_right_joint': 0.3, 
            'knee_ankle_left_joint': -0.15, 
            'knee_ankle_right_joint': -0.15
        }
        for name, val in safe_pose.items():
            if name in self.joint_pubs:
                self.joint_pubs[name].publish(Float64(data=val))
        time.sleep(1)
        # Kích hoạt tín hiệu Reset sang Node C++
        self.reset_pub.publish(Bool(data=True))
        time.sleep(1)
        self.reset_pub.publish(Bool(data=False))
        time.sleep(1)
        # Khởi tạo lại các biến trạng thái RL
        self.prev_action = np.zeros(self.action_dim)
        # Nếu bạn có dùng obs_history thì clear, nếu không có thể bỏ qua dòng dưới
        if hasattr(self, 'obs_history'):
            self.obs_history.clear()
        
        # 5. Unpause Simulation
        while not self.run_gz_command('/world/empty/control', 'gz.msgs.WorldControl', 'pause: false'):
            time.sleep(0.5)
        time.sleep(0.1)    
        self.is_falling = False
        self.data_received = False
        return True

    def save_training_state(self, is_best=False):
        if is_best: torch.save(self.ppo_agent.policy_old.state_dict(), self.model_path)
        meta = {'best_reward': float(self.best_reward), 'episode': self.current_episode, 'best_episode': self.best_episode}
        with open(self.meta_path, 'w') as f: json.dump(meta, f, indent=2)

    def load_training_state(self):
        if os.path.exists(self.model_path):
            self.ppo_agent.policy.load_state_dict(torch.load(self.model_path, map_location=self.ppo_agent.device), strict=False)
            self.ppo_agent.policy_old.load_state_dict(self.ppo_agent.policy.state_dict())
        if os.path.exists(self.meta_path):
            with open(self.meta_path, 'r') as f:
                meta = json.load(f)
                self.best_reward = meta.get('best_reward', -float('inf'))
                self.current_episode = meta.get('episode', 0)
                self.best_episode = meta.get('best_episode', 0)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()