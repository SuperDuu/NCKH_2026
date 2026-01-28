#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import threading
import time
import os

# ==============================================================================
# 1. MẠNG PPO (Actor - Critic)
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
        self.action_var = torch.full((action_dim,), action_std_init * action_std_init)
        
        # --- ACTOR: Đưa ra hành động ---
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, action_dim),
            nn.Tanh() # Output [-1, 1]
        )
        
        # --- CRITIC: Đánh giá trạng thái ---
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.Tanh(),
            nn.Linear(256, 256),
            nn.Tanh(),
            nn.Linear(256, 1)
        )

    def set_action_std(self, new_action_std):
        self.action_var = torch.full((self.action_dim,), new_action_std * new_action_std)

    def act(self, state):
        action_mean = self.actor(state)
        cov_mat = torch.diag(self.action_var).unsqueeze(dim=0)
        dist = Normal(action_mean, torch.sqrt(self.action_var))
        
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        state_val = self.critic(state)

        return action.detach(), action_logprob.detach(), state_val.detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        action_var = self.action_var.expand_as(action_mean)
        dist = Normal(action_mean, torch.sqrt(action_var))
        
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        
        return action_logprobs, state_values, dist_entropy

# ==============================================================================
# 2. PPO AGENT (Bộ não học tập)
# ==============================================================================
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logprobs = []
        self.buffer_rewards = []
        self.buffer_is_terminals = []
        
        self.policy = ActorCritic(state_dim, action_dim).float()
        self.optimizer = torch.optim.Adam([
                        {'params': self.policy.actor.parameters(), 'lr': lr_actor},
                        {'params': self.policy.critic.parameters(), 'lr': lr_critic}
                    ])
        
        self.policy_old = ActorCritic(state_dim, action_dim).float()
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()

    def select_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state)
            action, action_logprob, _ = self.policy_old.act(state)
            
        self.buffer_states.append(state)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(action_logprob)
        
        return action.detach().cpu().numpy().flatten()

    def update(self):
        # Tính toán phần thưởng (Monte Carlo)
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
            
        rewards = torch.tensor(rewards, dtype=torch.float32)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        old_states = torch.squeeze(torch.stack(self.buffer_states, dim=0)).detach()
        old_actions = torch.squeeze(torch.stack(self.buffer_actions, dim=0)).detach()
        old_logprobs = torch.squeeze(torch.stack(self.buffer_logprobs, dim=0)).detach()

        # Update K lần
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
        
        del self.buffer_states[:]
        del self.buffer_actions[:]
        del self.buffer_logprobs[:]
        del self.buffer_rewards[:]
        del self.buffer_is_terminals[:]

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)

# ==============================================================================
# 3. ROS NODE (REALTIME)
# ==============================================================================
class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        workspace_dir = os.getcwd() 
        
        # Tạo đường dẫn tuyệt đối đến thư mục weights
        self.weights_dir = os.path.join(workspace_dir, "src/ros2_pkg/weights")
        # --- ROS Communication ---
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        
        # --- Service Clients ---
        self.world_control_client = self.create_client(ControlWorld, '/world/empty/control')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/empty/set_pose')

        # --- Variables ---
        self.current_obs = np.array([0.0, 0.0, 0.0]) 
        self.prev_obs = np.array([0.0, 0.0, 0.0])
        self.data_received = False
        self.is_falling = False
        self.reset_count = 0 

        # --- PPO SETTINGS ---
        # Input 6: [Pitch, Roll, Phase, dPitch, dRoll, dPhase] (Thêm vận tốc để robot biết sắp ngã)
        self.state_dim = 6 
        self.action_dim = 7
        
        self.ppo_agent = PPOAgent(
            state_dim=self.state_dim,
            action_dim=self.action_dim,
            lr_actor=0.0003,
            lr_critic=0.001,
            gamma=0.99,
            K_epochs=10,
            eps_clip=0.2
        )
        
        # Giới hạn tham số (Action Space)
        self.param_mins = np.array([0.05, 0.05, 15.0, 3.0, 20.0, 5.0, 0.01])
        self.param_maxs = np.array([0.50, 0.40, 50.0, 15.0, 60.0, 25.0, 0.40])
        
        self.update_timestep = 2000 # Update mỗi 2000 bước (khoảng 20s realtime)
        self.time_step = 0
        self.best_reward = -float('inf')

        # Thread
        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.prev_obs = self.current_obs.copy()
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True 
        if abs(msg.x) > 35.0 or abs(msg.y) > 35.0:
            self.is_falling = True

    def get_state_vector(self):
        # Tính delta (vận tốc)
        delta = self.current_obs - self.prev_obs
        # Normalize
        norm_obs = self.current_obs / np.array([45.0, 45.0, 1.0])
        norm_delta = delta / np.array([5.0, 5.0, 0.1]) 
        return np.concatenate([norm_obs, norm_delta], axis=0).astype(np.float32)

    def scale_action(self, action):
        return self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)

    def call_service_async(self, client, request):
        if not rclpy.ok(): return
        return client.call_async(request)

    def reset_simulation(self):
        self.reset_count += 1
        
        # 1. Reset UVC (chân thẳng)
        if self.running: self.reset_pub.publish(Bool(data=True))
        
        # 2. Reset Pose & Pause
        req_pause = ControlWorld.Request()
        req_pause.world_control.pause = True
        self.call_service_async(self.world_control_client, req_pause)
        
        req_pose = SetEntityPose.Request()
        req_pose.entity.name = "humanoid_robot"
        req_pose.entity.type = 2
        req_pose.pose = Pose(position=Point(x=0.0, y=0.0, z=0.30), orientation=Quaternion(w=1.0))
        self.call_service_async(self.set_pose_client, req_pose)
        
        # 3. Unpause
        time.sleep(0.05)
        req_unpause = ControlWorld.Request()
        req_unpause.world_control.pause = False
        self.call_service_async(self.world_control_client, req_unpause)
        
        # 4. CHỜ ỔN ĐỊNH (Realtime cần chờ lâu hơn chút)
        time.sleep(1.0) 
        
        # 5. Bắt đầu
        if self.running: self.reset_pub.publish(Bool(data=False))
        self.is_falling = False
        self.data_received = False 
        self.prev_obs = np.zeros(3)
        self.current_obs = np.zeros(3)

    def train_loop(self):
        time.sleep(2)
        print("🚀 PPO REALTIME TRAINING START", flush=True)
        
        for episode in range(100000):
            if not self.running or not rclpy.ok(): break
            
            try:
                self.reset_simulation()
            except: continue

            wait_start = time.time()
            while not self.data_received and self.running:
                if time.time() - wait_start > 2.0: break
                time.sleep(0.01)
            
            state = self.get_state_vector()
            ep_reward = 0
            
            for t in range(2000): 
                if not self.running: break
                
                # 1. PPO Action
                action = self.ppo_agent.select_action(state)
                phys_act = self.scale_action(action)
                
                # 2. Gửi lệnh
                msg = Float64MultiArray()
                msg.data = list(phys_act)
                self.param_pub.publish(msg)
                
                # 3. Chờ Realtime (10ms khớp với Timer C++)
                time.sleep(0.01) 
                
                # 4. Reward
                next_state = self.get_state_vector()
                pitch = self.current_obs[0]
                roll = self.current_obs[1]
                tilt_mag = np.sqrt(pitch**2 + roll**2)
                
                reward = (1.0 - tilt_mag/35.0) + 0.1
                done = False
                if self.is_falling or tilt_mag > 35.0:
                    reward = -10.0
                    done = True
                
                # 5. Lưu vào Buffer
                self.ppo_agent.buffer_rewards.append(reward)
                self.ppo_agent.buffer_is_terminals.append(done)
                
                state = next_state
                ep_reward += reward
                self.time_step += 1
                
                # 6. Update PPO
                if self.time_step % self.update_timestep == 0:
                    print(f"🔄 Updating PPO...", flush=True)
                    self.ppo_agent.update()
                    # self.ppo_agent.save("/tmp/ppo_checkpoint.pt")
                    checkpoint_path = os.path.join(self.weights_dir, "ppo_checkpoint.pt")
                    self.ppo_agent.save(checkpoint_path)

                if done: break
            
            print(f"Ep {episode} | Reward: {ep_reward:.2f} | Steps: {t}", flush=True)
            
            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                # self.ppo_agent.save("/tmp/best_policy.pt") # Nhớ copy file này
                best_model_path = os.path.join(self.weights_dir, "best.pt")
                self.ppo_agent.save(best_model_path)
                print(f"🔥 NEW BEST! Reward: {ep_reward:.2f}", flush=True)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()