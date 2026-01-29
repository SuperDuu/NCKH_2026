import os
import torch
import torch.nn as nn
from torch.distributions import Normal
import numpy as np

# ==============================================================================
# 1. MẠNG PPO (Actor - Critic) - GIỮ NGUYÊN CẤU TRÚC VÀ THÊM FIX DEVICE
# ==============================================================================
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, action_std_init=0.6):
        super(ActorCritic, self).__init__()
        self.action_dim = action_dim
        # Khởi tạo action_var
        self.action_var = torch.full((action_dim,), action_std_init * action_std_init)
        
        # ACTOR
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(),
            nn.Linear(256, action_dim), nn.Tanh()
        )
        
        # CRITIC
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(),
            nn.Linear(256, 1)
        )

    def set_action_std(self, new_action_std):
        """Thiết lập độ rung và đẩy lên đúng device (GPU/CPU)"""
        device = next(self.parameters()).device
        self.action_var = torch.full((self.action_dim,), new_action_std * new_action_std).to(device)

    def act(self, state):
        action_mean = self.actor(state)
        
        # Chặn NaN bám sát file gốc của bạn
        if torch.isnan(action_mean).any():
            action_mean = torch.nan_to_num(action_mean, nan=0.0)
            
        device = state.device
        if self.action_var.device != device:
            self.action_var = self.action_var.to(device)

        dist = Normal(action_mean, torch.sqrt(self.action_var))
        action = dist.sample()
        action_logprob = dist.log_prob(action).sum(dim=-1)
        state_val = self.critic(state)

        return action.detach(), action_logprob.detach(), state_val.detach()
    
    def evaluate(self, state, action):
        action_mean = self.actor(state)
        device = state.device
        
        action_var = self.action_var.expand_as(action_mean).to(device)
        dist = Normal(action_mean, torch.sqrt(action_var))
        
        action_logprobs = dist.log_prob(action).sum(dim=-1)
        dist_entropy = dist.entropy().sum(dim=-1)
        state_values = self.critic(state)
        
        return action_logprobs, state_values, dist_entropy

# ==============================================================================
# 2. PPO AGENT - TÍCH HỢP HÀM DECAY VÀ CUDA
# ==============================================================================
class PPOAgent:
    def __init__(self, state_dim, action_dim, lr_actor, lr_critic, gamma, K_epochs, eps_clip):
        # Tự động nhận diện GPU T1200
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"✅ PPOAgent is using: {self.device}", flush=True)
        
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        self.action_std = 0.6 
        
        self.buffer_states = []
        self.buffer_actions = []
        self.buffer_logprobs = []
        self.buffer_rewards = []
        self.buffer_is_terminals = []
        
        self.policy = ActorCritic(state_dim, action_dim).to(self.device)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])
        
        self.policy_old = ActorCritic(state_dim, action_dim).to(self.device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        # Khởi tạo action_std ban đầu trên đúng device
        self.policy.set_action_std(self.action_std)
        self.policy_old.set_action_std(self.action_std)
        
        self.MseLoss = nn.MSELoss()

    def set_action_std(self, new_action_std):
        """Hàm thiết lập độ rung thủ công từ file của bạn"""
        self.action_std = new_action_std
        self.policy.set_action_std(new_action_std)
        self.policy_old.set_action_std(new_action_std)
        print(f"🔧 Manually set Action Std to: {self.action_std}", flush=True)

    def decay_action_std(self, action_std_decay_rate, min_action_std):
        """Hàm giảm dần độ rung từ file của bạn"""
        self.action_std = self.action_std - action_std_decay_rate
        self.action_std = round(self.action_std, 4)
        
        if (self.action_std <= min_action_std):
            self.action_std = min_action_std
            
        print(f"📉 Action Std decayed to: {self.action_std}", flush=True)
        self.policy.set_action_std(self.action_std)
        self.policy_old.set_action_std(self.action_std)

    def select_action(self, state):
        with torch.no_grad():
            state = torch.FloatTensor(state).to(self.device)
            action, action_logprob, _ = self.policy_old.act(state)
            
        self.buffer_states.append(state)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(action_logprob)
        return action.detach().cpu().numpy().flatten()

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
            state_values = torch.squeeze(state_values)
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            
            # Loss function
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            # Gradient Clipping từ file gốc của bạn
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 0.5) 
            self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())
        del self.buffer_states[:], self.buffer_actions[:], self.buffer_logprobs[:], self.buffer_rewards[:], self.buffer_is_terminals[:]

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)
    
    def load(self, checkpoint_path):
        self.policy_old.load_state_dict(torch.load(checkpoint_path, map_location=self.device))
        self.policy.load_state_dict(torch.load(checkpoint_path, map_location=self.device))