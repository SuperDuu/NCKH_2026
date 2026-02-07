#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import mujoco
import mujoco.viewer
import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal
import time
import os
import sys
import json
import csv
from datetime import datetime
from scipy.spatial.transform import Rotation as R

# ==============================================================================
# CẤU HÌNH HỆ THỐNG & ĐƯỜNG DẪN TỰ ĐỘNG
# ==============================================================================
# Lấy đường dẫn tuyệt đối của file script này
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Dò ngược ra thư mục gốc
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
# Trỏ vào thư mục assets
XML_PATH = os.path.join(ROOT_DIR, "assets", "robot_humanoid.xml")
WEIGHTS_DIR = os.path.join(ROOT_DIR, "weights_mujoco")

if not os.path.exists(XML_PATH):
    print(f"❌ LỖI NGHIÊM TRỌNG: Không tìm thấy file XML tại: {XML_PATH}")
    sys.exit(1)

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DT_AGENT = 0.05        
PHYSICS_DT = 0.001     
N_SUBSTEPS = int(DT_AGENT / PHYSICS_DT) 

MAX_STEPS = 3000
L3, L4, L5 = 0.06, 0.1034, 0.057 
MAX_LEG_LENGTH = L3 + L4 + L5 - 0.002
SAFE_LIMIT = MAX_LEG_LENGTH * 0.98

STD_Z = 0.195 
LIFT_H = 0.03
ACTION_DIM = 4 
STATE_DIM = 6
STEPS_PER_PHASE = 15

# ==============================================================================
# 1. PPO ALGORITHM
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

    # def set_action_std(self, new_action_std):
    #     action_var = torch.full((self.actor[-1].out_features,), new_action_std * new_action_std).to(DEVICE)
    #     self.action_var = action_var
    
    def set_action_std(self, new_action_std):
        action_var = torch.full((self.action_var.shape[0],),
                                new_action_std * new_action_std).to(DEVICE)
        self.action_var = action_var

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
        self.buffer_states.append(state_t)
        self.buffer_actions.append(action)
        self.buffer_logprobs.append(logprob)
        return action.cpu().numpy().flatten()

    def update(self):
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.buffer_rewards), reversed(self.buffer_is_terminals)):
            if is_terminal: discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        rewards = torch.tensor(rewards, dtype=torch.float32).to(DEVICE)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-7)

        old_states = torch.stack(self.buffer_states).detach()
        old_actions = torch.stack(self.buffer_actions).detach()
        old_logprobs = torch.stack(self.buffer_logprobs).detach()

        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            state_values = state_values.squeeze()
            ratios = torch.exp(logprobs - old_logprobs)
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            loss = -torch.min(surr1, surr2) + 0.5 * self.MseLoss(state_values, rewards) - 0.01 * dist_entropy
            self.optimizer.zero_grad(); loss.mean().backward(); self.optimizer.step()

        self.policy_old.load_state_dict(self.policy.state_dict())
        self.buffer_states.clear(); self.buffer_actions.clear()
        self.buffer_logprobs.clear(); self.buffer_rewards.clear(); self.buffer_is_terminals.clear()

# ==============================================================================
# 2. MÔI TRƯỜNG MUJOCO
# ==============================================================================
class HumanoidEnv:
    def __init__(self, xml_path, render_mode=False):
        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            print(f"❌ Lỗi khi nạp XML: {e}")
            sys.exit(1)

        self.render_mode = render_mode
        self.viewer = None
        
        self.joint_names = [
            'base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint', 'knee_ankle_left_joint', 'ankle_ankle_left_joint',
            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint', 'knee_ankle_right_joint', 'ankle_ankle_right_joint',
            'base_hip_middle_joint', 'hip_shoulder_left_joint', 'shoulder_shoulder_left_joint', 'shoulder_elbow_left_joint',
            'hip_shoulder_right_joint', 'shoulder_shoulder_right_joint', 'shoulder_elbow_right_joint'
        ]
        
        self.actuator_ids = []
        for name in self.joint_names:
            act_name = f"act_{name[:-6]}"
            id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            self.actuator_ids.append(id)
        
        # --- SỬA LỖI Ở ĐÂY: KHỞI TẠO ĐẦY ĐỦ CÁC BIẾN ---
        self.prev_action = np.zeros(ACTION_DIM)
        self.smooth_roll = 0.0; self.smooth_pitch = 0.0
        self.prev_roll = 0.0; self.prev_pitch = 0.0
        self.roll = 0.0; self.pitch = 0.0  # <--- DÒNG NÀY ĐÃ ĐƯỢC THÊM VÀO ĐỂ KHÔNG BỊ CRASH
        self.current_time = 0.0
        self.is_left_support = True
        
        self.left_leg_state = {'x': 0, 'y': 0.01, 'z': STD_Z}
        self.right_leg_state = {'x': 0, 'y': -0.01, 'z': STD_Z}

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[2] = 0.48 
        mujoco.mj_step(self.model, self.data)
        
        self.prev_action = np.zeros(ACTION_DIM)
        self.smooth_roll = 0.0; self.smooth_pitch = 0.0
        self.prev_roll = 0.0; self.prev_pitch = 0.0
        self.roll = 0.0; self.pitch = 0.0
        self.current_time = 0.0; self.is_left_support = True
        
        for _ in range(50): self.step_physics(np.zeros(10)) 
        return self.get_obs()

    def solve_ik(self, dx, dy, dz, is_right):
        d_target = np.sqrt(dx**2 + dy**2 + dz**2)
        if d_target > MAX_LEG_LENGTH:
            scale = MAX_LEG_LENGTH / d_target
            dx *= scale; dy *= scale; dz *= scale

        hn = np.arctan2(dy, dz)
        d_yz = np.sqrt(dz**2 + dy**2)
        c3_raw = ((d_yz - L5)**2 + dx**2 - L3**2 - L4**2) / (2.0 * L3 * L4)
        c3 = np.clip(c3_raw, -1.0, 1.0)
        s3 = np.sqrt(1.0 - c3**2)
        dg = np.arctan2(s3, c3)
        ht = np.arctan2(s3 * L4, L3 + c3 * L4) + np.arctan2(dx, d_yz - L5)

        angles = np.zeros(5)
        angles[0] = hn               
        angles[1] = ht if is_right else -ht  
        angles[2] = -dg if is_right else dg  
        angles[3] = -(ht - dg) if is_right else (ht - dg) 
        angles[4] = -hn              
        return angles

    def step_physics(self, leg_targets):
        for i in range(10):
            aid = self.actuator_ids[i]
            if aid != -1: self.data.ctrl[aid] = leg_targets[i]
        for i in range(7):
            aid = self.actuator_ids[10 + i]
            if aid != -1: self.data.ctrl[aid] = 0.0

        mujoco.mj_step(self.model, self.data)
        
        quat = self.data.qpos[3:7] 
        r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])
        euler = r.as_euler('xyz', degrees=False)
        
        # Cập nhật giá trị cũ
        self.prev_roll, self.prev_pitch = self.roll, self.pitch
        # Cập nhật giá trị mới
        self.roll, self.pitch = euler[0], euler[1]
        
        self.smooth_roll = 0.3 * self.smooth_roll + 0.7 * self.roll
        self.smooth_pitch = 0.3 * self.smooth_pitch + 0.7 * self.pitch

    def get_obs(self):
        d_roll = (self.smooth_roll - self.prev_roll) / DT_AGENT
        d_pitch = (self.smooth_pitch - self.prev_pitch) / DT_AGENT
        obs = [self.smooth_roll, self.smooth_pitch, d_roll, d_pitch]
        if not self.is_left_support: obs[0] *= -1; obs[2] *= -1
        phase = (self.current_time % (STEPS_PER_PHASE * DT_AGENT * 2)) * np.pi
        return np.array(obs + [np.sin(phase), np.cos(phase)])

    def step(self, action, step_idx):
        smooth_act = 0.7 * self.prev_action + 0.3 * action
        self.prev_action = smooth_act
        
        dx_s, dy_s, dz_s, dst_s = smooth_act[0]*0.06, smooth_act[1]*0.02, smooth_act[2]*0.02, smooth_act[3]*0.02
        tz = STD_Z + dz_s
        
        total_steps = STEPS_PER_PHASE * 2
        progress = (step_idx % total_steps) / total_steps
        weight_shift = np.sin(progress * 2 * np.pi)
        self.is_left_support = (weight_shift > 0)
        
        lift_l = lift_r = 0.0
        if weight_shift < -0.4: lift_l = LIFT_H * ((-weight_shift - 0.4) / 0.6) ** 0.3
        elif weight_shift > 0.4: lift_r = LIFT_H * ((weight_shift - 0.4) / 0.6) ** 0.3

        target_y_l = 0.01 + dy_s; target_y_r = -0.01 - dy_s
        if weight_shift > 0: target_y_l -= 0.012 * weight_shift; target_y_r -= dst_s
        elif weight_shift < 0: target_y_r += 0.012 * abs(weight_shift); target_y_l += dst_s
            
        tx_l = dx_s if lift_l > 0.01 else 0.0; tx_r = dx_s if lift_r > 0.01 else 0.0
        
        if np.linalg.norm([tx_l, target_y_l, tz]) > SAFE_LIMIT or \
           np.linalg.norm([tx_r, target_y_r, tz]) > SAFE_LIMIT:
            return self.get_obs(), -5.0, True, "IK_FAIL"

        for i in range(N_SUBSTEPS):
            l_angles = self.solve_ik(tx_l, target_y_l, tz - lift_l, False)
            r_angles = self.solve_ik(tx_r, target_y_r, tz - lift_r, True)
            self.step_physics(np.concatenate((l_angles, r_angles)))
            if self.render_mode and self.viewer is not None: self.viewer.sync()

        self.current_time += DT_AGENT
        done = abs(self.smooth_roll) > 0.6 or abs(self.smooth_pitch) > 0.6
        reward = (1.5 + 3.5 * np.exp(-6.0 * np.sqrt(self.smooth_pitch**2 + self.smooth_roll**2)))
        if done: reward = -15.0
        
        return self.get_obs(), reward, done, "FALL" if done else "TIME"

# ==============================================================================
# 3. MAIN TRAINING LOOP
# ==============================================================================
def train():
    os.makedirs(WEIGHTS_DIR, exist_ok=True)
    history_path = os.path.join(WEIGHTS_DIR, "training_history.csv")
    
    print(f"🚀 KHỞI ĐỘNG MUJOCO TRAINING")
    print(f"📂 XML: {XML_PATH}")
    print(f"📂 WEIGHTS: {WEIGHTS_DIR}")

    env = HumanoidEnv(XML_PATH, render_mode=True)
    ppo = PPO(STATE_DIM, ACTION_DIM)
    
    latest_model = os.path.join(WEIGHTS_DIR, "latest_model.pth")
    if os.path.exists(latest_model):
        print(f"🔄 Nạp weights cũ từ: {latest_model}")
        ppo.policy.load_state_dict(torch.load(latest_model, map_location=DEVICE))
        ppo.policy_old.load_state_dict(ppo.policy.state_dict())

    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        env.viewer = viewer
        if not os.path.exists(history_path):
            with open(history_path, 'w', newline='') as f:
                csv.writer(f).writerow(["Episode", "Reward", "Steps", "Reason"])

        ep = 0
        best_reward = -float('inf')
        
        while viewer.is_running():
            state = env.reset(); ep_reward = 0; start_time = time.time()
            
            for t in range(MAX_STEPS):
                action = ppo.select_action(state)
                next_state, reward, done, reason = env.step(action, t)
                ppo.buffer_rewards.append(reward); ppo.buffer_is_terminals.append(done)
                state = next_state; ep_reward += reward
                
                if len(ppo.buffer_rewards) >= 8000:
                    print("\033[92m>>> UPDATING POLICY <<<\033[0m"); ppo.update()
                if done: break
            
            if ep_reward > best_reward:
                best_reward = ep_reward
                torch.save(ppo.policy.state_dict(), os.path.join(WEIGHTS_DIR, "best_model.pth"))
                print(f"\033[1;33m⭐ NEW BEST: {ep_reward:.2f}\033[0m")
            
            torch.save(ppo.policy.state_dict(), os.path.join(WEIGHTS_DIR, "latest_model.pth"))
            with open(history_path, 'a', newline='') as f:
                csv.writer(f).writerow([ep, f"{ep_reward:.2f}", t, reason])
            
            if ep % 100 == 0 and ep > 0:
                new_std = max(0.05, torch.sqrt(ppo.policy.action_var.mean()).item() * 0.98)
                ppo.policy.set_action_std(new_std)
                ppo.policy_old.set_action_std(new_std)
                print(f"\033[94m>>> DECAY STD TO: {new_std:.4f}\033[0m")

            print(f"Ep {ep} | Reward: {ep_reward:.2f} | Steps: {t} | Reason: {reason} | FPS: {t/(time.time()-start_time):.0f}")
            ep += 1

if __name__ == "__main__":
    train()