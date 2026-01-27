#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Vector3
from ros_gz_interfaces.srv import ControlWorld # Service để reset Gazebo
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import threading
import time
import subprocess
# --- MẠNG NEURAL ---
class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        self.fc = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU()
        )
        self.mu = nn.Linear(256, action_dim)
        # Use 1D log_std so std has shape (action_dim,) and samples are 1D
        self.log_std = nn.Parameter(torch.ones(action_dim) * -0.5)

    def forward(self, x):
        x = self.fc(x)
        mu = torch.tanh(self.mu(x)) # Action trong khoảng [-1, 1]
        std = torch.exp(self.log_std)
        return mu, std

class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')
        
        # 1. ROS Communication
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        # Publisher to request UVC node to reset to standing pose
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)
        
        # 2. State Variables
        self.current_obs = np.array([0.0, 0.0, 0.0]) 
        self.data_received = False
        self.is_falling = False
        
        # 3. RL Hyperparameters
        # State: [pitch, roll, tilt_magnitude]
        # Action: [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]
        self.state_dim = 3
        self.action_dim = 7
        self.policy = PolicyNetwork(self.state_dim, self.action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.001)  # Tăng learning rate
        
        # Mapping action thực tế - từ tối ưu hiện tại
        # [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]
        self.param_mins = np.array([0.10, 0.10, 20.0, 5.0, 30.0, 8.0, 0.05])
        self.param_maxs = np.array([0.30, 0.20, 35.0, 8.0, 45.0, 15.0, 0.20])
        
        # Giá trị tối ưu hiện tại làm initialization
        self.current_params = np.array([0.18, 0.12, 25.0, 6.0, 35.0, 12.0, 0.10])
        
        # Tracking
        self.best_reward = -np.inf
        self.best_params = self.current_params.copy()
        self.episode_rewards = []
        self.episode_tilt_history = []

        # 4. Threading
        self.running = True
        # Make thread daemon so it won't block shutdown; train_loop will check `self.running`
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True
        print(f"DEBUG: Nhan du lieu IMU - Pitch: {msg.x:.2f}, Roll: {msg.y:.2f}")
        # Nếu nghiêng quá 30 độ (Pitch hoặc Roll)
        if abs(msg.x) > 30.0 or abs(msg.y) > 30.0:
            self.is_falling = True

    def reset_simulation(self):
        """Reset robot state mà không gọi Gazebo service (tránh crash)"""
        self.get_logger().info('--- RESET: Episode Start ---')
        
        # Chỉ reset internal tracking variables, không reset Gazebo
        self.is_falling = False
        self.data_received = False
        self.current_obs = np.array([0.0, 0.0, 0.0])
        
        # Publish safe/default params to stop aggressive actions
        try:
            safe_params = Float64MultiArray()
            safe_params.data = list(self.current_params)
            self.param_pub.publish(safe_params)
        except Exception:
            pass

        # Chờ robot settle và đợi IMU báo đứng thẳng trước khi tiếp tục
        stable_needed = 6
        stable_cnt = 0
        wait_cycles = 0
        max_cycles = 600  # ~30s (600 * 0.05)
        while wait_cycles < max_cycles:
            if not self.data_received:
                stable_cnt = 0
                time.sleep(0.05)
                wait_cycles += 1
                continue
            p, r, _ = self.current_obs
            tilt_now = float(np.sqrt(p**2 + r**2))
            if tilt_now < 5.0:
                stable_cnt += 1
                if stable_cnt >= stable_needed:
                    break
            else:
                stable_cnt = 0
            time.sleep(0.05)
            wait_cycles += 1
    def scale_action(self, action):
        return self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)
    
    def save_checkpoint(self, episode):
        """Lưu best params và model checkpoint"""
        try:
            torch.save(self.policy.state_dict(), f"/tmp/rl_policy_ep{episode}.pt")
            self.get_logger().info(f"✓ Checkpoint saved at episode {episode}")
            self.get_logger().info(
                f"  Best Params: Gain={self.best_params[0]:.3f}, Scale={self.best_params[1]:.3f}, "
                f"StepDur={self.best_params[2]:.1f}, LandingPhase={self.best_params[3]:.1f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error saving checkpoint: {e}")

    def train_loop(self):
        time.sleep(3) 
        self.get_logger().info("--- HỆ THỐNG RL ĐÃ SẴN SÀNG ---")
        self.get_logger().info("ACTION SPACE: [gain, scale_base, step_duration, landing_phase, stance_width, max_foot_lift, recovery_rate]")
        print("\n" + "="*100, flush=True)
        print("RL TRAINING STARTED - Learning UVC Parameters", flush=True)
        print("="*100 + "\n", flush=True)
        
        for episode in range(5000):
            self.reset_simulation()
            ep_reward = 0.0
            ep_step_count = 0
            tilt_samples = []
            phys_act = None  # Track last action for best params
            
            if not self.running:
                break
            try:
                for step in range(500):
                    if not self.data_received:
                        time.sleep(0.05)
                        continue

                    # === STATE ===
                    pitch, roll, _ = self.current_obs
                    # Chuyển tilt_mag thành float scalar ngay từ đầu
                    tilt_mag = float(np.sqrt(pitch**2 + roll**2))
                    state = np.array([pitch, roll, tilt_mag], dtype=np.float32)
                    tilt_samples.append(tilt_mag)
                    
                    # === ACTION ===
                    state_tensor = torch.FloatTensor(state)
                    mu, std = self.policy(state_tensor)
                    dist = Normal(mu, std)
                    action = dist.sample()
                    # Ensure action is 1D (shape (action_dim,)) not (1, action_dim)
                    if action.dim() > 1:
                        action = action.squeeze(0)
                    
                    # Publish to C++
                    phys_act = self.scale_action(action.detach().cpu().numpy().reshape(-1))
                    msg = Float64MultiArray()
                    msg.data = list(phys_act)  # Convert numpy array to list
                    self.param_pub.publish(msg)
                    
                    # === REWARD ===
                    # tilt_mag đã là Python float từ trên
                    stability_score = 1.0 / (1.0 + 0.05 * tilt_mag)
                    fall_penalty = 1.0 if tilt_mag > 45.0 else 0.0
                    reward = stability_score - 2.0 * fall_penalty
                    ep_reward += reward
                    ep_step_count += 1
                    
                    # === LEARNING ===
                    # Tính loss - pure PyTorch operations
                    log_prob = dist.log_prob(action)  # Shape: (7,)
                    log_prob_sum = log_prob.sum()  # Scalar tensor
                    
                    # Convert reward to Python float, then back to tensor
                    reward_val = float(reward)
                    reward_tensor = torch.as_tensor(reward_val, dtype=torch.float32)
                    
                    # Loss = -log_prob_sum * reward (both are scalar tensors)
                    loss = -log_prob_sum * reward_tensor
                    
                    self.optimizer.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 1.0)
                    self.optimizer.step()

                    # Log mỗi 50 bước
                    if step % 50 == 0:
                        log_msg = (
                            f"Ep {episode:4d} Step {step:3d} | "
                            f"Gain: {phys_act[0]:.3f} | Scale: {phys_act[1]:.3f} | "
                            f"StepDur: {phys_act[2]:.1f} | Tilt: {tilt_mag:.1f}° | Reward: {reward:.3f}"
                        )
                        self.get_logger().info(log_msg)
                        print(log_msg, flush=True)  # Ensure terminal output

                    time.sleep(0.05)  # 20Hz

                    if self.is_falling:
                        self.get_logger().warn(f"Episode {episode} - Robot ngã tại bước {step}, tilt = {tilt_mag:.1f}°")
                        print(f"⚠ Episode {episode}: Robot fell at step {step} with tilt {tilt_mag:.1f}°", flush=True)
                        # Request UVC node to reset to standing pose immediately
                        try:
                            self.reset_pub.publish(Bool(data=True))
                        except Exception:
                            pass
                        # Immediately publish safe/default parameters to stop aggressive actions
                        safe_params = Float64MultiArray()
                        safe_params.data = list(self.current_params)
                        try:
                            self.param_pub.publish(safe_params)
                        except Exception:
                            pass
                        # Give simulator a moment to settle
                        time.sleep(0.5)
                        # Reset internal episode state and wait for upright (reset_simulation will also publish safe params)
                        self.reset_simulation()
                        # Clear reset request
                        try:
                            self.reset_pub.publish(Bool(data=False))
                        except Exception:
                            pass
                        # Ensure phys_act reflects safe_params for summary
                        phys_act = self.current_params.copy()
                        break
                
            except Exception as e:
                self.get_logger().error(f"Error in episode {episode}: {e}")
                print(f"❌ Episode {episode} error: {e}", flush=True)
                continue
            
            # === EPISODE SUMMARY ===
            avg_tilt = np.mean(tilt_samples) if tilt_samples else 0.0
            max_tilt = np.max(tilt_samples) if tilt_samples else 0.0
            
            self.episode_rewards.append(ep_reward)
            self.episode_tilt_history.append((avg_tilt, max_tilt))
            
            # Track best params
            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                self.best_params = phys_act.copy()
                log_msg = (
                    f"🎉 NEW BEST! Episode {episode} | Reward: {ep_reward:.2f} | "
                    f"AvgTilt: {avg_tilt:.1f}° | MaxTilt: {max_tilt:.1f}° | "
                    f"BestParams: G={self.best_params[0]:.3f} S={self.best_params[1]:.3f} "
                    f"SD={self.best_params[2]:.1f} LP={self.best_params[3]:.1f}"
                )
                self.get_logger().info(log_msg)
                print(log_msg, flush=True)
            else:
                log_msg = (
                    f"Episode {episode:4d} | Reward: {ep_reward:.2f} | "
                    f"AvgTilt: {avg_tilt:.1f}° | MaxTilt: {max_tilt:.1f}° | "
                    f"Steps: {ep_step_count}"
                )
                self.get_logger().info(log_msg)
                print(log_msg, flush=True)
            
            # Lưu best params mỗi 100 episode
            if episode % 100 == 0 and episode > 0:
                self.save_checkpoint(episode)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RL Training interrupted by user")
    finally:
        # Signal training thread to stop and join
        try:
            node.running = False
            if hasattr(node, 'train_thread'):
                node.train_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore shutdown errors

if __name__ == '__main__':
    main()