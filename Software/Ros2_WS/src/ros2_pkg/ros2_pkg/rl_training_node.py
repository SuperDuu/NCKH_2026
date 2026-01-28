#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from geometry_msgs.msg import Vector3
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
import threading
import time
import subprocess
import signal

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
        self.log_std = nn.Parameter(torch.ones(action_dim) * -0.5)

    def forward(self, x):
        x = self.fc(x)
        mu = torch.tanh(self.mu(x))
        std = torch.exp(self.log_std)
        return mu, std

class RLTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_training_node')

        # 1. ROS Communication
        self.param_pub = self.create_publisher(Float64MultiArray, '/uvc_parameters', 10)
        self.feedback_sub = self.create_subscription(Vector3, '/uvc_rl_feedback', self.feedback_callback, 10)
        self.reset_pub = self.create_publisher(Bool, '/uvc_reset', 10)

        # 2. Joint Publishers (for reset)
        self.joint_pubs = {}
        joint_names = [
            'base_hip_left_joint', 'hip_hip_left_joint', 'hip_knee_left_joint',
            'knee_ankle_left_joint', 'ankle_ankle_left_joint',
            'base_hip_right_joint', 'hip_hip_right_joint', 'hip_knee_right_joint',
            'knee_ankle_right_joint', 'ankle_ankle_right_joint',
            'hip_shoulder_left_joint', 'shoulder_shoulder_left_joint', 'shoulder_elbow_left_joint',
            'hip_shoulder_right_joint', 'shoulder_shoulder_right_joint', 'shoulder_elbow_right_joint',
            'base_hip_middle_joint',
        ]
        
        for joint_name in joint_names:
            topic = f'/model/humanoid_robot/joint/{joint_name}/cmd_pos'
            self.joint_pubs[joint_name] = self.create_publisher(Float64, topic, 10)

        # 3. State Variables
        self.current_obs = np.array([0.0, 0.0, 0.0]) 
        self.data_received = False
        self.is_falling = False
        self.reset_count = 0  # Track reset attempts

        # 4. RL Hyperparameters
        self.state_dim = 3
        self.action_dim = 7
        self.policy = PolicyNetwork(self.state_dim, self.action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.001)

        self.param_mins = np.array([0.10, 0.10, 20.0, 5.0, 30.0, 8.0, 0.05])
        self.param_maxs = np.array([0.30, 0.20, 35.0, 8.0, 45.0, 15.0, 0.20])
        self.current_params = np.array([0.18, 0.12, 25.0, 6.0, 35.0, 12.0, 0.10])

        # Tracking
        self.best_reward = -np.inf
        self.best_params = self.current_params.copy()
        self.episode_rewards = []
        self.episode_tilt_history = []

        # 5. Threading
        self.running = True
        self.train_thread = threading.Thread(target=self.train_loop, daemon=True)
        self.train_thread.start()

    def feedback_callback(self, msg):
        self.current_obs = np.array([msg.x, msg.y, msg.z])
        self.data_received = True
        if abs(msg.x) > 30.0 or abs(msg.y) > 30.0:
            self.is_falling = True

    def run_gz_command(self, service_name, req_type, rep_type, req_data, timeout=3.0, max_retries=3):
        """
        Chạy gz service command với retry logic
        """
        for attempt in range(max_retries):
            try:
                cmd = ['gz', 'service', '-s', service_name,
                       '--reqtype', req_type,
                       '--reptype', rep_type,
                       '--timeout', '2000',
                       '--req', req_data]
                
                # Sử dụng Popen để có thể kill process nếu timeout
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=lambda: signal.signal(signal.SIGPIPE, signal.SIG_DFL)
                )
                
                try:
                    stdout, stderr = process.communicate(timeout=timeout)
                    
                    if process.returncode == 0:
                        if attempt > 0:
                            self.get_logger().info(f'✓ {service_name} succeeded on attempt {attempt + 1}')
                        return True
                    else:
                        self.get_logger().warn(f'Attempt {attempt + 1}: {service_name} failed: {stderr.decode()}')
                        
                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f'Attempt {attempt + 1}: {service_name} timeout, killing process')
                    process.kill()
                    process.wait()
                    
                time.sleep(0.2)  # Wait before retry
                
            except Exception as e:
                self.get_logger().warn(f'Attempt {attempt + 1}: Exception in {service_name}: {e}')
                time.sleep(0.2)
        
        self.get_logger().error(f'❌ {service_name} failed after {max_retries} attempts')
        return False

    def pause_simulation(self):
        """Pause Gazebo with retry"""
        return self.run_gz_command(
            '/world/empty/control',
            'gz.msgs.WorldControl',
            'gz.msgs.Boolean',
            'pause: true'
        )

    def unpause_simulation(self):
        """Unpause Gazebo with retry"""
        return self.run_gz_command(
            '/world/empty/control',
            'gz.msgs.WorldControl',
            'gz.msgs.Boolean',
            'pause: false'
        )

    def reset_robot_pose(self):
        """Reset robot pose with retry"""
        pose_msg = '''
        name: "humanoid_robot"
        position {
          x: 0.0
          y: 0.0
          z: 0.3
        }
        orientation {
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
        }
        '''
        
        success = self.run_gz_command(
            '/world/empty/set_pose',
            'gz.msgs.Pose',
            'gz.msgs.Boolean',
            pose_msg,
            timeout=4.0
        )
        
        if success:
            self.get_logger().info('✓ Robot pose reset')
        else:
            self.get_logger().error('❌ Failed to reset pose')
        
        return success

    def reset_all_joints(self):
        """Reset all joints to neutral position"""
        msg = Float64()
        msg.data = 0.0
        
        # Publish multiple times to ensure delivery
        for _ in range(10):
            for pub in self.joint_pubs.values():
                pub.publish(msg)
            time.sleep(0.01)
        
        self.get_logger().info('✓ All joints commanded to neutral')
        time.sleep(0.3)
    def reset_simulation(self):
        self.reset_count += 1
        self.get_logger().info(f'========================================')
        self.get_logger().info(f'🔄 RESET #{self.reset_count}: Episode Start')
        
        # 1. Gửi lệnh Reset=True: Bắt UVC vào chế độ chờ (mode -1) và duỗi chân
        self.reset_pub.publish(Bool(data=True))
        time.sleep(0.2) 
        
        # 2. Pause & Reset Pose như cũ
        self.pause_simulation()
        self.reset_robot_pose()
        self.reset_all_joints() # Đưa khớp về 0
        
        # 3. Unpause để robot rơi xuống sàn
        self.unpause_simulation()
        
        # --- THAY ĐỔI QUAN TRỌNG Ở ĐÂY ---
        
        # 4. Đợi 1 giây để robot rơi chạm đất và hết rung lắc
        self.get_logger().info('⏳ Waiting for physics to settle...')
        time.sleep(1.0) 
        
        # 5. BÂY GIỜ mới báo UVC bắt đầu Calibrate (Reset=False)
        self.get_logger().info('▶ Triggering Calibration...')
        self.reset_pub.publish(Bool(data=False))
        
        # 6. Đợi Calibration hoàn tất (100 samples * 50ms = 5s -> đợi dư ra chút)
        # Chúng ta đợi khoảng 5.5s
        time.sleep(5.5)
        
        self.is_falling = False
        self.data_received = False
        
        self.get_logger().info('✓ Reset sequence complete. Robot ready.')
        self.get_logger().info(f'========================================\n')
    # def reset_simulation(self):
    #     """Reset simulation for new episode"""
    #     self.reset_count += 1
    #     self.get_logger().info(f'========================================')
    #     self.get_logger().info(f'🔄 RESET #{self.reset_count}: Episode Start')
    #     self.get_logger().info(f'========================================')
        
    #     # 1. Request UVC reset FIRST (before Gazebo reset)
    #     try:
    #         self.reset_pub.publish(Bool(data=True))
    #         self.get_logger().info('→ UVC reset requested')
    #         time.sleep(0.3)  # Give UVC time to process
    #     except Exception as e:
    #         self.get_logger().warn(f'UVC reset request failed: {e}')
        
    #     # 2. Pause simulation
    #     if not self.pause_simulation():
    #         self.get_logger().error('Failed to pause, attempting to continue...')
    #     time.sleep(0.2)
        
    #     # 3. Reset robot pose
    #     if not self.reset_robot_pose():
    #         self.get_logger().error('Failed to reset pose, attempting to continue...')
    #     time.sleep(0.2)
        
    #     # 4. Reset joints (wake up controllers)
    #     self.reset_all_joints()
        
    #     # 5. Reset internal state
    #     self.is_falling = False
    #     self.data_received = False
    #     self.current_obs = np.array([0.0, 0.0, 0.0])
        
    #     # 6. Publish safe params
    #     safe_params = Float64MultiArray()
    #     safe_params.data = list(self.current_params)
    #     self.param_pub.publish(safe_params)
    #     time.sleep(0.1)
        
    #     # 7. Unpause simulation
    #     if not self.unpause_simulation():
    #         self.get_logger().error('Failed to unpause, attempting to continue...')
    #     time.sleep(0.2)
        
    #     # 8. Clear UVC reset flag
    #     try:
    #         self.reset_pub.publish(Bool(data=False))
    #     except:
    #         pass
        
    #     # 9. Wait for stable
    #     self.get_logger().info('→ Waiting for robot to stabilize...')
    #     stable = self.wait_for_stable()
        
    #     if stable:
    #         self.get_logger().info('✓ Reset complete - Robot stable')
    #     else:
    #         self.get_logger().warn('⚠ Reset timeout - Continuing anyway')
        
    #     self.get_logger().info(f'========================================\n')

    def wait_for_stable(self):
        """Wait for robot to stabilize"""
        stable_needed = 8  # Increased from 6
        stable_cnt = 0
        wait_cycles = 0
        max_cycles = 300  # Increased from 200
        
        while wait_cycles < max_cycles:
            if not self.data_received:
                time.sleep(0.05)
                wait_cycles += 1
                continue
                
            p, r, _ = self.current_obs
            tilt_now = float(np.sqrt(p**2 + r**2))
            
            if tilt_now < 8.0:  # More lenient threshold
                stable_cnt += 1
                if stable_cnt >= stable_needed:
                    self.get_logger().info(f'✓ Stable after {wait_cycles * 0.05:.1f}s (tilt={tilt_now:.1f}°)')
                    return True
            else:
                stable_cnt = 0
                
            time.sleep(0.05)
            wait_cycles += 1
        
        # Log final state even if not stable
        p, r, _ = self.current_obs
        tilt_final = float(np.sqrt(p**2 + r**2))
        self.get_logger().warn(f'Timeout after {max_cycles * 0.05:.1f}s (final tilt={tilt_final:.1f}°)')
        return False

    def scale_action(self, action):
        return self.param_mins + (action + 1.0) * 0.5 * (self.param_maxs - self.param_mins)

    def save_checkpoint(self, episode):
        try:
            torch.save(self.policy.state_dict(), f"/tmp/rl_policy_ep{episode}.pt")
            self.get_logger().info(f"✓ Checkpoint saved at episode {episode}")
        except Exception as e:
            self.get_logger().error(f"Error saving checkpoint: {e}")

    def train_loop(self):
        time.sleep(3) 
        self.get_logger().info("=== RL TRAINING STARTED ===")
        print("\n" + "="*100, flush=True)
        print("RL TRAINING - Learning UVC Parameters", flush=True)
        print("="*100 + "\n", flush=True)

        for episode in range(5000):
            self.reset_simulation()
            ep_reward = 0.0
            ep_step_count = 0
            tilt_samples = []
            phys_act = self.current_params.copy()

            if not self.running:
                break
                
            try:
                for step in range(500):
                    if not self.data_received:
                        time.sleep(0.05)
                        continue

                    # STATE
                    pitch, roll, _ = self.current_obs
                    tilt_mag = float(np.sqrt(pitch**2 + roll**2))
                    state = np.array([pitch, roll, tilt_mag], dtype=np.float32)
                    tilt_samples.append(tilt_mag)

                    # ACTION
                    state_tensor = torch.FloatTensor(state)
                    mu, std = self.policy(state_tensor)
                    dist = Normal(mu, std)
                    action = dist.sample()
                    if action.dim() > 1:
                        action = action.squeeze(0)

                    phys_act = self.scale_action(action.detach().cpu().numpy().reshape(-1))
                    msg = Float64MultiArray()
                    msg.data = list(phys_act)
                    self.param_pub.publish(msg)

                    # REWARD
                    stability_score = 1.0 / (1.0 + 0.05 * tilt_mag)
                    fall_penalty = 1.0 if tilt_mag > 45.0 else 0.0
                    reward = stability_score - 2.0 * fall_penalty
                    ep_reward += reward
                    ep_step_count += 1

                    # LEARNING
                    log_prob = dist.log_prob(action).sum()
                    loss = -log_prob * float(reward)

                    self.optimizer.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(self.policy.parameters(), 1.0)
                    self.optimizer.step()

                    if step % 50 == 0:
                        print(f"Ep {episode:4d} Step {step:3d} | Tilt: {tilt_mag:.1f}° | Reward: {reward:.3f}", flush=True)

                    time.sleep(0.05)

                    if self.is_falling:
                        self.get_logger().warn(f"⚠ Episode {episode} - Robot fell at step {step}")
                        phys_act = self.current_params.copy()
                        break

            except Exception as e:
                self.get_logger().error(f"Episode {episode} error: {e}")
                continue

            # EPISODE SUMMARY
            avg_tilt = np.mean(tilt_samples) if tilt_samples else 0.0

            if ep_reward > self.best_reward:
                self.best_reward = ep_reward
                self.best_params = phys_act.copy()
                print(f"🎉 NEW BEST! Ep {episode} | Reward: {ep_reward:.2f} | AvgTilt: {avg_tilt:.1f}°", flush=True)
            else:
                print(f"Ep {episode} | Reward: {ep_reward:.2f} | AvgTilt: {avg_tilt:.1f}° | Steps: {ep_step_count}", flush=True)

            if episode % 100 == 0 and episode > 0:
                self.save_checkpoint(episode)

def main():
    rclpy.init()
    node = RLTrainingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RL Training interrupted")
    finally:
        try:
            node.running = False
            if hasattr(node, 'train_thread'):
                node.train_thread.join(timeout=2.0)
        except:
            pass
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()