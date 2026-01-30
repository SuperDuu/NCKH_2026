import numpy as np

class RobotMath:
    def __init__(self):
        # --- MECHANICAL PARAMETERS (Converted to Meters) ---
        self.L3 = 0.060      # 60mm -> 0.06m
        self.L4 = 0.1034     # 103.4mm -> 0.1034m (Matches URDF)
        self.L5 = 0.057      # 57mm -> 0.057m (Ankle offset)
        self.HEIGHT_STD = 0.200 # Standard height 200mm -> 0.2m
        
        # --- STATE VARIABLES ---
        self.fwct = 0.0
        self.support_leg = 0 # 0: Right support, 1: Left support
        self.autoH = self.HEIGHT_STD
        
        # Initial foot positions (Meters)
        self.dxi = 0.015      # 15mm -> 0.015m
        self.dyi = 0.020      # 20mm -> 0.020m (Initial stance width)
        self.dxis = 0.015
        self.dyis = -0.020
        
        self.pitch_prev = 0.0
        self.roll_prev = 0.0
        self.dxi_before, self.dyi_before = self.dxi, self.dyi

    def solve_ik(self, dx, dy, dz):
        """Inverse Kinematics solver matching uvc_node.cpp logic"""
        hn = np.arctan2(dy, dz)
        d_yz = np.sqrt(dz**2 + dy**2)
        
        dist_to_ankle = d_yz - self.L5
        
        # Cosine rule for knee angle
        # Note: dz must be sufficient for dist_to_ankle to form valid triangle
        cos3 = (pow(dist_to_ankle, 2) + dx**2 - self.L3**2 - self.L4**2) / (2 * self.L3 * self.L4)
        cos3 = np.clip(cos3, -1.0, 1.0)
        sin3 = np.sqrt(1.0 - cos3**2)
        
        dg = np.arctan2(sin3, cos3) 
        ht = np.arctan2(sin3 * self.L4, self.L3 + cos3 * self.L4) + np.arctan2(dx, dist_to_ankle)
        
        mct = -dg + ht 
        mcn = -hn      
        
        # Return logical angles: [Yaw, Hip, Knee, AnkleP, AnkleR]
        return [hn, ht, dg, mct, mcn]

    def compute_joints(self, pitch, roll, params):
        """Calculates joint angles and maps them to actuator names."""
        # Unpack params (Assuming RL provides values in millimeters for stance/lift)
        gain, scale_b, fwctEnd, landP, stance_w_mm, fhMax_mm, rr = params
        stance_w = stance_w_mm / 1000.0
        fhMax = fhMax_mm / 1000.0

        # --- 1. UVC LOGIC ---
        p_der = (pitch - self.pitch_prev) / 0.01
        r_der = (roll - self.roll_prev) / 0.01
        self.pitch_prev, self.roll_prev = pitch, roll

        tilt_mag = np.sqrt(pitch**2 + roll**2)
        scale_f = scale_b * (1.0 + 0.8 * min(tilt_mag, 1.0))
        
        # Roll Control
        k_r = np.arctan(self.dyi / self.autoH)
        kl_r = self.autoH / np.cos(k_r)
        ks_r = k_r - (scale_f * roll + gain * r_der)
        self.dyi = kl_r * np.sin(ks_r)
        self.autoH = kl_r * np.cos(ks_r)
        
        # Pitch Control
        k_p = np.arctan(self.dxi / self.autoH)
        kl_p = self.autoH / np.cos(k_p)
        ks_p = k_p - (scale_f * pitch + gain * p_der)
        self.dxi = kl_p * np.sin(ks_p)
        self.autoH = kl_p * np.cos(ks_p)

        # --- 2. FOOT LIFT ---
        fh = 0.0
        if landP < self.fwct <= (fwctEnd - landP):
            sw_prog = (self.fwct - landP) / (fwctEnd - 2 * landP)
            fh = fhMax * np.sin(np.pi * sw_prog)
            fh = min(fh, self.autoH - 0.018) # Min clearance 18mm

        # --- 3. IK CALCULATION ---
        sway = 0.015 # 15mm sway offset
        self.dyis = -stance_w
        
        l_j = [0.0]*5; r_j = [0.0]*5

        if self.support_leg == 0: # Right Support
            r_j = self.solve_ik(self.dxi, self.dyi - sway, self.autoH)
            l_j = self.solve_ik(self.dxis, self.dyis, self.autoH - fh)
        else: # Left Support
            l_j = self.solve_ik(self.dxis, self.dyis + sway, self.autoH)
            r_j = self.solve_ik(self.dxi, self.dyi, self.autoH - fh)

        # --- 4. JOINT MAPPING (Matches Robot.xml Names) ---
        # The list order MUST match the actuator order in Robot.xml or how you assign them in the environment
        # Assuming environment sets ctrl as: [L_Yaw, L_Roll, L_Pitch, L_AnkleP, L_AnkleR, R_Yaw, R_Roll, R_Pitch, R_AnkleP, R_AnkleR]
        
        joints = [0.0] * 10
        
        # CHÂN TRÁI (Index 0-4 trong joint_angles, sẽ map vào ctrl[1-5])
        joints[0] = l_j[0]   # base_hip_left_joint
        joints[1] = -l_j[1]  # hip_hip_left_joint
        joints[2] = l_j[2]   # hip_knee_left_joint
        joints[3] = l_j[3]   # knee_ankle_left_joint (mct chân trái)
        joints[4] = l_j[4]   # ankle_ankle_left_joint
        
        # CHÂN PHẢI (Index 5-9 trong joint_angles, sẽ map vào ctrl[6-10])
        joints[5] = r_j[0]   # base_hip_right_joint
        joints[6] = r_j[1]   # hip_hip_right_joint
        joints[7] = -r_j[2]  # hip_knee_right_joint
        joints[8] = -r_j[3]  # knee_ankle_right_joint (mct chân phải - ĐẢO DẤU)
        joints[9] = -r_j[4]  # ankle_ankle_right_joint
        
        return joints
        
        return joints

    def update_cycle(self, fwctEnd):
        self.fwct += 1.0
        if self.fwct >= fwctEnd:
            self.fwct = 0.0
            self.support_leg ^= 1
            # Swap positions
            self.dxi, self.dxis = self.dxis, self.dxi
            self.dyi, self.dyis = self.dyis, self.dyi
            
            # Reset stance width to standard
            if self.support_leg == 0:
                self.dyi = 0.020; self.dyis = -0.020
            else:
                self.dyis = -0.020; self.dyi = 0.020
            
            self.dxi_before, self.dyi_before = self.dxi, self.dyi