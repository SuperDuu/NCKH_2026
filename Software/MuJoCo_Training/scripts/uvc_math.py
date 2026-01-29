import numpy as np

class RobotMath:
    def __init__(self):
        # --- THÔNG SỐ CƠ KHÍ (Giữ nguyên từ uvc_node.cpp) ---
        self.L3, self.L4, self.L5 = 60.0, 100.0, 65.0
        self.HEIGHT_STD = 204.0 # HEIGHT_STD - 20
        
        # --- BIẾN TRẠNG THÁI KHỞI TẠO ---
        self.fwct = 0.0
        self.support_leg = 0 
        self.autoH = self.HEIGHT_STD
        self.dxi, self.dyi = 15.0, 20.0
        self.dxis, self.dyis = 15.0, -20.0
        self.pitch_prev, self.roll_prev = 0.0, 0.0

    def solve_ik(self, dx, dy, dz):
        """IK bám sát 100% uvc_node.cpp"""
        hn = np.arctan2(dy, dz)
        d_yz = np.sqrt(dz**2 + dy**2)
        cos3 = (pow(d_yz - self.L5, 2) + dx**2 - self.L3**2 - self.L4**2) / (2 * self.L3 * self.L4)
        cos3 = np.clip(cos3, -1.0, 1.0)
        dg = np.arctan2(np.sqrt(1.0 - cos3**2), cos3)
        ht = np.arctan2(np.sqrt(1.0 - cos3**2) * self.L4, self.L3 + cos3 * self.L4) + np.arctan2(dx, d_yz - self.L5)
        return [hn, ht, dg, -dg + ht, -hn]

    def compute_joints(self, pitch, roll, params):
        """Thực thi ĐÚNG 7 tham số theo Index từ file train của bạn"""
        # Index bám sát hàm rl_param_callback:
        # 0:gain, 1:scale_base, 2:fwctEnd, 3:landing_phase, 4:stance_width, 5:max_foot_lift, 6:recovery_rate
        gain, scale_b, fwctEnd, landP, stance_w, fhMax, rr = params

        # Tính đạo hàm IMU
        p_der = (pitch - self.pitch_prev) / 0.01
        r_der = (roll - self.roll_prev) / 0.01
        self.pitch_prev, self.roll_prev = pitch, roll

        # 1. UVC Geometry Control (Dòng 342-411)
        tilt_mag = np.sqrt(pitch**2 + roll**2)
        scale_f = scale_b * (1.0 + 0.8 * min(tilt_mag, 1.0))
        
        # Roll Control (Vặn núm gain/scale_b)
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

        # 2. Foot Lift (Vặn núm fhMax/landP)
        fh = 0.0
        if landP < self.fwct <= (fwctEnd - landP):
            sw_prog = (self.fwct - landP) / (fwctEnd - 2 * landP)
            fh = fhMax * np.sin(np.pi * sw_prog)
            fh = min(fh, self.autoH - 18.0) # min_clearance mặc định 18.0

        # 3. IK & Mapping (Vặn núm stance_w)
        sway = 15.0
        self.dyis = -stance_w # RL học cách xoè chân rộng ra
        if self.support_leg == 0: # PHẢI trụ
            r_j = self.solve_ik(self.dxi, self.dyi - sway, self.autoH)
            l_j = self.solve_ik(-self.dxi, self.dyis, self.autoH - fh)
        else: # TRÁI trụ
            l_j = self.solve_ik(-self.dxi, self.dyis + sway, self.autoH)
            r_j = self.solve_ik(self.dxi, self.dyi, self.autoH - fh)

        return [l_j[0], -l_j[1], l_j[2], l_j[3], l_j[4], r_j[0], r_j[1], -r_j[2], -r_j[3], -r_j[4]]

    def update_cycle(self, fwctEnd):
        """Cập nhật bộ đếm chu kỳ bước (fwct)"""
        self.fwct += 1.0
        if self.fwct >= fwctEnd:
            self.fwct = 0.0
            self.support_leg ^= 1 # Hoán đổi chân trụ (0 <-> 1)
            # Reset vị trí cơ bản sau mỗi bước bám sát uvc_node.cpp
            self.dxi, self.dyi = 10.0, 5.0 
            self.dxi_before, self.dyi_before = self.dxi, self.dyi