import struct
import sys

def get_bounds(fname):
    with open(fname, 'rb') as f:
        header = f.read(80)
        num_tri = struct.unpack('I', f.read(4))[0]
        min_x = min_y = min_z = float('inf')
        max_x = max_y = max_z = float('-inf')
        
        for _ in range(num_tri):
            f.read(12)  # normal
            for _ in range(3):
                x, y, z = struct.unpack('3f', f.read(12))
                min_x = min(min_x, x); max_x = max(max_x, x)
                min_y = min(min_y, y); max_y = max(max_y, y)
                min_z = min(min_z, z); max_z = max(max_z, z)
            f.read(2)  # attr
        print(f"{fname} bounds:")
        print(f"X: {min_x:.4f} to {max_x:.4f} (size: {max_x-min_x:.4f})")
        print(f"Y: {min_y:.4f} to {max_y:.4f} (size: {max_y-min_y:.4f})")
        print(f"Z: {min_z:.4f} to {max_z:.4f} (size: {max_z-min_z:.4f})")

get_bounds('/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/urdf/meshes/ankle_ankle_right_link.STL')
