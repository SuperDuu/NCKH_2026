import csv
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Đường dẫn tuyệt đối của bạn
CSV_PATH = '/home/du/Desktop/NCKH_2026/Software/Ros2_WS_v2/src/ros2_pkg_v2/ros2_pkg/weights/balancing_step_ppo/training_history.csv'

def visualize_workspace():
    x, y, z = [], [], []
    
    try:
        with open(CSV_PATH, mode='r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x.append(float(row['x']))
                y.append(float(row['y']))
                z.append(float(row['z']))
    except FileNotFoundError:
        print(f"Lỗi: Không tìm thấy file tại {CSV_PATH}")
        return

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    # Vẽ các điểm hợp lệ, màu sắc thay đổi theo độ cao Z
    scatter = ax.scatter(x, y, z, c=z, cmap='magma', s=2, alpha=0.6)

    ax.set_xlabel('X (m) - Tiến/Lùi')
    ax.set_ylabel('Y (m) - Trái/Phải')
    ax.set_zlabel('Z (m) - Độ cao')
    ax.set_title(f'Workspace Robot Humanoid - {len(x)} điểm khả thi')
    
    plt.colorbar(scatter, label='Độ cao Z (m)')
    
    # Giữ tỉ lệ các trục để không bị méo hình
    ax.set_box_aspect([1, 1, 1]) 
    plt.show()

if __name__ == "__main__":
    visualize_workspace()