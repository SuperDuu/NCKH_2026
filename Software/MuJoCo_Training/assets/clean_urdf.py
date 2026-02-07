import os
import re

def clean_urdf_for_mujoco(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # 1. Thay đổi đường dẫn mesh thành tương đối
    # MuJoCo sẽ tìm trong assets/meshes/
    content = content.replace("package://ros2_pkg/urdf/meshes/", "meshes/")

    # 2. Loại bỏ toàn bộ các thẻ <gazebo> (vì MuJoCo không đọc được)
    content = re.sub(r'<gazebo.*?>.*?</gazebo>', '', content, flags=re.DOTALL)
    
    # 3. Loại bỏ các thẻ rác khác thường đi kèm từ xacro
    content = re.sub(r'<robot.*?>', '<robot name="humanoid_robot">', content)

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"✅ Đã dọn dẹp file URDF: {output_file}")

if __name__ == "__main__":
    urdf_input = "robot_final.urdf"
    urdf_cleaned = "robot_cleaned.urdf"
    
    if os.path.exists(urdf_input):
        clean_urdf_for_mujoco(urdf_input, urdf_cleaned)
        
        # Gọi lệnh compile của MuJoCo
        os.system(f"python3 -m mujoco.utils.compile {urdf_cleaned} robot_final.xml")
        print("🚀 Đã biên dịch thành công sang robot_final.xml")
    else:
        print("❌ Không tìm thấy file robot_final.urdf trong thư mục hiện tại!")