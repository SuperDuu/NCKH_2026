import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'ros2_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. Xử lý file Robot (Xacro -> URDF)
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Thiết lập đường dẫn tài nguyên (Mesh)
    resource_path = os.path.join(pkg_share, '..')

    # 3. Node Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 4. Khởi động Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # Thêm -r để chạy ngay lập tức, -v 4 để xem log lỗi nếu có
        launch_arguments={'gz_args': ' empty.sdf'}.items(), 
    )

    # 5. Node Spawn Robot (Z-SHAPE CONFIGURATION)
    # Cấu hình này khớp với code UVC mới: Hông âm, Gối dương, Cổ chân âm
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            # '-z', '0.29115', # Nâng cao chút để rơi xuống nhẹ nhàng (Tránh kẹt sàn)
            '-z', '0.3',
            # # --- TƯ THẾ ZIG-ZAG (Để đứng vững ngay) ---
            # # Gối gập RA SAU (Dương)
            # '-J', 'hip_knee_left_joint', '0.7',
            # '-J', 'hip_knee_right_joint', '0.7',
            
            # # Hông gập TỚI (Âm)
            # '-J', 'hip_hip_left_joint', '-0.35',
            # '-J', 'hip_hip_right_joint', '-0.35',
            
            # # Cổ chân bù lại (Âm)
            # '-J', 'knee_ankle_left_joint', '-0.35',
            # '-J', 'knee_ankle_right_joint', '-0.35'
        ],
        output='screen',
    )

    # 6. Node Bridge (CẬP NHẬT CHO JOINT POSITION CONTROLLER)
    # Tạo danh sách các cầu nối cần thiết
    bridge_arguments = [
        # --- Cảm biến ---
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        
        # --- Điều khiển 11 Khớp (Float64 -> Double) ---
        # Chân Trái
        '/model/humanoid_robot/joint/base_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/hip_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/hip_knee_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/knee_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/ankle_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        
        # Chân Phải
        '/model/humanoid_robot/joint/base_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/hip_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/hip_knee_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/knee_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/ankle_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        # Tay Trái
        '/model/humanoid_robot/joint/hip_shoulder_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/shoulder_shoulder_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/shoulder_elbow_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        
        # Tay Phải
        '/model/humanoid_robot/joint/hip_shoulder_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/shoulder_shoulder_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        '/model/humanoid_robot/joint/shoulder_elbow_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        # Hông Giữa
        '/model/humanoid_robot/joint/base_hip_middle_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',


        #rl
        # '/world/empty/control@ros_gz_interfaces/srv/ControlWorld[gz.msgs.WorldControl',
    ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_arguments,
        output='screen'
    )

    # 7. Node Xử lý IMU
    imu_process_node = Node(
        package='ros2_pkg',
        executable='imu_process_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Node Điều khiển UVC
    uvc_controller_node = Node(
        package='ros2_pkg',
        executable='uvc_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # # 9. Node RL Training (Bộ não học tập)
    # rl_training_node = Node(
    #     package='ros2_pkg',
    #     executable='rl_training_node.py', # Phải trùng với tên file trong thư mục của bạn
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )
    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        imu_process_node, 
        uvc_controller_node
        # rl_training_node
    ])