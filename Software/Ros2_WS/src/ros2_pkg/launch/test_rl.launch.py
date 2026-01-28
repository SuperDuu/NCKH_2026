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
    
    # 1. Xử lý file Robot (Dùng main_turbo.xacro cho khớp với RL)
    # Nếu muốn xem visual đẹp thì đổi thành 'main.xacro', nhưng tốt nhất là dùng file đã train
    xacro_file = os.path.join(pkg_share, 'urdf', 'main_turbo.xacro')
    
    # Xử lý xacro bằng python thuần (Tránh lỗi YAML)
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

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

    # 4. Khởi động Gazebo Sim (CÓ GIAO DIỆN ĐỂ XEM)
    # Không dùng -s (headless), chỉ dùng -r (run immediately)
    world_file = os.path.join(pkg_share, 'worlds', 'turbo.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f' -v 4 {world_file}'}.items(),
    )

    # 5. Node Spawn Robot (Độ cao chuẩn 0.291)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.292', # Độ cao chuẩn của bạn
        ],
        output='screen',
    )

    # 6. Node Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # --- Cảm biến ---
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # --- Service Control (Quan trọng để Reset) ---
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',

            # --- Điều khiển Khớp ---
            '/model/humanoid_robot/joint/base_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_knee_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/knee_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/ankle_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            
            '/model/humanoid_robot/joint/base_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_knee_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/knee_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/ankle_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            
            '/model/humanoid_robot/joint/hip_shoulder_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/shoulder_shoulder_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/shoulder_elbow_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            
            '/model/humanoid_robot/joint/hip_shoulder_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/shoulder_shoulder_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/shoulder_elbow_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            
            '/model/humanoid_robot/joint/base_hip_middle_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen',
        respawn=True,
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
    
    # 9. Node RL INFERENCE (Chạy file Test thay vì Train)
    rl_inference_node = Node(
        package='ros2_pkg',
        executable='rl_inference',  # <--- CHẠY FILE NÀY
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        imu_process_node, 
        uvc_controller_node,
        rl_inference_node # <--- Đã thay thế rl_training_node
    ])