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

    # 2. Thiết lập đường dẫn tài nguyên
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
        launch_arguments={'gz_args': ' empty.sdf'}.items(), # -r để tự động Play
    )

    # 5. Node Spawn Robot (ĐÃ SỬA ĐỂ KHÔNG BỊ NGÃ)
    # Chúng ta cài đặt sẵn các góc khớp để robot sinh ra ở tư thế "Ngồi Xổm" (Squat)
    # khớp với độ cao HEIGHT_STD = 135.0 trong code C++
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.29115', # Hạ thấp độ cao spawn vì chân đã co lại
            
            # Gập gối -1.0 rad (Ngồi xuống)
            '-J', 'hip_knee_left_joint', '-1.0',
            '-J', 'hip_knee_right_joint', '-1.0',
            
            # Gập hông 0.5 rad (Để lưng thẳng)
            '-J', 'hip_hip_left_joint', '0.5',
            '-J', 'hip_hip_right_joint', '0.5',
            
            # Gập cổ chân -0.5 rad (Để bàn chân phẳng)
            '-J', 'knee_ankle_left_joint', '-0.5',
            '-J', 'knee_ankle_right_joint', '-0.5'
        ],
        output='screen',
    )

    # 6. Node Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/humanoid_robot/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory'
        ],
        output='screen'
    )

    # 7. Node Xử lý IMU (MỚI THÊM VÀO)
    # Nhiệm vụ: Tính toán góc Roll/Pitch/Yaw
    imu_process_node = Node(
        package='ros2_pkg',
        executable='imu_process_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. Node Điều khiển UVC (MỚI)
    # Nhiệm vụ: Nhận góc -> Tính toán cân bằng -> Gửi lệnh Motor
    uvc_controller_node = Node(
        package='ros2_pkg',
        executable='uvc_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        imu_process_node,    # Đừng quên dòng này
        uvc_controller_node  # Đừng quên dòng này
    ])