import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Khai báo package và đường dẫn
    pkg_name = 'ros2_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Đường dẫn file main.xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Thiết lập Resource Path cho Gazebo (Để tìm thấy thư mục meshes)
    # Trỏ vào thư mục install/ros2_pkg/share để Gazebo hiểu package://ros2_pkg
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

    # 4. Khởi động Gazebo Harmonic (empty world)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # Thay '-r empty.sdf' bằng 'empty.sdf' (Bỏ -r đi)
        launch_arguments={'gz_args': 'empty.sdf'}.items(), 
    )

    # 5. Node Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.29115' # Độ cao spawn để tránh va chạm mặt đất gây sập
        ],
        output='screen',
    )

    # 6. Cầu nối Bridge
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

    return LaunchDescription([
        # Khai báo biến môi trường cho Gazebo TRƯỚC khi chạy các node khác
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])