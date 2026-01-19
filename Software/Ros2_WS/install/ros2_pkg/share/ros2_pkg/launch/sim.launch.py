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

    # 2. Thiết lập đường dẫn tài nguyên cho Gazebo (Để nhận diện package://)
    resource_path = os.path.join(pkg_share, '..')

    # 3. Node Robot State Publisher: Công bố cấu trúc TF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    # 4. Khởi động Gazebo Sim (Phiên bản Harmonic/Jazzy)
    # '-r' giúp simulation chạy ngay lập tức
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ' empty.sdf'}.items(), 
    )

    # 5. Node Spawn Robot: Đưa robot vào thế giới ảo
    # -z 0.2 là độ cao vừa đủ để robot PETG tiếp đất nhẹ nhàng
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.29115' 
            # '-z', '0.35'
        ],
        output='screen',
    )

    # 6. Node Bridge: Cầu nối dữ liệu ROS 2 <-> Gazebo
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

    # 7. Node điều khiển UVC + IK (Bộ não thăng bằng)
    uvc_controller_node = Node(
        package='ros2_pkg',
        executable='uvc_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Thiết lập biến môi trường để Gazebo tìm thấy file STL
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        uvc_controller_node
    ])