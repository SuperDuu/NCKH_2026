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
    
    # 1. [MỚI] Dùng file xacro turbo (Collision đơn giản)
    xacro_file = os.path.join(pkg_share, 'urdf', 'main_turbo.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    resource_path = os.path.join(pkg_share, '..')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 2. [MỚI] Dùng turbo.sdf và tắt giao diện (-s)
    world_file = os.path.join(pkg_share, 'worlds', 'turbo.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r -s -v 4 "{world_file}"'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'humanoid_robot', '-topic', 'robot_description', '-z', '0.296'],
        output='screen',
    )

    # 3. [MỚI] Thêm Service Bridge cho file Fast Python
    bridge_arguments = [
        '/world/empty/control@ros_gz_interfaces/srv/ControlWorld', # Reset nhanh
        '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose', # Pose nhanh
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    
    # Tự động thêm bridge cho các khớp (Copy từ file cũ của bạn)
    joints = [
        'base_hip_left', 'hip_hip_left', 'hip_knee_left', 'knee_ankle_left', 'ankle_ankle_left',
        'base_hip_right', 'hip_hip_right', 'hip_knee_right', 'knee_ankle_right', 'ankle_ankle_right',
        'hip_shoulder_left', 'shoulder_shoulder_left', 'shoulder_elbow_left',
        'hip_shoulder_right', 'shoulder_shoulder_right', 'shoulder_elbow_right',
        'base_hip_middle'
    ]
    for j in joints:
        bridge_arguments.append(f'/model/humanoid_robot/joint/{j}_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=bridge_arguments, output='screen', respawn=True,
    )

    imu_process_node = Node(package='ros2_pkg', executable='imu_process_node', parameters=[{'use_sim_time': True}])
    uvc_controller_node = Node(package='ros2_pkg', executable='uvc_node', parameters=[{'use_sim_time': True}])
    
    # 4. [MỚI] Chạy node Python mới (rl_training_fast)
    rl_training_node = Node(
        package='ros2_pkg',
        executable='rl_training_fast', # Node mới
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        node_robot_state_publisher, gazebo, spawn_robot, bridge,
        imu_process_node, uvc_controller_node, rl_training_node
    ])