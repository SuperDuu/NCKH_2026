import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'ros2_pkg_v2'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Xử lý URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    resource_path = os.path.join(pkg_share, '..')

    # Cấu hình GPU & Resource
    set_env_vars = [
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
    ]

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # Gazebo Sim — KHÔNG có -r để bạn tự bấm Play
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid_robot', '-topic', 'robot_description', '-z', '0.3'],
        output='screen',
    )

    # Bridge: IMU + Clock + Joint State
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': True}]
    )

    # === SPAWN CONTROLLERS (quan trọng!) ===
    # Đợi 3 giây để controller_manager khởi động xong
    spawn_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    spawn_joint_trajectory_controller = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        )]
    )

    # IMU Process Node
    imu_proc = Node(
        package='ros2_pkg_v2',
        executable='imu_process_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Low Level Controller (humanoid_llc_node)
    llc_node = Node(
        package='ros2_pkg_v2',
        executable='humanoid_llc_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Stand Test Node — robot đứng yên
    stand_test = Node(
        package='ros2_pkg_v2',
        executable='stand_test',
        output='screen',
        emulate_tty=True,
        prefix=['python3 -u'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(
        set_env_vars + [
            node_robot_state_publisher,
            gazebo,
            spawn_robot,
            bridge,
            spawn_joint_state_broadcaster,
            spawn_joint_trajectory_controller,
            imu_proc,
            llc_node,
            stand_test,
        ]
    )
