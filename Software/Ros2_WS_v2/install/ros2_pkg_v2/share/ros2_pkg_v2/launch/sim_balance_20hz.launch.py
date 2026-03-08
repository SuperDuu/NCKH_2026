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

    # Xử lý Robot URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

    resource_path = os.path.join(pkg_share, '..')

    # ========================================================================
    # 1. CẤU HÌNH MẠNG (ÉP FASTDDS & DỌN DẸP CYCLONE)
    # ========================================================================
    set_rmw = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp')
    unset_cyclone_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI', value='')
    set_discovery = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='SUBNET')

    # ========================================================================
    # 2. CẤU HÌNH GPU & GAZEBO RESOURCE
    # ========================================================================
    set_env_vars = [
        SetEnvironmentVariable(
            name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH', value=resource_path),
    ]

    # ========================================================================
    # 3. NODES
    # ========================================================================

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True,
        }],
    )

    world_file = os.path.join(pkg_share, 'worlds', 'empty_wrench.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f' -r {world_file}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.35',
        ],
        output='screen',
    )

    # === BRIDGE ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/humanoid_robot/link/base_footprint/wrench@geometry_msgs/msg/Wrench@gz.msgs.Wrench',
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
        respawn=True,
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./imu.publisher.reliability': 'best_effort',
            'qos_overrides./imu.subscription.reliability': 'best_effort',
            'qos_overrides./model/humanoid_robot/link/base_footprint/wrench.publisher.reliability': 'best_effort',
            'qos_overrides./model/humanoid_robot/link/base_footprint/wrench.subscription.reliability': 'best_effort'
        }],
    )

    # Spawn Controllers
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

    # IMU Processor V2: Quat + Gyro + PureAccel → /imu/data @ 100Hz
    imu_proc_v2 = Node(
        package='ros2_pkg_v2',
        executable='imu_process_node_v2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Humanoid LLC V2 (low-level controller — IK, L3=0.08 L4=0.15 L5=0.065 Z=0.265)
    llc_node = Node(
        package='ros2_pkg_v2',
        executable='humanoid_llc_node_v2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # PPO Balance Training Node @ 20Hz (70D Architecture)
    # Delay 3s để đảm bảo imu_process_node_v2 đã khởi chạy và publish
    # dữ liệu ổn định trước khi PPO bắt đầu đọc. Nếu không delay,
    # HistoryBuffer sẽ chứa toàn zero → PPO học trên dữ liệu rác.
    ppo_balance = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros2_pkg_v2',
                executable='ppo_balance_sim_20hz',
                output='screen',
                emulate_tty=True,
                prefix=['python3 -u'],
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    return LaunchDescription([
        # Env config
        set_rmw,
        unset_cyclone_uri,
        set_discovery,
    ] + set_env_vars + [
        # Core nodes
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller,
        # Processing — khởi chạy TRƯỚC PPO
        imu_proc_v2,
        llc_node,
        # RL Training — delay 3s chờ IMU ổn định
        ppo_balance,
    ])
