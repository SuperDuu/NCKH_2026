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
    
    # 1. Xử lý file Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Cấu hình môi trường (Giữ nguyên của bạn)
    set_env_vars = [
        SetEnvironmentVariable(name='CYCLONEDDS_URI', value='/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/cyclonedds.xml'),
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='LOCALHOST'),
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(pkg_share, '..'))
    ]

    # 3. Nodes cơ bản
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '  empty.sdf'}.items(),
    )
    imu_proc = Node(
        package='ros2_pkg',
        executable='imu_process_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid_robot', '-topic', 'robot_description', '-z', '0.31'],
    )

    # 4. Bridge (Giữ nguyên các topic bạn đã khai báo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Left leg
            '/model/humanoid_robot/joint/base_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_knee_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/knee_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/ankle_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Right leg
            '/model/humanoid_robot/joint/base_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_hip_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_knee_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/knee_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/ankle_ankle_right_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        output='screen'
    )

    # 5. Node C++ Test (Thêm vào đây)
    node_test_joints = Node(
        package=pkg_name,
        executable='task_space_controller', # Tên executable khai báo trong CMakeLists.txt
        output='screen'
    )

    return LaunchDescription(set_env_vars + [
        imu_proc,
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        node_test_joints # Chạy node test luôn
    ])