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
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    
    resource_path = os.path.join(pkg_share, '..')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        # '-r empty.sdf' va 'empty.sdf' 
        launch_arguments={'gz_args': 'empty.sdf'}.items(), 
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', 'robot_description',
            '-z', '0.29115' 
        ],
        output='screen',
    )

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

        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])