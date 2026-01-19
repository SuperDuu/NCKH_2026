import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    pkg_name = 'ros2_pkg'
    pkg_share = get_package_share_directory(pkg_name)
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    
    robot_description = ParameterValue(Command(['xacro ',xacro_file]), value_type = str)

    rviz_config_path = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    
    resource_path = os.path.join(pkg_share, '..')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,

        }]
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments= ["-d",rviz_config_path]
        
    )

    return LaunchDescription([

        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path),
        
        node_robot_state_publisher,
        joint_state_publisher_gui_node,
        rviz2_node
        
    ])