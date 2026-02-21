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
    
    # Xử lý file Robot URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

    resource_path = os.path.join(pkg_share, '..')

    # cấu hình mạng tối ưu cho FastDDS
    set_rmw = SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp')
    set_discovery = SetEnvironmentVariable(name='ROS_AUTOMATIC_DISCOVERY_RANGE', value='SUBNET')

    # Cấu hình GPU & Gazebo Resource
    set_env_vars = [
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)
    ]

    # NODES KHAI BÁO
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ' -r empty.sdf'}.items(), 
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid_robot', '-topic', 'robot_description', '-z', '0.35'],
        output='screen',
    )

    # === BRIDGE CẬP NHẬT: THÊM ODOMETRY ĐỂ HỌC ĐI BỘ ===
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # ODOS: Lấy dữ liệu vận tốc thực tế của robot từ Sim
            '/model/humanoid_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            
            # Bridge cho 10 khớp chân (Giữ nguyên)
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
        ],
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': True}]
    )

    imu_proc = Node(package='ros2_pkg', executable='imu_process_node', output='screen', parameters=[{'use_sim_time': True}])
    llc_node = Node(package='ros2_pkg', executable='humanoid_llc_node', output='screen', parameters=[{'use_sim_time': True}])
    
    # === THAY ĐỔI NODE HUẤN LUYỆN SANG WALKING ===
    rl_train = Node(
        package='ros2_pkg', 
        executable='ft_walk', # Đảm bảo tên này khớp với entry_point trong setup.py của bạn
        output='screen', 
        emulate_tty=True,            
        prefix=['python3 -u'],       
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        set_rmw, 
        set_discovery
    ] + set_env_vars + [
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        imu_proc, 
        llc_node,  
        rl_train   
    ])