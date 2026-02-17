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
    
    # === CẤU HÌNH ĐƯỜNG DẪN ===
    # Sử dụng file cyclonedds.xml bạn đã tạo
    cyclonedds_xml_path = '/home/du/Desktop/NCKH_2026/Software/Ros2_WS/src/ros2_pkg/cyclonedds.xml'
    
    # Xử lý file Robot URDF/Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'main.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_raw = doc.toxml()

    resource_path = os.path.join(pkg_share, '..')

    # ========================================================================
    # 1. CẤU HÌNH CYCLONEDDS (QUAN TRỌNG CHO GAZEBO)
    # ========================================================================
    set_cyclonedds_config = SetEnvironmentVariable(
        name='CYCLONEDDS_URI', 
        value=cyclonedds_xml_path
    )
    set_rmw_implementation = SetEnvironmentVariable(
        name='RMW_IMPLEMENTATION', 
        value='rmw_cyclonedds_cpp'
    )
    # Ép ROS2 chỉ tìm kiếm node trong localhost để tránh nghẽn mạng
    set_discovery_range = SetEnvironmentVariable(
        name='ROS_AUTOMATIC_DISCOVERY_RANGE', 
        value='LOCALHOST'
    )

    # ========================================================================
    # 2. CẤU HÌNH NVIDIA GPU (DÀNH CHO T1200)
    # ========================================================================
    set_nvidia_offload = SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1')
    set_nvidia_provider = SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia')
    set_vk_icd = SetEnvironmentVariable(name='VK_ICD_FILENAMES', value='/usr/share/vulkan/icd.d/nvidia_icd.json')
    set_ogre_rtshader = SetEnvironmentVariable(name='OGRE_RTShader_Write', value='/tmp')
    set_vblank = SetEnvironmentVariable(name='__GL_SYNC_TO_VBLANK', value='0')
    set_egl_platform = SetEnvironmentVariable(name='__EGL_VENDOR_LIBRARY_FILENAMES', value='/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    set_gz_resource = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)

    # ========================================================================
    # 3. KHAI BÁO CÁC NODES
    # ========================================================================
    
    # A. Robot State Publisher (Tính toán TF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # B. Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ' -r empty.sdf'}.items(), 
    )

    # C. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'humanoid_robot', '-topic', 'robot_description', '-z', '0.35'],
        output='screen',
    )

    # D. ROS-GZ Bridge (Cầu nối Topic)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/humanoid_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Dịch vụ Reset
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            # Khớp chân trái
            '/model/humanoid_robot/joint/base_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_hip_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/hip_knee_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/knee_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/humanoid_robot/joint/ankle_ankle_left_joint/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Khớp chân phải
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

    # ========================================================================
    # 4. NODE ĐIỀU KHIỂN & TRAINING (ĐÃ CẬP NHẬT)
    # ========================================================================

    # Node xử lý IMU (Giữ lại để tạo topic /robot_orientation cho RL dùng)
    imu_proc = Node(
        package='ros2_pkg', 
        executable='imu_process_node', 
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # [NEW] C++ Low-Level Controller (Chạy IK & Bezier)
    # Thay thế uvc_node cũ
    llc_node = Node(
        package='ros2_pkg', 
        executable='humanoid_llc_node', 
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    # [NEW] Python RL Trainer (Chạy PPO & Safety Shield)
    # Thay thế rl_ppo_training cũ
    rl_train = Node(
        package='ros2_pkg', 
        executable='train_balance_ppo', 
        output='screen', 
        emulate_tty=True,            # <--- THÊM DÒNG NÀY (Để hiện màu log)
        arguments=['--ros-args'],    # Hack nhẹ để node Python hiểu
        prefix=['python3 -u'],       # <--- QUAN TRỌNG: Ép Python in log ngay lập tức (-u: unbuffered)
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        # PHẢI Set các biến môi trường TRƯỚC các node
        set_discovery_range,
        set_cyclonedds_config,
        set_rmw_implementation,
        
        # GPU Config
        set_nvidia_offload, set_nvidia_provider, set_vk_icd, set_ogre_rtshader, set_vblank, set_egl_platform, set_gz_resource,
        
        # Khởi chạy Nodes
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        imu_proc, 
        llc_node,  # Node C++ mới
        rl_train   # Node Python mới
    ])