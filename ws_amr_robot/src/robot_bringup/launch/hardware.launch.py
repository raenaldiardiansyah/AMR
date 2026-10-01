import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # File ini DEDIKATIF untuk perangkat keras, fusi sensor (EKF), dan Laser Odometry (RF2O).
    # TIDAK ADA algoritma pemetaan (SLAM) atau navigasi di sini.
    
    lidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    
    bridge_node = Node(
        package='robot_bringup',
        executable='stm32_bridge',
        name='stm32_bridge',
        output='screen',
        respawn=True,
        respawn_delay=3.0
    )
    odom_node = Node(package='robot_bringup', executable='odometry_node', name='odometry_node', output='screen')
    
    # REVISI MUTLAK: Transformasi Fisik LiDAR
    # Urutan argumen: [X, Y, Z, Yaw, Pitch, Roll, Parent_Frame, Child_Frame]
    # X = 0.08 meter (8cm ke depan)
    # Z = 0.29 meter (2026-07-21: spacer +12cm added for full 360deg clearance,
    #     was 0.17m -- CONFIRM X/Y unchanged if the spacer also shifted the
    #     mount horizontally, not just vertically)
    LIDAR_X = '0.08'   # + ke depan   (MASIH NILAI LAMA, ukur ulang)
    LIDAR_Y = '0.0'    # + ke kiri
    LIDAR_Z = '0.29'   # tinggi

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # SEBELUM: arguments=['0.08', '0.0', '0.29', '3.14159', '0.0', '0.0', 'base_link', 'laser_frame'],
        arguments=[LIDAR_X, LIDAR_Y, LIDAR_Z, '3.14159', '0.0', '0.0', 'base_link', 'laser_frame'],
        output='screen'
    )

    sllidar_dir = get_package_share_directory('sllidar_ros2')
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={'serial_port': lidar_port, 'frame_id': 'laser_frame'}.items()
    )

    restamper_node = Node(package='robot_bringup', executable='laser_restamper', name='laser_restamper', output='screen')

    # NODE BARU: RF2O Laser Odometry
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        respawn=False,
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0
        }]
    )

    # REKONFIGURASI: Memanggil file ekf.yaml eksternal
    ekf_config_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('odometry/filtered', 'odom')
        ]
    )
    
    twist_mux_config_path = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'twist_mux.yaml'
    )
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_path, {'use_stamped': False}],
        remappings=[('cmd_vel_out', 'cmd_vel')]
    )

    return LaunchDescription([
        bridge_node,
        odom_node,
        tf_node,
        lidar_launch,
        restamper_node,
        rf2o_node,
        ekf_node,
        twist_mux_node
    ])
