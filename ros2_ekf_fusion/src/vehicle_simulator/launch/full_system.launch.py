"""
Ana başlatma dosyası - tüm düğümleri birlikte başlatır.

Parametreler launch argümanları ile değiştirilebilir:
  ros2 launch vehicle_simulator full_system.launch.py use_gps:=true wheel_linear_std:=0.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    ekf_fusion_share_dir = get_package_share_directory('ekf_fusion')
    rviz_config_path = os.path.join(ekf_fusion_share_dir, 'rviz', 'full.rviz') 

    args = [

        DeclareLaunchArgument('wheelbase',          default_value='2.7'),
        DeclareLaunchArgument('max_speed',          default_value='15.0'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.52'),
        DeclareLaunchArgument('max_acceleration',   default_value='3.0'),
        DeclareLaunchArgument('update_rate',        default_value='50.0'),


        DeclareLaunchArgument('wheel_linear_std',   default_value='0.03'),
        DeclareLaunchArgument('wheel_angular_std',  default_value='0.08'),
        DeclareLaunchArgument('wheel_linear_bias',  default_value='0.005'),
        DeclareLaunchArgument('imu_accel_std',      default_value='0.05'),
        DeclareLaunchArgument('imu_gyro_std',       default_value='0.003'),
        DeclareLaunchArgument('cam_linear_std',     default_value='0.05'),
        DeclareLaunchArgument('cam_angular_std',    default_value='0.02'),
        DeclareLaunchArgument('cam_scale_error',    default_value='0.02'),
        DeclareLaunchArgument('gps_pos_std',        default_value='1.5'),
        DeclareLaunchArgument('gps_multipath_prob', default_value='0.02'),


        DeclareLaunchArgument('use_wheel_odom',     default_value='true'),
        DeclareLaunchArgument('use_camera_odom',    default_value='true'),
        DeclareLaunchArgument('use_gps',            default_value='true'),
        DeclareLaunchArgument('use_imu',            default_value='true'),
        DeclareLaunchArgument('ekf_q_pos',          default_value='0.1'),
        DeclareLaunchArgument('ekf_q_vel',          default_value='0.5'),
        DeclareLaunchArgument('ekf_q_omega',        default_value='0.1'),
        DeclareLaunchArgument('ekf_r_gps',          default_value='2.0'),
        DeclareLaunchArgument('ekf_r_wheel_lin',    default_value='0.05'),
        DeclareLaunchArgument('ekf_r_wheel_ang',    default_value='0.1'),
        DeclareLaunchArgument('ekf_r_cam_lin',      default_value='0.1'),
        DeclareLaunchArgument('ekf_r_cam_ang',      default_value='0.05'),
        DeclareLaunchArgument('ekf_r_imu_gyro',     default_value='0.005'),
    ]


    vehicle_sim_node = Node(
        package='vehicle_simulator',
        executable='vehicle_simulator_node',
        name='vehicle_simulator',
        output='screen',
        parameters=[{
            'wheelbase':          LaunchConfiguration('wheelbase'),
            'max_speed':          LaunchConfiguration('max_speed'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'max_acceleration':   LaunchConfiguration('max_acceleration'),
            'update_rate':        LaunchConfiguration('update_rate'),
        }]
    )


    sensor_pub_node = Node(
        package='sensor_publisher',
        executable='sensor_publisher_node',
        name='sensor_publisher',
        output='screen',
        parameters=[{
            'wheel_odom.rate_hz':          50.0,
            'wheel_odom.linear_x_std':     LaunchConfiguration('wheel_linear_std'),
            'wheel_odom.angular_z_std':    LaunchConfiguration('wheel_angular_std'),
            'wheel_odom.linear_bias':      LaunchConfiguration('wheel_linear_bias'),
            'wheel_odom.drift_rate':       0.001,
            'imu.rate_hz':                 200.0,
            'imu.accel_std':               LaunchConfiguration('imu_accel_std'),
            'imu.gyro_std':                LaunchConfiguration('imu_gyro_std'),
            'imu.accel_bias':              0.02,
            'imu.gyro_bias':               0.001,
            'imu.accel_drift_rate':        0.0001,
            'imu.gyro_drift_rate':         0.00005,
            'camera_odom.rate_hz':         30.0,
            'camera_odom.linear_std':      LaunchConfiguration('cam_linear_std'),
            'camera_odom.angular_std':     LaunchConfiguration('cam_angular_std'),
            'camera_odom.scale_error':     LaunchConfiguration('cam_scale_error'),
            'gps.rate_hz':                 5.0,
            'gps.position_std':            LaunchConfiguration('gps_pos_std'),
            'gps.multipath_std':           5.0,
            'gps.multipath_prob':          LaunchConfiguration('gps_multipath_prob'),
            'gps.origin_lat':              39.9334,
            'gps.origin_lon':              32.8597,
        }]
    )


    ekf_node = Node(
        package='ekf_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion',
        output='screen' )

    nav_node = Node(
        package='navigation',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen' )
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
       )

    keyboard_node = Node(
        package='vehicle_simulator',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',     
        parameters=[{
            'max_speed':    LaunchConfiguration('max_speed'),
            'max_steering': LaunchConfiguration('max_steering_angle'),
            'speed_step':   0.5,
            'steer_step':   0.05,
        }]
    )

    return LaunchDescription(args + [
        LogInfo(msg="=== EKF Füzyon Sistemi başlatılıyor ==="),
        vehicle_sim_node,
        sensor_pub_node,
        ekf_node,
        nav_node,
        rviz_node,
        keyboard_node,
    ])
