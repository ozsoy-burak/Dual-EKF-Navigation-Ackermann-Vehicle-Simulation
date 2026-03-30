"""
YAML parametreli başlatma dosyası.
config/params.yaml dosyasını kullanır.

Kullanım:
  ros2 launch vehicle_simulator params.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Paket dizinini bul - params.yaml vehicle_simulator paketinin share'inde
    pkg_dir = get_package_share_directory('vehicle_simulator')
    params_file = os.path.join(pkg_dir, '..', '..', '..', '..', 
                               'src', 'ros2_ekf_fusion', 'config', 'params.yaml')
    
    # Alternatif: absolute path kullan
    # params_file = '/path/to/ros2_ekf_fusion/config/params.yaml'

    vehicle_sim_node = Node(
        package='vehicle_simulator',
        executable='vehicle_simulator_node',
        name='vehicle_simulator',
        output='screen',
        parameters=[params_file],
    )

    sensor_pub_node = Node(
        package='sensor_publisher',
        executable='sensor_publisher_node',
        name='sensor_publisher',
        output='screen',
        parameters=[params_file],
    )

    ekf_node = Node(
        package='ekf_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion',
        output='screen',
        parameters=[params_file],
    )

    keyboard_node = Node(
        package='vehicle_simulator',
        executable='keyboard_teleop.py',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -e',
        parameters=[params_file],
    )

    return LaunchDescription([
        vehicle_sim_node,
        sensor_pub_node,
        ekf_node,
        keyboard_node,
    ])
