import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('rover_control'), 'config', 'esp32_bridge.yaml')
    return LaunchDescription([
        Node(package='rover_control', executable='esp32_bridge', name='esp32_bridge',
             parameters=[params], output='screen'),
    ])
