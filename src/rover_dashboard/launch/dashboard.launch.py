"""ros2 launch rover_dashboard dashboard.launch.py   -> open http://<pi-ip>:8080"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('rover_dashboard'), 'config', 'dashboard.yaml')
    return LaunchDescription([
        Node(package='rover_dashboard', executable='dashboard', name='rover_dashboard',
             parameters=[params], output='screen'),
    ])
