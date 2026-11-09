"""Launch the VL53L5CX driver + obstacle/cliff detector, optionally with a viewer.

  ros2 launch vl53l5cx_ros vl53l5cx.launch.py                  # driver + hazard detection
  ros2 launch vl53l5cx_ros vl53l5cx.launch.py hazards:=false   # driver only
  ros2 launch vl53l5cx_ros vl53l5cx.launch.py viewer:=heatmap  # + GUI heatmap
(For the terminal viewer run `ros2 run vl53l5cx_ros tof_matrix_viewer` in its own terminal.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('vl53l5cx_ros'), 'config', 'vl53l5cx.yaml')
    cfg = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('viewer', default_value='none', description='none | heatmap'),
        DeclareLaunchArgument('hazards', default_value='true'),
        Node(package='vl53l5cx_ros', executable='vl53l5cx_node', name='vl53l5cx_node',
             parameters=[cfg], output='screen'),
        Node(package='vl53l5cx_ros', executable='tof_hazard_node', name='tof_hazard_node',
             parameters=[cfg], output='screen',
             condition=IfCondition(LaunchConfiguration('hazards'))),
        Node(package='vl53l5cx_ros', executable='tof_heatmap_viewer', name='tof_heatmap_viewer',
             output='screen',
             condition=IfCondition(EqualsSubstitution(LaunchConfiguration('viewer'), 'heatmap'))),
    ])
