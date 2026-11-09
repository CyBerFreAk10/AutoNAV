"""Everything needed for autonomous mapping, in one command.

  ros2 launch rover_bringup autonomous_mapping.launch.py               # wait for Start in the dashboard
  ros2 launch rover_bringup autonomous_mapping.launch.py explore:=true # start exploring immediately
  ros2 launch rover_bringup autonomous_mapping.launch.py continue_map:=warehouse   # extend a saved map

Starts: ESP32 bridge, YDLidar, laser odometry, slam_toolbox, explorer, VL53L5CX ToF + hazard
detection, web dashboard (http://<pi-ip>:8080). Uses /dev/rover_esp32 and /dev/rover_lidar
when they exist (ros2 run rover_bringup setup_serial_ports --install), else the old names.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _share(pkg, *p):
    return os.path.join(get_package_share_directory(pkg), *p)


def setup(context):
    a = context.launch_configurations
    tof = a['tof'].lower() == 'true'
    actions = []

    esp_port = a['esp32_port'] or ('/dev/rover_esp32' if os.path.exists('/dev/rover_esp32')
                                   else '/dev/ttyUSB0')
    if not os.path.exists('/dev/rover_esp32') and not a['esp32_port']:
        actions.append(LogInfo(msg='WARNING: /dev/rover_esp32 not set up - using /dev/ttyUSB0. '
                                   'Run: ros2 run rover_bringup setup_serial_ports --install'))
    actions.append(Node(package='rover_control', executable='esp32_bridge', name='esp32_bridge',
                        output='screen',
                        parameters=[_share('rover_control', 'config', 'esp32_bridge.yaml'),
                                    {'port': esp_port}]))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share('rover_nav', 'launch', 'mapping.launch.py')),
        launch_arguments={'explore': a['explore'], 'continue_map': a['continue_map'],
                          'maps_dir': a['maps_dir'], 'map_name': a['map_name'],
                          'use_tof': a['tof'], 'lidar_port': a['lidar_port']}.items()))

    if tof:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_share('vl53l5cx_ros', 'launch', 'vl53l5cx.launch.py'))))

    if a['dashboard'].lower() == 'true':
        actions.append(Node(package='rover_dashboard', executable='dashboard', name='rover_dashboard',
                            output='screen',
                            parameters=[_share('rover_dashboard', 'config', 'dashboard.yaml'),
                                        {'maps_dir': a['maps_dir']}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('explore', default_value='false',
                              description='true = start exploring immediately'),
        DeclareLaunchArgument('continue_map', default_value='',
                              description='extend this saved map instead of starting a new one'),
        DeclareLaunchArgument('map_name', default_value='', description='name for the saved map'),
        DeclareLaunchArgument('maps_dir', default_value='~/maps'),
        DeclareLaunchArgument('tof', default_value='true',
                              description='run the ToF sensor and let the explorer use it'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('esp32_port', default_value='', description='override the ESP32 port'),
        DeclareLaunchArgument('lidar_port', default_value='', description='override the lidar port'),
        OpaqueFunction(function=setup),
    ])
