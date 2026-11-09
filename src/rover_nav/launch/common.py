"""Shared pieces for the rover_nav launch files."""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SHARE = get_package_share_directory('rover_nav')
CFG = os.path.join(SHARE, 'config')


def sensors(sim, world, start):
    """Lidar driver (+ its base_link->laser_frame TF), or the simulator."""
    if sim:
        x, y, yaw = start
        return [
            Node(package='rover_nav', executable='rover_sim', name='rover_sim', output='screen',
                 parameters=[{'world_yaml': world, 'start_x': x, 'start_y': y, 'start_yaw': yaw}]),
            Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_pub_laser',
                 arguments=['--z', '0.02', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        ]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')),
        launch_arguments={'params_file': os.path.join(CFG, 'ydlidar_x4.yaml')}.items())]


def odometry():
    return Node(package='rover_nav', executable='laser_odometry', name='laser_odometry',
                output='screen', parameters=[os.path.join(CFG, 'localizer.yaml')])


def slam(params_file, overrides, localization=False):
    """slam_toolbox via its own launch file, with overrides merged into a temp params file."""
    with open(params_file) as f:
        cfg = yaml.safe_load(f)
    cfg['slam_toolbox']['ros__parameters'].update(overrides)
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix='slam_', delete=False)
    yaml.safe_dump(cfg, tmp)
    tmp.close()
    launch = 'localization_launch.py' if localization else 'online_async_launch.py'
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', launch)),
        launch_arguments={'slam_params_file': tmp.name, 'use_sim_time': 'false'}.items())


def localizer(**params):
    return Node(package='rover_nav', executable='global_localizer', name='global_localizer',
                output='screen', parameters=[os.path.join(CFG, 'localizer.yaml'), params])


def explorer(**params):
    return Node(package='rover_nav', executable='explorer', name='explorer', output='screen',
                parameters=[os.path.join(CFG, 'explorer.yaml'), params])


def as_bool(v):
    return str(v).lower() in ('1', 'true', 'yes', 'on')
