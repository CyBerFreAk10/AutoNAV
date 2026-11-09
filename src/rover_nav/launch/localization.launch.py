"""Localize in a saved map (no need to know where the robot starts).

  ros2 launch rover_nav localization.launch.py map:=kitchen
  ros2 launch rover_nav localization.launch.py map:=kitchen sim:=true start:=6.5,4.5,2.0

slam_toolbox (localization mode) tracks the pose by scan matching against the saved pose
graph; global_localizer finds the initial pose from scratch and re-localizes if lost.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402


def setup(context):
    a = {k: context.launch_configurations[k] for k in ('map', 'sim', 'world', 'maps_dir', 'start')}
    if not a['map']:
        raise RuntimeError('map:=<name> is required (a map saved in maps_dir)')
    base = os.path.join(os.path.expanduser(a['maps_dir']), a['map'])
    if not os.path.exists(base + '.posegraph'):
        raise RuntimeError(f'{base}.posegraph not found - save maps with the explorer, the '
                           'dashboard or: ros2 service call /slam_toolbox/serialize_map ...')
    start = [float(v) for v in a['start'].split(',')]
    actions = common.sensors(common.as_bool(a['sim']), a['world'], start)
    actions.append(common.odometry())
    actions.append(common.slam(os.path.join(common.CFG, 'slam_localization.yaml'),
                               {'mode': 'localization', 'map_file_name': base,
                                'map_start_pose': [0.0, 0.0, 0.0]}, localization=True))
    actions.append(common.localizer(mode='initialpose', auto_start=True))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('start', default_value='1.0,1.0,0.3'),
        DeclareLaunchArgument('maps_dir', default_value='~/maps'),
        OpaqueFunction(function=setup),
    ])
