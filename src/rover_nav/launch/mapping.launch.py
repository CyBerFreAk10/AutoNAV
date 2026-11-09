"""Start map recording.

  ros2 launch rover_nav mapping.launch.py                         # new map, drive it yourself
  ros2 launch rover_nav mapping.launch.py explore:=true           # + autonomous boundary-first exploration
  ros2 launch rover_nav mapping.launch.py continue_map:=kitchen   # extend ~/maps/kitchen from ANY start pose
  ros2 launch rover_nav mapping.launch.py sim:=true explore:=true # try it all in the simulator

Explorer is always started; with explore:=false it waits for /explorer/start (or the dashboard).
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, os.path.dirname(__file__))
import common  # noqa: E402


def setup(context):
    a = {k: context.launch_configurations[k] for k in
         ('sim', 'world', 'explore', 'continue_map', 'maps_dir', 'map_name', 'start',
          'use_tof', 'lidar_port')}
    maps_dir = os.path.expanduser(a['maps_dir'])
    start = [float(v) for v in a['start'].split(',')]
    actions = common.sensors(common.as_bool(a['sim']), a['world'], start, a['lidar_port'])
    actions.append(common.odometry())
    actions.append(common.slam(os.path.join(common.CFG, 'slam_mapping.yaml'), {'mode': 'mapping'}))
    actions.append(common.explorer(autostart=common.as_bool(a['explore']), maps_dir=maps_dir,
                                   map_name=a['map_name'], use_tof=common.as_bool(a['use_tof'])))
    if a['continue_map']:
        base = os.path.join(maps_dir, a['continue_map'])
        for ext in ('.yaml', '.posegraph'):
            if not os.path.exists(base + ext):
                raise RuntimeError(f'continue_map: {base + ext} not found')
        # find where we are in the old map, then load its pose graph at that pose
        actions.append(common.localizer(mode='deserialize', map_yaml=base + '.yaml',
                                        posegraph_file=base, auto_start=True, monitor=False))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('world', default_value='', description='sim world map yaml'),
        DeclareLaunchArgument('start', default_value='1.0,1.0,0.3', description='sim start x,y,yaw'),
        DeclareLaunchArgument('explore', default_value='false'),
        DeclareLaunchArgument('continue_map', default_value=''),
        DeclareLaunchArgument('maps_dir', default_value='~/maps'),
        DeclareLaunchArgument('map_name', default_value=''),
        DeclareLaunchArgument('use_tof', default_value='true',
                              description='explorer reacts to ToF cliff/obstacle flags'),
        DeclareLaunchArgument('lidar_port', default_value='',
                              description="override the lidar port ('' = /dev/rover_lidar if set up)"),
        OpaqueFunction(function=setup),
    ])
