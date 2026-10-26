"""Restartable map/localization/navigation/traffic group; no simulation or scheduler."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from factory_bot.map_source import resolve_map_yaml


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('factory_bot')
    get = lambda name: LaunchConfiguration(name).perform(context)
    path = resolve_map_yaml(get('map')); robots = get('robots')
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share, 'launch', 'navigation.launch.py')), launch_arguments={
            name: get(name) for name in ('robots', 'map', 'algorithm', 'controller', 'initial_poses', 'profiles')}.items())]
    if get('traffic').lower() in ('true', '1', 'yes'):
        actions.append(Node(package='factory_bot', executable='traffic_manager', name='traffic_manager',
            output='screen', parameters=[{'use_sim_time': True, 'robots': robots.split(','), 'map_yaml': path}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('factory_bot')
    defaults = {'robots': 'robot1,robot2', 'map': os.path.join(share, 'maps', 'factory_map.yaml'),
                'algorithm': 'astar', 'controller': 'pure_pursuit', 'traffic': 'true',
                'initial_poses': '{}', 'profiles': '{}'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=launch_setup)])
