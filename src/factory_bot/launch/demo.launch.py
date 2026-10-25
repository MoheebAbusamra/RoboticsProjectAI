"""Start the complete factory fleet, with an optional RViz window."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("factory_bot")

    def include(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
            launch_arguments=arguments.items())

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("movers", default_value="true"),
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("controller", default_value="pure_pursuit"),
        DeclareLaunchArgument("algorithm", default_value="astar"),
        DeclareLaunchArgument("allocation", default_value="greedy"),
        DeclareLaunchArgument("traffic", default_value="true"),
        DeclareLaunchArgument("dashboard", default_value="true"),
        DeclareLaunchArgument("cameras", default_value="true"),
        DeclareLaunchArgument("camera_width", default_value="512"),
        DeclareLaunchArgument("camera_height", default_value="288"),
        DeclareLaunchArgument("camera_rate", default_value="8.0"),
        include("fleet_sim.launch.py", {
            name: LaunchConfiguration(name) for name in ("gui", "movers", "robots", "cameras", "camera_width", "camera_height", "camera_rate")}),
        Node(package="factory_bot", executable="dashboard", output="screen",
             parameters=[{"use_sim_time": True}],
             condition=IfCondition(LaunchConfiguration("dashboard"))),
        TimerAction(period=15.0, actions=[
            include("fleet.launch.py", {
                name: LaunchConfiguration(name)
                for name in ("robots", "controller", "algorithm", "allocation", "traffic")}),
            Node(package="rviz2", executable="rviz2", output="screen",
                 arguments=["-d", os.path.join(share, "rviz", "navigation.rviz")],
                 parameters=[{"use_sim_time": True}],
                 condition=IfCondition(LaunchConfiguration("rviz"))),
        ]),
    ])
