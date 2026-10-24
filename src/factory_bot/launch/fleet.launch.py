# Step 15: localization + navigators (navigation.launch.py) + the fleet manager.
# Start the simulation first (fleet_sim.launch.py), then:
#   ros2 launch factory_bot fleet.launch.py
#   ros2 launch factory_bot fleet.launch.py allocation:=optimal algorithm:=dijkstra controller:=pid
# Send jobs:  ros2 run factory_bot send_job --demo
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("factory_bot")
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    robots = [r.strip() for r in get("robots").split(",") if r.strip()]
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "navigation.launch.py")),
        launch_arguments={"robots": ",".join(robots), "controller": get("controller"),
                          "algorithm": get("algorithm")}.items())
    manager = Node(package="factory_bot", executable="fleet_manager", name="fleet_manager", output="screen",
                   parameters=[{"use_sim_time": True, "robots": robots, "allocation": get("allocation"),
                                "algorithm": get("algorithm")}])
    return [nav, manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("controller", default_value="pure_pursuit", description="pid or pure_pursuit"),
        DeclareLaunchArgument("algorithm", default_value="astar", description="astar, dijkstra or bfs"),
        DeclareLaunchArgument("allocation", default_value="greedy", description="greedy or optimal"),
        OpaqueFunction(function=launch_setup),
    ])
