# Step 15-17: localization + navigators (navigation.launch.py) + the fleet manager + the traffic manager.
# Start the simulation first (fleet_sim.launch.py), then:
#   ros2 launch factory_bot fleet.launch.py
#   ros2 launch factory_bot fleet.launch.py allocation:=optimal algorithm:=dijkstra controller:=pid
#   ros2 launch factory_bot fleet.launch.py traffic:=false        (no traffic rules - for comparison)
# Send jobs:  ros2 run factory_bot send_job --demo
# AI-assisted (Claude): cite in report.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from factory_bot.map_source import resolve_map_yaml


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("factory_bot")
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    map_yaml = resolve_map_yaml(get("map"))
    robots = [r.strip() for r in get("robots").split(",") if r.strip()]
    managed = get("managed_maps").lower() in ("true", "1", "yes")
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "navigation.launch.py")),
        launch_arguments={"robots": ",".join(robots), "map": map_yaml, "controller": get("controller"),
                          "algorithm": get("algorithm")}.items())
    manager = Node(package="factory_bot", executable="fleet_manager", name="fleet_manager", output="screen",
                   parameters=[{"use_sim_time": True, "robots": robots, "map_yaml": map_yaml, "start_paused": managed, "allocation": get("allocation"),
                                "algorithm": get("algorithm")}])
    if managed:
        return [manager, Node(package="factory_bot", executable="map_coordinator", name="map_coordinator",
            output="screen", parameters=[{"use_sim_time": True, "robots": robots, "map_yaml": map_yaml,
                "algorithm": get("algorithm"), "controller": get("controller"),
                "traffic": get("traffic").lower() in ("true", "1", "yes")}])]
    actions = [nav, manager]
    if get("traffic").lower() in ("true", "1", "yes"):
        actions.append(Node(package="factory_bot", executable="traffic_manager", name="traffic_manager",
                            output="screen", parameters=[{"use_sim_time": True, "robots": robots, "map_yaml": map_yaml}]))
    return actions


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory("factory_bot"), "maps", "factory_map.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="Saved map YAML shared by allocation, traffic and navigation"),
        DeclareLaunchArgument("controller", default_value="pure_pursuit", description="pid or pure_pursuit"),
        DeclareLaunchArgument("algorithm", default_value="astar", description="astar, dijkstra or bfs"),
        DeclareLaunchArgument("allocation", default_value="greedy", description="greedy or optimal"),
        DeclareLaunchArgument("traffic", default_value="true", description="traffic rules on/off"),
        DeclareLaunchArgument("managed_maps", default_value="false",
                              description="Keep scheduler alive and supervise restartable navigation"),
        OpaqueFunction(function=launch_setup),
    ])
