# Step 13: localization (ICP, Step 9) + one navigator per robot.
# Start the simulation first (fleet_sim.launch.py), then:
#   ros2 launch factory_bot navigation.launch.py robots:=robot1
#   ros2 launch factory_bot navigation.launch.py robots:=robot1 controller:=pid algorithm:=dijkstra
# Send a goal:  ros2 topic pub --once /robot1/goto std_msgs/msg/String "{data: TOOL_CRIB}"
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from factory_bot.map_source import resolve_map_yaml
from factory_bot.map_catalog import robot_options


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("factory_bot")
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    map_yaml = resolve_map_yaml(get("map"))
    robots = [r.strip() for r in get("robots").split(",") if r.strip()]
    profiles = robot_options(get("profiles"), robots, "profiles")
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "localization.launch.py")),
        launch_arguments={"robots": ",".join(robots), "map": map_yaml, "initial_poses": get("initial_poses")}.items())]
    for r in robots:
        actions.append(Node(package="factory_bot", executable="navigator", name="navigator", namespace=r,
                            output="screen",
                            parameters=[{"use_sim_time": True, "robot": r, "map_yaml": map_yaml,
                                         "controller": profiles.get(r, {}).get("controller", get("controller")),
                                         "algorithm": profiles.get(r, {}).get("algorithm", get("algorithm"))}]))
    return actions


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory("factory_bot"), "maps", "factory_map.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="Saved map YAML shared by map server, ICP and navigators"),
        DeclareLaunchArgument("controller", default_value="pure_pursuit", description="pid or pure_pursuit"),
        DeclareLaunchArgument("algorithm", default_value="astar", description="astar, dijkstra or bfs"),
        DeclareLaunchArgument("initial_poses", default_value="{}"),
        DeclareLaunchArgument("profiles", default_value="{}"),
        OpaqueFunction(function=launch_setup),
    ])
