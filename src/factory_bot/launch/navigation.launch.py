# Step 13: localization (ICP, Step 9) + one navigator per robot.
# Start the simulation first (fleet_sim.launch.py), then:
#   ros2 launch factory_bot navigation.launch.py robots:=robot1
#   ros2 launch factory_bot navigation.launch.py robots:=robot1 controller:=pid algorithm:=dijkstra
# Send a goal:  ros2 topic pub --once /robot1/goto std_msgs/msg/String "{data: TOOL_CRIB}"
# AI-assisted (Claude): cite in report.
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
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "localization.launch.py")),
        launch_arguments={"robots": ",".join(robots)}.items())]
    for r in robots:
        actions.append(Node(package="factory_bot", executable="navigator", name="navigator", namespace=r,
                            output="screen",
                            parameters=[{"use_sim_time": True, "robot": r, "controller": get("controller"),
                                         "algorithm": get("algorithm")}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("controller", default_value="pure_pursuit", description="pid or pure_pursuit"),
        DeclareLaunchArgument("algorithm", default_value="astar", description="astar, dijkstra or bfs"),
        OpaqueFunction(function=launch_setup),
    ])
