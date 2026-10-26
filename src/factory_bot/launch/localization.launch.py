# Step 9: both robots keep themselves located on the SAME saved map with ICP (Lab 4).
#   one map_server     -> /map (factory_map from Step 8), for RViz and later steps
#   one icp_localizer per robot, in that robot's namespace -> TF map -> robotN/odom
# Run AFTER the simulation is up; SLAM must NOT be running (two map->odom publishers fight).
#   ros2 launch factory_bot localization.launch.py
#   ros2 launch factory_bot localization.launch.py robots:=robot1
#
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from factory_bot.map_source import resolve_map_yaml
from factory_bot.map_catalog import robot_options

# Where each robot is spawned in GAZEBO (must match SPAWN in fleet_sim.launch.py)
SPAWN = {"robot1": (1.0, 1.0, 0.0), "robot2": (2.6, 1.0, 0.0)}
# The map's (0,0) is where robot1 stood when the map was made (Gazebo 1.0, 1.0, facing +x)
MAP_ORIGIN_IN_GAZEBO = (1.0, 1.0)


def launch_setup(context, *args, **kwargs):
    robots = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",") if r.strip()]
    map_yaml = resolve_map_yaml(LaunchConfiguration("map").perform(context))
    poses = robot_options(LaunchConfiguration("initial_poses").perform(context), robots, "poses")

    actions = [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[{"yaml_filename": map_yaml, "use_sim_time": True,
                          "topic_name": "map", "frame_id": "map"}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_map", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": ["map_server"], "bond_timeout": 0.0}]),
    ]
    for r in robots:
        gx, gy, yaw = SPAWN[r]
        start = poses.get(r, [gx - MAP_ORIGIN_IN_GAZEBO[0], gy - MAP_ORIGIN_IN_GAZEBO[1], yaw])
        actions.append(Node(package="factory_bot", executable="icp_localizer", name="icp_localizer",
                            namespace=r, output="screen",
                            parameters=[{"use_sim_time": True, "robot": r, "map_yaml": map_yaml,
                                         "initial_pose": start}]))
    return actions


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory("factory_bot"), "maps", "factory_map.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2"),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("initial_poses", default_value="{}"),
        OpaqueFunction(function=launch_setup),
    ])
