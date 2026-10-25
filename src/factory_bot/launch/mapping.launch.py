# PHASE 1 of mapping: drive the route once and RECORD everything.
#   robot1 only, movers parked, Gazebo without its window, the auto-driver,
#   a recording (bag), mapping_check (measures SLAM's inputs) and a live OGM map to watch.
# The final map is made afterwards from the recording (map_from_bag.launch.py).
# AI-assisted (Claude): cite in report.
#   ros2 launch factory_bot mapping.launch.py
#   ros2 launch factory_bot mapping.launch.py slam:=true    # DEMO ONLY: live SLAM, recording OFF
#   ros2 launch factory_bot mapping.launch.py rviz:=true    # also open RViz (heavy in the VM)
# Why live SLAM turns the recording off: SLAM publishes map -> robot1/odom on /tf. If that were
# recorded, the replay would contain an old SLAM's transform fighting the new SLAM's.
import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("factory_bot")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "fleet_sim.launch.py")),
        launch_arguments={"robots": "robot1", "movers": "false",
                          "gui": LaunchConfiguration("gazebo_gui")}.items())

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": os.path.join(share, "config", "slam_mapping.yaml"),
                          "use_sim_time": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("slam")))

    ogm_live = Node(package="factory_bot", executable="ogm_mapper", name="ogm_live", output="screen",
                    parameters=[{"use_sim_time": True, "pose_source": "ground_truth",
                                 "map_topic": "/ogm_map_truth"}])

    check = Node(package="factory_bot", executable="mapping_check", output="screen",
                 parameters=[{"use_sim_time": True}])

    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", os.path.join(share, "rviz", "mapping.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")))

    bags = os.path.expanduser("~/factory_ws/bags")
    os.makedirs(bags, exist_ok=True)
    bag_name = os.path.join(bags, "mapping_industrial_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_name, "--topics",
             "/robot1/scan", "/robot1/odom", "/robot1/ground_truth", "/robot1/imu","/tf", "/tf_static"],
        output="screen", condition=UnlessCondition(LaunchConfiguration("slam")))

    driver = Node(package="factory_bot", executable="mapping_driver", output="screen",
                  parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false", description="demo only: live SLAM, no recording"),
        DeclareLaunchArgument("rviz", default_value="false", description="RViz costs a lot of CPU in the VM"),
        simulation,
        TimerAction(period=12.0, actions=[record, check, ogm_live, slam, rviz]),
        TimerAction(period=25.0, actions=[driver]),
    ])
