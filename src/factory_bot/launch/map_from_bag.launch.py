# PHASE 2 of mapping: build the maps from the recording of phase 1. NO Gazebo.
# The recording is played at half speed, so SLAM gets the whole CPU and twice the time
# per scan. Three mappers listen to exactly the same data (Lab 3, stages A, B, C):
#   SLAM Toolbox                 -> /map             final map (factory_map)
#   ogm_mapper, ground-truth pose -> /ogm_map_truth  reference map (A: known poses)
#   ogm_mapper, wheel odometry    -> /ogm_map_odom   drift map     (B: odometry poses)
# AI-assisted (Claude): cite in report.
#   ros2 launch factory_bot map_from_bag.launch.py                  # newest recording
#   ros2 launch factory_bot map_from_bag.launch.py rate:=0.25       # even slower
#   ros2 launch factory_bot map_from_bag.launch.py bag:=$HOME/factory_ws/bags/mapping_2026...
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("factory_bot")
    bag = LaunchConfiguration("bag").perform(context)
    if bag == "latest":                       # newest folder in ~/factory_ws/bags
        found = sorted(glob.glob(os.path.expanduser("~/factory_ws/bags/mapping_industrial_*")))
        if not found:
            return [LogInfo(msg="No industrial-layout recording found - run mapping.launch.py first (older recordings use the old layout)")]
        bag = found[-1]
    rate = LaunchConfiguration("rate").perform(context)

    # robot1's fixed frames (base_footprint -> base_link -> laser_link), same as in the simulation
    description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "fleet_bot.urdf.xacro"), " ns:=robot1"]),
        value_type=str)
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               namespace="robot1", output="screen",
               parameters=[{"robot_description": description, "frame_prefix": "robot1/",
                            "use_sim_time": True}])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": os.path.join(share, "config", "slam_mapping.yaml"),
                          "use_sim_time": "true"}.items())

    ogm_truth = Node(package="factory_bot", executable="ogm_mapper", name="ogm_truth", output="screen",
                     parameters=[{"use_sim_time": True, "pose_source": "ground_truth",
                                  "map_topic": "/ogm_map_truth"}])
    ogm_odom = Node(package="factory_bot", executable="ogm_mapper", name="ogm_odom", output="screen",
                    parameters=[{"use_sim_time": True, "pose_source": "odom",
                                 "map_topic": "/ogm_map_odom"}])
    check = Node(package="factory_bot", executable="mapping_check", output="screen",
                 parameters=[{"use_sim_time": True}])

    play = ExecuteProcess(cmd=["ros2", "bag", "play", bag, "--clock", "100", "--rate", rate],
                          output="screen")
    done = RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[LogInfo(
        msg="RECORDING FINISHED - wait 10 seconds, then save the maps (step 8-fix F)")]))

    return [LogInfo(msg=f"Building maps from {bag} at rate {rate}"),
            rsp, slam, ogm_truth, ogm_odom, check, done,
            TimerAction(period=8.0, actions=[play])]      # give SLAM time to start first


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value="latest"),
        DeclareLaunchArgument("rate", default_value="0.5"),
        OpaqueFunction(function=launch_setup),
    ])
