# Starts the factory in Gazebo, spawns each robot with its controllers,
# bridges sensors, ground truth and the set_pose service, and starts the obstacle mover.

#   ros2 launch factory_bot fleet_sim.launch.py                  # both robots, movers on, window on
#   ros2 launch factory_bot fleet_sim.launch.py gui:=false       # no Gazebo window (much faster)
#   ros2 launch factory_bot fleet_sim.launch.py movers:=false    # forklift and worker parked
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where each robot appears in Gazebo: x, y, yaw (HOME_1 and HOME_2 discs)
SPAWN = {"robot1": (1.0, 1.0, 0.0), "robot2": (2.6, 1.0, 0.0)}


def robot_nodes(name, share, camera_settings):
    """Everything ONE robot needs, all tagged with its name (unchanged)."""
    urdf = xacro.process_file(os.path.join(share, "urdf", "fleet_bot.urdf.xacro"),
                              mappings={"ns": name, **camera_settings}).toxml()
    controllers = os.path.join(share, "config", "fleet_controllers.yaml")
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               namespace=name, output="screen",
               parameters=[{"robot_description": urdf, "use_sim_time": True,
                            "frame_prefix": f"{name}/"}])
    x, y, yaw = SPAWN[name]
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", f"/{name}/robot_description", "-name", name,
                            "-x", str(x), "-y", str(y), "-z", "0.08", "-Y", str(yaw)])
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "-c", f"/{name}/controller_manager",
                          "--param-file", controllers])
    diff = Node(package="controller_manager", executable="spawner", output="screen",
                arguments=["diff_drive_controller", "-c", f"/{name}/controller_manager",
                           "--param-file", controllers,
                           "--controller-ros-args",
                           f"-r /{name}/diff_drive_controller/cmd_vel:=/{name}/cmd_vel "
                           f"-r /{name}/diff_drive_controller/odom:=/{name}/odom"])
    return [rsp, spawn,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[diff]))]


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("factory_bot")
    robots = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",") if r.strip()]
    camera_settings = {n: LaunchConfiguration(n).perform(context)
                       for n in ('cameras', 'camera_width', 'camera_height', 'camera_rate')}
    movers_on = LaunchConfiguration("movers").perform(context) == "true"
    gui_on = LaunchConfiguration("gui").perform(context) == "true"            # NEW (8)
    world = os.path.join(share, "worlds", "factory.sdf")

    # -r = start running; -s = server only (no window), which saves a lot of CPU
    gz_args = f"{world} -r --render-engine-gui ogre" + ("" if gui_on else " -s")                         # NEW (8)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items())

    # Gazebo <-> ROS bridge: clock, set_pose service, and per robot: lidar, IMU, ground truth
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                   "/world/factory/set_pose@ros_gz_interfaces/srv/SetEntityPose"]
    for name in robots:
        bridge_args += [f"/{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{name}/obstacle_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{name}/rear_obstacle_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                        f"/{name}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry"]  # NEW (8)
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=bridge_args, parameters=[{"use_sim_time": True}])

    mover = Node(package="factory_bot", executable="obstacle_mover", output="screen",
                 parameters=[{"use_sim_time": True, "enabled": movers_on}])

    cargo = Node(package="factory_bot", executable="cargo_manager", output="screen",
                 parameters=[{"use_sim_time": True}])

    resources = os.path.join(share, "models")
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    actions = [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join([resources, os.path.dirname(share), existing])),
               gazebo, bridge, mover, cargo]
    if camera_settings['cameras'] == 'true':
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                            name='camera_bridge', output='screen',
                            parameters=[{'use_sim_time': True, 'subscription_heartbeat': 100,
                                         'config_file': os.path.join(share, 'config', 'camera_bridge.yaml')}]))
    for i, name in enumerate(robots):
        actions.append(TimerAction(period=2.0 + 6.0 * i, actions=robot_nodes(name, share, camera_settings)))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="robot1,robot2",
                              description="comma-separated robot names"),
        DeclareLaunchArgument("movers", default_value="true",
                              description="move the forklift and worker (false for mapping)"),
        DeclareLaunchArgument("gui", default_value="true",                  # NEW (8)
                              description="show the Gazebo window (false = faster)"),
        DeclareLaunchArgument("cameras", default_value="true", description="enable both first-person RGB cameras"),
        DeclareLaunchArgument("camera_width", default_value="512"),
        DeclareLaunchArgument("camera_height", default_value="288"),
        DeclareLaunchArgument("camera_rate", default_value="8.0"),
        OpaqueFunction(function=launch_setup),
    ])
