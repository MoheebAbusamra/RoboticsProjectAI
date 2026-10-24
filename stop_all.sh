#!/bin/bash
# Stops everything left over from earlier runs. Two Gazebos / two clocks at once = robots without
# controllers, SLAM/ICP confused. Asks nicely first (SIGTERM), then forces (SIGKILL) what is left.
PATTERNS='gz sim|gz-sim|ruby.*gz|parameter_bridge|robot_state_publisher|slam_toolbox|rviz2|ros2 bag|'\
'obstacle_mover|mapping_driver|map_viewer|ogm_mapper|mapping_check|icp_localizer|map_server|'\
'lifecycle_manager|loc_check|odom_test|navigator|nav_test|spawner|controller_manager|fleet_manager|send_job|call_emergency'
pkill -f "$PATTERNS"
sleep 3
if pgrep -f "$PATTERNS" > /dev/null; then
  echo "some processes ignored the stop request - forcing them:"
  pgrep -af "$PATTERNS"
  pkill -9 -f "$PATTERNS"
  sleep 1
fi
ros2 daemon stop > /dev/null 2>&1
if pgrep -af "$PATTERNS"; then
  echo "^ still running - run this script again"
else
  echo "clean"
fi
