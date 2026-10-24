"""ogm_mapper: builds the factory map with Occupancy Grid Mapping using KNOWN poses
(the Lab 3 method). Every 2nd lidar scan is added to the grid at the robot's pose
at the moment the scan was taken.

pose_source:
  ground_truth (default) - Gazebo's exact pose: a clean map ("mapping with known poses")
  odom                   - the robot's wheel odometry: shows how odometry drift bends a map
Publishes the map on /ogm_map every second. Save it with:
  ros2 run nav2_map_server map_saver_cli -t /ogm_map -f <file>
"""
import collections
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from factory_bot.ogm import OccupancyGridMapper


def stamp_to_sec(stamp):
    """A ROS time stamp (sec + nanosec) as one number of seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    """Heading (rotation about the vertical axis) from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OgmMapper(Node):
    def __init__(self):
        super().__init__("ogm_mapper")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("pose_source", "ground_truth")      # or "odom"
        self.declare_parameter("start_in_gazebo", [1.0, 1.0])      # map (0,0) = robot1's start
        self.declare_parameter("every_n_scans", 2)                 # use every 2nd scan (5 per second)
        self.declare_parameter("map_topic", "/ogm_map")
        get = lambda name: self.get_parameter(name).value          # noqa: E731
        self.robot = get("robot")
        self.source = get("pose_source")
        self.start = get("start_in_gazebo")
        self.every = int(get("every_n_scans"))

        self.mapper = OccupancyGridMapper()                # 13 x 13 m grid of 5 cm cells
        self.poses = collections.deque(maxlen=100)          # recent (time, x, y, yaw)
        self.scans_seen = 0
        self.scans_used = 0

        pose_topic = f"/{self.robot}/ground_truth" if self.source == "ground_truth" else f"/{self.robot}/odom"
        self.create_subscription(Odometry, pose_topic, self.on_pose, 50)
        self.create_subscription(LaserScan, f"/{self.robot}/scan", self.on_scan, qos_profile_sensor_data)
        # the map is published and KEPT, so late listeners (viewer, map saver) still get it
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, get("map_topic"), qos)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f"OGM mapping {self.robot} with poses from {pose_topic}")

    def on_pose(self, msg):
        """Store the robot's pose in the MAP frame, with its time."""
        p = msg.pose.pose.position
        x, y = p.x, p.y
        if self.source == "ground_truth":          # Gazebo coordinates -> map coordinates
            x, y = x - self.start[0], y - self.start[1]
        self.poses.append((stamp_to_sec(msg.header.stamp), x, y, yaw_of(msg.pose.pose.orientation)))

    def on_scan(self, msg):
        """Add every n-th scan to the map, at the pose closest in time to the scan."""
        self.scans_seen += 1
        if self.scans_seen % self.every or not self.poses:
            return
        t = stamp_to_sec(msg.header.stamp)
        pose = min(self.poses, key=lambda p: abs(p[0] - t))
        if abs(pose[0] - t) > 0.1:                 # no pose close enough in time: skip this scan
            self.get_logger().warn("no pose near this scan's time, skipping", throttle_duration_sec=5.0)
            return
        _, x, y, yaw = pose
        self.mapper.update(x, y, yaw, msg.ranges, msg.angle_min, msg.angle_increment,
                           msg.range_min, msg.range_max)
        self.scans_used += 1

    def publish_map(self):
        """Publish the current grid as a normal ROS map (nav_msgs/OccupancyGrid)."""
        m = self.mapper
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = m.res
        msg.info.width = m.width
        msg.info.height = m.height
        msg.info.origin.position.x = float(m.origin[0])
        msg.info.origin.position.y = float(m.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = m.to_occupancy().reshape(-1).tolist()
        self.pub.publish(msg)
        self.get_logger().info(f"map published ({self.scans_used} scans added)", throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = OgmMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
