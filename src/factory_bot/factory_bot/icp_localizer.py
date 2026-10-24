"""icp_localizer: keeps ONE robot located on the saved map with scan-to-map ICP (Lab 4).
Run one per robot, inside the robot's namespace (localization.launch.py does that).

Every 2nd scan:
  1. guess  = last correction (map -> odom) combined with wheel odometry at the scan's time
  2. ICP    = move the guess until the lidar points sit on the map's walls  (icp.py)
  3. check  = enough points matched, small residual, no sudden jump?  yes -> accept
  4. accept = new correction map -> odom = ICP pose combined with the inverse of odometry
Publishes TF map -> robotN/odom (20 times a second), /robotN/icp_pose and /robotN/localization_ok.
If ICP is rejected the robot keeps going on odometry alone until the next good match.
LOST: if ICP is rejected lost_after times in a row (e.g. the robot was pushed, so odometry is far
off), localization_ok becomes false (the navigator stops) and relocalize() searches +-3 m / +-57 deg
around the guess for the pose where the scan fits the walls best. Found -> RECOVERED, else retry.
AI-assisted (Claude): cite in report."""
import collections
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from factory_bot.compare_maps import load_map
from factory_bot.icp import WallLookup, icp_localize, relocalize, scan_to_points


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """Pose a followed by pose b (both (x, y, yaw)): 'b expressed inside a'."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def inverse(a):
    """The pose that undoes a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class IcpLocalizer(Node):
    def __init__(self):
        super().__init__("icp_localizer")
        default_map = os.path.join(get_package_share_directory("factory_bot"), "maps", "factory_map.yaml")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("map_yaml", default_map)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])    # x, y, yaw on the MAP
        self.declare_parameter("every_n_scans", 2)
        self.declare_parameter("laser_x", 0.10)                    # lidar is 10 cm ahead of the centre
        self.declare_parameter("max_pair_distance", 0.30)          # scan point -> wall further = ignored
        self.declare_parameter("min_matched", 0.50)                # accept only if >= 50 % of points match
        self.declare_parameter("max_rms", 0.08)                    # ... and they sit within 8 cm on average
        self.declare_parameter("max_jump", 0.50)                   # ... and the correction is < 0.5 m
        self.declare_parameter("lost_after", 10)                   # rejected this many in a row = lost
        self.declare_parameter("recovery_min_matched", 0.80)       # a recovery must fit this well
        get = lambda n: self.get_parameter(n).value               # noqa: E731
        self.robot = get("robot")
        self.initial = tuple(get("initial_pose"))
        self.every = int(get("every_n_scans"))

        occupied, res, origin = load_map(get("map_yaml"))
        self.walls = WallLookup(occupied, res, origin, max_distance=0.5)
        self.get_logger().info(f"{self.robot}: map loaded, {len(self.walls.walls)} wall cells")

        self.correction = None                    # map -> odom, set when the first odometry arrives
        self.odom_hist = collections.deque(maxlen=100)        # recent (time, x, y, yaw) in odom frame
        self.scans = 0
        self.rejected_in_row = 0                  # ICP failures since the last good match
        self.next_recovery = 0.0                  # sim time of the next allowed recovery search
        self.stats = {"tried": 0, "accepted": 0, "rms": [], "matched": []}

        self.create_subscription(Odometry, f"/{self.robot}/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, f"/{self.robot}/scan", self.on_scan, qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseStamped, f"/{self.robot}/icp_pose", 10)
        self.ok_pub = self.create_publisher(Bool, f"/{self.robot}/localization_ok", 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish_tf)
        self.create_timer(5.0, self.log_stats)

    def on_odom(self, msg):
        p = msg.pose.pose
        odom = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.odom_hist.append((stamp_to_sec(msg.header.stamp),) + odom)
        if self.correction is None:               # first message: robot is at initial_pose on the map
            self.correction = compose(self.initial, inverse(odom))
            self.get_logger().info(f"{self.robot}: starting at map {self.initial}")

    def on_scan(self, msg):
        self.scans += 1
        if self.correction is None or self.scans % self.every:
            return
        t = stamp_to_sec(msg.header.stamp)
        best = min(self.odom_hist, key=lambda o: abs(o[0] - t))
        if abs(best[0] - t) > 0.1:                # no odometry from the same moment: skip
            return
        odom = best[1:]
        guess = compose(self.correction, odom)    # where odometry says we are on the map
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, self.get_parameter("laser_x").value)
        if len(points) < 20:
            return
        pose, info = icp_localize(points, guess, self.walls,
                                  max_distance=self.get_parameter("max_pair_distance").value)
        self.stats["tried"] += 1
        jump = math.hypot(pose[0] - guess[0], pose[1] - guess[1])
        if (info["matched"] >= self.get_parameter("min_matched").value
                and info["rms"] <= self.get_parameter("max_rms").value
                and jump <= self.get_parameter("max_jump").value):
            self.correction = compose(pose, inverse(odom))     # so that correction o odom = pose
            self.stats["accepted"] += 1
            self.stats["rms"].append(info["rms"])
            self.stats["matched"].append(info["matched"])
            self.rejected_in_row = 0
        else:
            pose = guess                          # keep driving on odometry until the next good scan
            self.rejected_in_row += 1
            if self.rejected_in_row >= self.get_parameter("lost_after").value:
                pose = self.recover(points, guess, odom, t) or pose
        self.ok_pub.publish(Bool(data=self.rejected_in_row < self.get_parameter("lost_after").value))
        self.publish_pose(pose, msg.header.stamp)

    def recover(self, points, guess, odom, t):
        """Wide search when lost. Returns the recovered pose, or None (then retry in 3 s)."""
        if t < self.next_recovery:
            return None
        self.get_logger().warn(f"{self.robot}: LOST (ICP failed {self.rejected_in_row} times in a row) "
                               f"- searching around the odometry guess")
        pose, info = relocalize(points, guess, self.walls)
        if info["matched"] >= self.get_parameter("recovery_min_matched").value \
                and info["rms"] <= self.get_parameter("max_rms").value:
            self.correction = compose(pose, inverse(odom))
            self.rejected_in_row = 0
            self.get_logger().warn(f"{self.robot}: RECOVERED - estimate moved "
                                   f"{math.hypot(pose[0] - guess[0], pose[1] - guess[1]):.2f} m and "
                                   f"{math.degrees(abs(pose[2] - guess[2])):.0f} deg "
                                   f"({100 * info['matched']:.0f}% of points on walls)")
            return pose
        self.next_recovery = t + 3.0
        self.get_logger().warn(f"{self.robot}: not found ({100 * info['matched']:.0f}% best fit) - retry in 3 s")
        return None

    def publish_pose(self, pose, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.pose_pub.publish(msg)

    def publish_tf(self):
        """TF map -> robotN/odom = our correction. Everyone else (RViz, loc_check, later the
        navigator) gets map -> robot by chaining it with the wheels' odom -> robot."""
        if self.correction is None:
            return
        x, y, yaw = self.correction
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = f"{self.robot}/odom"
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.tf_pub.sendTransform(tf)

    def log_stats(self):
        if self.correction is None:               # no wheel odometry yet -> no TF -> navigator has no pose
            self.get_logger().warn(f"{self.robot}: waiting for /{self.robot}/odom "
                                   f"(is the robot's diff_drive_controller running?)")
            return
        s = self.stats
        if s["tried"]:
            rms = sum(s["rms"]) / len(s["rms"]) if s["rms"] else float("nan")
            matched = 100 * sum(s["matched"]) / len(s["matched"]) if s["matched"] else 0.0
            self.get_logger().info(f"{self.robot}: ICP accepted {s['accepted']}/{s['tried']} scans, "
                                   f"average fit {rms:.3f} m, {matched:.0f}% of points on walls")
        self.stats = {"tried": 0, "accepted": 0, "rms": [], "matched": []}


def main(args=None):
    rclpy.init(args=args)
    node = IcpLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
