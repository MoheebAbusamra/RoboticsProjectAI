"""mapping_driver: drives robot1 automatically through every aisle so SLAM can
draw the whole factory. For each waypoint: TURN to face it, PAUSE, DRIVE to it, PAUSE.

Position comes from Gazebo's ground truth (/robot1/ground_truth), used ONLY to
steer this scripted drive. SLAM builds the map from the lidar and wheel odometry
and never sees the ground truth. The lidar gives a safety stop.
"""
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def wrap(angle):
    """Keep an angle between -pi and +pi (so 350 degrees becomes -10 degrees)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class MappingDriver(Node):
    def __init__(self):
        super().__init__("mapping_driver")

        # ---- read the route file
        default_route = os.path.join(get_package_share_directory("factory_bot"),
                                     "config", "mapping_route.yaml")
        self.declare_parameter("route", default_route)
        with open(self.get_parameter("route").value, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        self.robot = cfg["robot"]
        self.waypoints = [(float(x), float(y)) for x, y in cfg["waypoints_gazebo"]]
        self.speed = cfg["speed"]
        self.turn_speed = cfg["turn_speed"]
        self.steer_limit = cfg.get("steer_limit", 0.3)        # NEW (v2)
        self.pause = cfg.get("pause_after_turn", 1.0)         # NEW (v2)
        self.tolerance = cfg["tolerance"]
        self.stop_distance = cfg["stop_distance"]

        # ---- what the robot is doing right now
        self.index = 0            # which waypoint we're heading to
        self.mode = "TURN"        # TURN (face the waypoint) or DRIVE (go to it)
        self.pose = None          # (x, y, heading) from Gazebo
        self.front = None         # closest thing in front, from the lidar
        self.pause_until = None   # NEW (v2): simulated time until which we stand still
        self.done = False

        # ---- ROS connections
        self.cmd_pub = self.create_publisher(TwistStamped, f"/{self.robot}/cmd_vel", 10)
        self.create_subscription(Odometry, f"/{self.robot}/ground_truth", self.on_ground_truth, 10)
        self.create_subscription(LaserScan, f"/{self.robot}/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_timer(0.1, self.loop)                     # 10 times per second

        self.get_logger().info(f"driving {self.robot} through {len(self.waypoints)} waypoints")

    # ------------------------------------------------------------------ helpers
    def now(self):                                            # NEW (v2)
        """Simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def start_pause(self):                                    # NEW (v2)
        """Stand still for `pause` seconds (gives SLAM time to catch up)."""
        self.pause_until = self.now() + self.pause

    # ------------------------------------------------------------------ inputs
    def on_ground_truth(self, msg):
        """Remember where Gazebo says the robot really is: x, y and heading."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_scan(self, msg):
        """Remember the closest valid lidar reading within 25 degrees of straight ahead."""
        closest = math.inf
        for i, r in enumerate(msg.ranges):
            angle = wrap(msg.angle_min + i * msg.angle_increment)
            if abs(angle) < math.radians(25) and msg.range_min < r < msg.range_max:
                closest = min(closest, r)
        self.front = closest

    # ------------------------------------------------------------------ output
    def send(self, v, w):
        """Send one speed command: v = forward m/s, w = turning rad/s (left is +)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot}/base_footprint"
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------ brain
    def loop(self):
        """Runs 10 times per second: decide whether to pause, turn, drive or stop."""
        if self.done:
            return
        if self.pose is None or self.front is None:
            self.get_logger().info(f"waiting for {self.robot}'s ground truth and lidar...",
                                   throttle_duration_sec=5.0)
            return

        # 0. Pausing? Keep standing still until the pause is over.     NEW (v2)
        if self.pause_until is not None:
            if self.now() < self.pause_until:
                self.send(0.0, 0.0)
                return
            self.pause_until = None

        x, y, yaw = self.pose
        tx, ty = self.waypoints[self.index]
        distance = math.hypot(tx - x, ty - y)

        # 1. Arrived at this waypoint? Pause, then move on to the next one.
        if distance < self.tolerance:
            self.get_logger().info(f"reached waypoint {self.index + 1}/{len(self.waypoints)}")
            self.index += 1
            self.mode = "TURN"
            self.send(0.0, 0.0)
            self.start_pause()                                           # NEW (v2)
            if self.index == len(self.waypoints):
                self.done = True
                self.get_logger().info("ROUTE FINISHED - press Ctrl+C in the launch terminal to stop the recording")
            return

        error = wrap(math.atan2(ty - y, tx - x) - yaw)   # how far to turn (+ = left)

        # 2. TURN: rotate on the spot until facing the waypoint, then pause
        if self.mode == "TURN":
            if abs(error) < math.radians(4):
                self.mode = "DRIVE"
                self.send(0.0, 0.0)
                self.start_pause()                                       # NEW (v2)
                return
            w = max(0.10, min(self.turn_speed, 1.5 * abs(error)))
            self.send(0.0, math.copysign(w, error))
            return

        # 3. DRIVE: go forward, steering gently to stay on line
        if abs(error) > math.radians(30):
            self.mode = "TURN"
            self.send(0.0, 0.0)
            return
        if self.front < self.stop_distance:
            self.get_logger().warn("something is in front of me: waiting", throttle_duration_sec=2.0)
            self.send(0.0, 0.0)
            return
        v = max(0.06, min(self.speed, 0.6 * distance))
        w = max(-self.steer_limit, min(self.steer_limit, 1.5 * error))   # NEW (v2): limit from file
        self.send(v, w)


def main(args=None):
    rclpy.init(args=args)
    node = MappingDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0.0, 0.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
