"""odom_test: measures how well wheel odometry counts turns and distance.

The robot (alone, in an empty spot) does three moves by itself:
  1. spins LEFT  two full turns (720 deg)
  2. spins RIGHT two full turns
  3. drives straight 2 m
For each move it compares how far ODOMETRY thinks it went with how far it REALLY
went (Gazebo ground truth) and prints the ratio. 1.000 = perfect.
  ros2 run factory_bot odom_test --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Tracker:
    """Adds up how far one pose source has turned (unwrapped, so 720 deg stays 720)
    and how far it has travelled."""

    def __init__(self):
        self.last = None             # last (x, y, yaw)
        self.turned = 0.0            # radians, + = left
        self.travelled = 0.0         # metres

    def update(self, x, y, yaw):
        if self.last is not None:
            self.turned += wrap(yaw - self.last[2])
            self.travelled += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y, yaw)


class OdomTest(Node):
    # (name, angular speed rad/s, linear speed m/s, how much to do, measured how)
    MOVES = [("left turns", 0.5, 0.0, 4 * math.pi, "turn"),
             ("right turns", -0.5, 0.0, 4 * math.pi, "turn"),
             ("straight", 0.0, 0.2, 2.0, "distance")]

    def __init__(self):
        super().__init__("odom_test")
        self.declare_parameter("robot", "robot1")
        robot = self.get_parameter("robot").value
        self.odom, self.truth = Tracker(), Tracker()
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 50)
        self.create_subscription(Odometry, f"/{robot}/ground_truth", self.on_truth, 50)
        self.pub = self.create_publisher(TwistStamped, f"/{robot}/cmd_vel", 10)
        self.move = -1               # index into MOVES; -1 = waiting to start
        self.pause_until = None      # sim time (s) until which we stand still
        self.start = None            # (odom turned, odom travelled, truth turned, truth travelled)
        self.results = []
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f"odometry test on {robot}: keep the area around the robot clear")

    @staticmethod
    def pose(msg):
        p = msg.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def on_odom(self, msg):
        self.odom.update(*self.pose(msg))

    def on_truth(self, msg):
        self.truth.update(*self.pose(msg))

    def drive(self, angular, linear):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def snapshot(self):
        return (self.odom.turned, self.odom.travelled, self.truth.turned, self.truth.travelled)

    def loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.odom.last is None or self.truth.last is None:
            return                                   # no data yet
        if self.pause_until is None:                 # first time: stand still 2 s
            self.pause_until = now + 2.0
        if now < self.pause_until:
            self.drive(0.0, 0.0)
            return
        if self.move == -1 or self.start is None:    # start the next move
            self.move += 1
            if self.move >= len(self.MOVES):
                self.finish()
                return
            self.start = self.snapshot()
            self.get_logger().info(f"move {self.move + 1}/3: {self.MOVES[self.move][0]}")
        name, ang, lin, amount, kind = self.MOVES[self.move]
        t_t = self.truth.turned - self.start[2]      # how far it REALLY turned in this move
        t_d = self.truth.travelled - self.start[3]   # how far it REALLY travelled in this move
        done = abs(t_t) >= amount if kind == "turn" else t_d >= amount     # judged by the TRUTH
        if not done:
            self.drive(ang, lin)
            return
        self.drive(0.0, 0.0)                         # stop, wait 1.5 s, then record
        self.pause_until = now + 1.5
        self.results.append((name, kind, self.start))
        self.start = None

    def finish(self):
        """Print, for each move, odometry's amount divided by the true amount."""
        self.drive(0.0, 0.0)
        lines = ["===== odometry test ====="]
        turn_ratios = []
        for i, (name, kind, start) in enumerate(self.results):
            # a move ends where the next one starts (after the robot has fully stopped)
            end = self.results[i + 1][2] if i + 1 < len(self.results) else self.snapshot()
            o_t, o_d, t_t, t_d = (a - b for a, b in zip(end, start))
            if kind == "turn":
                ratio = o_t / t_t
                turn_ratios.append(ratio)
                lines.append(f"{name:12s}: truth {math.degrees(t_t):7.1f} deg, odom {math.degrees(o_t):7.1f} deg"
                             f"  -> ratio {ratio:.3f}  (error per 90 deg turn: {90 * (ratio - 1):+.1f} deg)")
            else:
                ratio = o_d / t_d
                lines.append(f"{name:12s}: truth {t_d:7.3f} m,   odom {o_d:7.3f} m    -> ratio {ratio:.3f}")
        avg = sum(turn_ratios) / len(turn_ratios)
        lines.append(f"average turn ratio {avg:.3f}  (good: 0.99 - 1.01)")
        lines.append(f"if a fix is still needed: wheel_separation_multiplier: {avg:.3f}")
        print("\n".join(lines), flush=True)
        self.get_logger().info("test finished - press Ctrl+C")
        self.pause_until = float("inf")              # stand still from now on


def main(args=None):
    rclpy.init(args=args)
    node = OdomTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0.0, 0.0)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
