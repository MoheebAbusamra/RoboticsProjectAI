"""loc_check: measures localization error for every robot.

  where the robot THINKS it is : TF map -> robotN/base_footprint (ICP correction + wheel odometry)
  where it REALLY is           : /robotN/ground_truth from Gazebo, shifted into map coordinates
Once per second (sim time) it records the difference for each robot; every 5 seconds it
prints one line; on Ctrl+C it prints the average and worst error per robot.
Ground truth is used ONLY to measure, never to drive or localize.
  ros2 run factory_bot loc_check --ros-args -p use_sim_time:=true
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LocCheck(Node):
    def __init__(self):
        super().__init__("loc_check")
        self.declare_parameter("robots", ["robot1", "robot2"])
        self.declare_parameter("map_origin_in_gazebo", [1.0, 1.0])
        self.declare_parameter("csv_dir", os.path.expanduser("~/factory_ws/results"))
        self.robots = list(self.get_parameter("robots").value)
        self.origin = self.get_parameter("map_origin_in_gazebo").value

        self.truth = {r: None for r in self.robots}           # latest true (x, y, yaw) in map frame
        self.errors = {r: [] for r in self.robots}            # list of (position m, heading deg)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)   # fills tf_buffer from /tf
        for r in self.robots:
            self.create_subscription(Odometry, f"/{r}/ground_truth",
                                     lambda msg, r=r: self.on_truth(r, msg), 20)
        self.samples = 0
        self.create_timer(1.0, self.sample)

        csv_dir = self.get_parameter("csv_dir").value
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_path = os.path.join(csv_dir, "loc_check_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
        self.csv = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["sim_time_s", "robot", "est_x", "est_y", "est_yaw_deg",
                              "true_x", "true_y", "true_yaw_deg", "pos_err_m", "heading_err_deg"])

    def on_truth(self, robot, msg):
        p = msg.pose.pose
        self.truth[robot] = (p.position.x - self.origin[0], p.position.y - self.origin[1],
                             yaw_of(p.orientation))

    def estimate(self, robot):
        """Latest map -> robot transform, or None if the localizer is not publishing yet."""
        try:
            tf = self.tf_buffer.lookup_transform("map", f"{robot}/base_footprint", Time())
        except TransformException:
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def sample(self):
        self.samples += 1
        t = self.get_clock().now().nanoseconds * 1e-9
        parts = []
        for r in self.robots:
            est, true = self.estimate(r), self.truth[r]
            if est is None or true is None:
                parts.append(f"{r}: waiting for {'map -> ' + r + '/odom' if est is None else 'ground truth'}")
                continue
            pos = math.hypot(est[0] - true[0], est[1] - true[1])
            head = abs(math.degrees(wrap(est[2] - true[2])))
            self.errors[r].append((pos, head))
            self.writer.writerow([f"{t:.1f}", r, f"{est[0]:.3f}", f"{est[1]:.3f}", f"{math.degrees(est[2]):.1f}",
                                  f"{true[0]:.3f}", f"{true[1]:.3f}", f"{math.degrees(true[2]):.1f}",
                                  f"{pos:.3f}", f"{head:.2f}"])
            flag = "   <-- LOST?" if pos > 0.5 else ""
            parts.append(f"{r}: at ({est[0]:5.2f},{est[1]:5.2f}) error {pos:4.2f} m / {head:4.1f} deg{flag}")
        self.csv.flush()
        if self.samples % 5 == 0:
            self.get_logger().info(f"t={t:6.0f}s | " + " | ".join(parts))

    def summary(self):
        lines = ["===== loc_check summary (want: average < 0.10 m and < 3 deg) ====="]
        for r in self.robots:
            e = self.errors[r]
            if not e:
                lines.append(f"{r}: no samples")
                continue
            pos = [p for p, _ in e]
            head = [h for _, h in e]
            lines.append(f"{r}: {len(e)} samples | position average {sum(pos) / len(pos):.3f} m, "
                         f"worst {max(pos):.3f} m | heading average {sum(head) / len(head):.1f} deg, "
                         f"worst {max(head):.1f} deg")
        lines.append(f"CSV log: {self.csv_path}")
        print("\n".join(lines), flush=True)
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = LocCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
