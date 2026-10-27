"""mapping_check: measures SLAM's INPUTS while mapping (Lab 3: "check scans, odometry,
the laser-to-base transform and clock first ... tuning SLAM before checking its
inputs makes diagnosis harder").

Every 5 seconds (wall clock) it prints one line and adds one row to a CSV file:
  RTF          simulated seconds per real second (1.0 = real time)
  scan         lidar rate (Hz), % of beams that hit something, nearest/farthest reading
  tilt         how far the robot leans (deg), from the IMU's gravity reading
  odom error   wheel odometry vs Gazebo ground truth: position (m) and heading (deg)
On Ctrl+C it prints a summary with the worst values and what they mean.
Works live (mapping.launch.py) and on a recording (map_from_bag.launch.py).
"""
import csv
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MappingCheck(Node):
    def __init__(self):
        super().__init__("mapping_check")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("start_in_gazebo", [1.0, 1.0])
        self.declare_parameter("csv_dir", os.path.expanduser("~/factory_ws/results"))
        robot = self.get_parameter("robot").value
        self.start = self.get_parameter("start_in_gazebo").value

        self.odom = None                 # latest (x, y, yaw) from wheel odometry
        self.err = None                  # latest (position error, heading error in degrees)
        self.worst = {"pos": 0.0, "head": 0.0, "valid": 100.0, "rtf": 99.0, "hz": 99.0, "tilt": 0.0}
        self.scans = 0
        self.valid_pct = []              # one value per scan in the current 5 s window
        self.tilts = []                  # one value per IMU message in the current 5 s window
        self.near, self.far = 99.0, 0.0
        self.wall = Clock(clock_type=ClockType.STEADY_TIME)     # real time, never simulated
        self.last_wall = self.wall.now()
        self.last_sim = None

        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 50)
        self.create_subscription(Odometry, f"/{robot}/ground_truth", self.on_truth, 50)
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, f"/{robot}/imu", self.on_imu, qos_profile_sensor_data)
        self.create_timer(5.0, self.report, clock=self.wall)

        csv_dir = self.get_parameter("csv_dir").value
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_path = os.path.join(csv_dir, "mapping_check_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")
        self.csv = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["sim_time_s", "rtf", "scan_hz", "hit_pct", "nearest_m", "farthest_m",
                              "tilt_deg", "odom_pos_err_m", "odom_heading_err_deg"])

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_truth(self, msg):
        """Compare the latest odometry with the true pose (both in the map frame)."""
        if self.odom is None:
            return
        p = msg.pose.pose
        tx, ty = p.position.x - self.start[0], p.position.y - self.start[1]
        pos = math.hypot(self.odom[0] - tx, self.odom[1] - ty)
        head = abs(math.degrees(wrap(self.odom[2] - yaw_of(p.orientation))))
        self.err = (pos, head)
        self.worst["pos"] = max(self.worst["pos"], pos)
        self.worst["head"] = max(self.worst["head"], head)

    def on_scan(self, msg):
        n = len(msg.ranges)
        good = [r for r in msg.ranges if math.isfinite(r) and msg.range_min * 1.01 < r < msg.range_max]
        self.scans += 1
        self.valid_pct.append(100.0 * len(good) / max(n, 1))
        if good:
            self.near = min(self.near, min(good))
            self.far = max(self.far, max(good))

    def on_imu(self, msg):
        """Lean angle: at rest the IMU feels gravity straight 'up' its z axis (0 deg).
        If the robot leans, part of gravity shows up on x or y."""
        a = msg.linear_acceleration
        self.tilts.append(math.degrees(math.atan2(math.hypot(a.x, a.y), a.z)))

    def report(self):
        now_wall = self.wall.now()
        now_sim = self.get_clock().now()
        if self.last_sim is None:                     # first call: just remember the times
            self.last_sim, self.last_wall = now_sim, now_wall
            return
        d_wall = (now_wall - self.last_wall).nanoseconds * 1e-9
        d_sim = (now_sim - self.last_sim).nanoseconds * 1e-9
        self.last_sim, self.last_wall = now_sim, now_wall
        if d_sim <= 0.0:
            self.get_logger().info("waiting for the clock (simulation or recording not running yet)")
            return
        rtf = d_sim / d_wall
        hz = self.scans / d_sim
        valid = sum(self.valid_pct) / len(self.valid_pct) if self.valid_pct else 0.0
        tilt = sum(self.tilts) / len(self.tilts) if self.tilts else float("nan")
        pos, head = self.err if self.err else (float("nan"), float("nan"))
        if self.scans > 0 and d_sim > 1.0:           # skip half-empty windows at start/end
            self.worst["rtf"] = min(self.worst["rtf"], rtf)
            self.worst["hz"] = min(self.worst["hz"], hz)
            self.worst["valid"] = min(self.worst["valid"], valid)
            if self.tilts:
                self.worst["tilt"] = max(self.worst["tilt"], tilt)
        warn = []
        if valid < 60.0 or self.far - self.near < 0.5:
            warn.append("LIDAR LOOKS BLIND/BROKEN")
        if tilt > 4.0:
            warn.append("ROBOT TILTED - LIDAR SEES THE FLOOR")
        if head > 5.0:
            warn.append("ODOMETRY HEADING DRIFT")
        if rtf < 0.5:
            warn.append("VM TOO BUSY")
        t = now_sim.nanoseconds * 1e-9
        self.get_logger().info(
            f"t={t:6.0f}s | RTF {rtf:4.2f} | scan {hz:4.1f} Hz, {valid:5.1f}% hit, "
            f"{self.near:4.2f}-{self.far:5.2f} m | tilt {tilt:4.1f} deg | "
            f"odom error {pos:4.2f} m / {head:4.1f} deg "
            f"(worst {self.worst['pos']:4.2f} m / {self.worst['head']:4.1f} deg)"
            + ("   <-- " + ", ".join(warn) if warn else ""))
        self.writer.writerow([f"{t:.1f}", f"{rtf:.3f}", f"{hz:.2f}", f"{valid:.1f}", f"{self.near:.2f}",
                              f"{self.far:.2f}", f"{tilt:.2f}", f"{pos:.3f}", f"{head:.2f}"])
        self.csv.flush()
        self.scans, self.valid_pct, self.tilts = 0, [], []
        self.near, self.far = 99.0, 0.0

    def summary(self):
        w = self.worst
        lines = [
            "===== mapping_check summary =====",
            f"lowest RTF          {w['rtf']:.2f}   (want > 0.7; low = the VM is overloaded)",
            f"lowest scan rate    {w['hz']:.1f} Hz (want about 10)",
            f"lowest beams hitting {w['valid']:.1f} %  (want > 60; very low = lidar rendering problem)",
            f"worst tilt          {w['tilt']:.1f} deg (want < 2; high = robot leaning, lidar sees the floor)",
            f"worst odom position {w['pos']:.2f} m  (grows slowly: that is normal drift)",
            f"worst odom heading  {w['head']:.1f} deg (want < 5; high = wheels slipping in turns)",
            f"CSV log: {self.csv_path}",
        ]
        print("\n".join(lines), flush=True)
        self.csv.close()


def main(args=None):
    rclpy.init(args=args)
    node = MappingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
