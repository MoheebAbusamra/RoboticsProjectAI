"""nav_test: sends one robot on a tour of stations, waits for each goal to finish, then prints a
table and appends the results to ~/factory_ws/results/gazebo_nav.csv (one row per goal).
Run it once per controller/algorithm to compare them on the REAL (Gazebo) robot:
  ros2 run factory_bot nav_test --ros-args -p use_sim_time:=true -p controller:=pid
  ros2 run factory_bot nav_test --ros-args -p use_sim_time:=true -p controller:=pure_pursuit
"""
import csv
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COLUMNS = ["robot", "name", "algorithm", "controller", "result", "plan_ms", "expanded", "path_m", "travel_s",
           "driven_m", "goal_error_m", "heading_error_deg", "cross_track_mean_m", "cross_track_max_m",
                     "min_front_m", "blocked_count", "replans", "replan_ms", "reason"]


class NavTest(Node):
    def __init__(self):
        super().__init__("nav_test")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("controller", "pure_pursuit")
        self.declare_parameter("algorithm", "astar")
        self.declare_parameter("tour", ["TOOL_CRIB", "STORES", "ASSEMBLY_A", "ASSEMBLY_B", "HOME_1"])
        self.declare_parameter("goal_timeout", 240.0)                 # simulated seconds per goal
        get = lambda n: self.get_parameter(n).value                  # noqa: E731
        self.robot, self.controller, self.algorithm = get("robot"), get("controller"), get("algorithm")
        self.tour, self.timeout = list(get("tour")), get("goal_timeout")
        self.status = None                    # latest status from the navigator
        self.next_goal, self.waiting_for, self.sent_at, self.seen_seq = 0, None, 0.0, 0
        self.rows, self.finished = [], False
        self.goal_pub = self.create_publisher(String, f"/{self.robot}/goto", 10)
        self.create_subscription(String, f"/{self.robot}/nav_status", self.on_status, 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f"tour for {self.robot} with {self.algorithm} + {self.controller}: "
                               + " -> ".join(self.tour))

    def on_status(self, msg):
        self.status = json.loads(msg.data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        if self.finished or self.status is None:             # navigator not heard from yet
            return
        goal = self.status.get("goal") or {}
        if self.waiting_for is None:                          # send the next goal
            if self.next_goal >= len(self.tour):
                self.report()
                return
            self.seen_seq = goal.get("seq", 0)
            self.waiting_for = self.tour[self.next_goal]
            self.next_goal += 1
            self.sent_at = self.now()
            self.goal_pub.publish(String(data=json.dumps({"goal": self.waiting_for, "algorithm": self.algorithm,
                                                          "controller": self.controller})))
            self.get_logger().info(f"goal {self.next_goal}/{len(self.tour)}: {self.waiting_for}")
            return
        done = (goal.get("seq", 0) > self.seen_seq and goal.get("name") == self.waiting_for
                and self.status["state"] in ("ARRIVED", "FAILED"))
        if done or self.now() - self.sent_at > self.timeout:
            row = {k: goal.get(k) for k in COLUMNS}
            row.update(robot=self.robot, name=self.waiting_for, algorithm=self.algorithm,
                       controller=self.controller, result=self.status["state"] if done else "TIMEOUT")
            self.rows.append(row)
            self.get_logger().info(f"  {row['result']}: {row['travel_s']} s, goal error {row['goal_error_m']} m, "
                                   f"cross-track mean {row['cross_track_mean_m']} m")
            self.waiting_for = None

    def report(self):
        self.finished = True
        print(f"\n{'goal':11s} {'result':8s} {'path m':>7s} {'time s':>7s} {'goal err':>9s} {'head err':>9s} "
                           f"{'track mean':>11s} {'track max':>10s} {'blocked':>8s} {'replans':>8s}")
        for r in self.rows:
            print(f"{r['name']:11s} {r['result']:8s} {r['path_m'] or 0:7.2f} {r['travel_s'] or 0:7.1f} "
                  f"{r['goal_error_m'] or 0:9.3f} {r['heading_error_deg'] or 0:9.1f} "
                                   f"{r['cross_track_mean_m'] or 0:11.3f} {r['cross_track_max_m'] or 0:10.3f} {r['blocked_count'] or 0:8d} "
                  f"{r['replans'] or 0:8d}")
        out = os.path.expanduser("~/factory_ws/results/gazebo_nav.csv")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        new = not os.path.exists(out)
        with open(out, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=COLUMNS)
            if new:
                writer.writeheader()
            writer.writerows(self.rows)
        print(f"\nappended {len(self.rows)} rows to {out}  - press Ctrl+C", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavTest()
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
