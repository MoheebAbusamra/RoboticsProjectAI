"""traffic_test: puts the two robots in each other's way on purpose, and measures what happens.
Needs the navigators (navigation.launch.py) and - for the "rules on" run - the traffic manager.
  ros2 run factory_bot traffic_test --ros-args -p use_sim_time:=true -p 'rules:="on"'
  ros2 run factory_bot traffic_test --ros-args -p use_sim_time:=true -p 'rules:="off"'   (manager not running)
Tests (one after the other, each waits until the previous one is over):
  setup           robot1 -> STORES                      (not measured: gets robot1 to the top of the aisle)
  head_on         robot1 -> HOME_1  and robot2 -> STORES at the same time: they meet in the left aisle
  parked_on_goal  robot1 -> STORES while robot2 is standing on it
  same_way        robot1 -> HOME_1  and robot2 -> HOME_2: both down the aisle, one behind the other
Measured per test: time until both are done, how close the robots came (true positions from Gazebo,
centre to centre: under ~0.47 m their bodies touch), how often a robot waited / made room.
Results: printed + appended to ~/factory_ws/results/traffic_test.csv
AI-assisted (Claude): cite in report."""
import csv
import json
import math
import os

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

TESTS = [("setup", {"robot1": "STORES"}),
         ("head_on", {"robot1": "HOME_1", "robot2": "STORES"}),
         ("parked_on_goal", {"robot1": "STORES"}),
         ("same_way", {"robot1": "HOME_1", "robot2": "HOME_2"})]
COLUMNS = ["rules", "test", "result", "time_s", "closest_m", "waits", "wait_s", "asides", "robot1", "robot2"]
ROBOTS = ("robot1", "robot2")


class TrafficTest(Node):
    def __init__(self):
        super().__init__("traffic_test")
        self.declare_parameter("rules", "on")                 # only a label for the CSV
        self.declare_parameter("test_timeout", 240.0)         # simulated seconds per test
        self.rules, self.timeout = self.get_parameter("rules").value, self.get_parameter("test_timeout").value
        self.status = {r: None for r in ROBOTS}
        self.truth = {r: None for r in ROBOTS}
        self.goto = {r: self.create_publisher(String, f"/{r}/goto", 10) for r in ROBOTS}
        for r in ROBOTS:
            self.create_subscription(String, f"/{r}/nav_status", lambda m, r=r: self.on_status(r, m), 10)
            self.create_subscription(Odometry, f"/{r}/ground_truth", lambda m, r=r: self.truth.__setitem__(
                r, (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.index, self.current, self.rows, self.finished, self.skipped = 0, None, [], False, []
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f"traffic test (rules {self.rules}): " + ", ".join(t for t, _ in TESTS))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, robot, msg):
        st = json.loads(msg.data)
        old = self.status[robot] or {}
        c = self.current
        if c is not None:
            if st.get("traffic") == "wait" and old.get("traffic") != "wait":
                c["waits"] += 1
            if st.get("aside") and not old.get("aside"):
                c["asides"] += 1
        self.status[robot] = st

    def quiet(self):
        """Nobody driving, nobody stepping aside: ready for the next test."""
        return all(s is not None and s["state"] not in ("DRIVING", "BLOCKED", "ALIGNING") and not s.get("aside")
                   for s in self.status.values())

    def tick(self):
        if self.finished or any(s is None for s in self.status.values()):
            return
        now, c = self.now(), self.current
        if c is None:                                          # start the next test
            if self.index >= len(TESTS):
                self.report()
                return
            if not self.quiet():
                return
            name, goals = TESTS[self.index]
            self.index += 1
            seqs = {r: ((self.status[r].get("goal") or {}).get("seq", 0)) for r in goals}
            self.current = {"name": name, "goals": goals, "seqs": seqs, "start": now, "closest": math.inf,
                            "waits": 0, "asides": 0, "wait_s": 0.0, "last": now}
            for r, g in goals.items():
                self.goto[r].publish(String(data=json.dumps({"goal": g})))
            self.get_logger().info(f"test {name}: " + ", ".join(f"{r} -> {g}" for r, g in goals.items()))
            return
        if all(self.truth.values()):
            c["closest"] = min(c["closest"], math.dist(self.truth["robot1"], self.truth["robot2"]))
        c["wait_s"] += sum(now - c["last"] for s in self.status.values() if s.get("traffic") == "wait")
        c["last"] = now
        results = {}
        for r, g in c["goals"].items():
            goal = self.status[r].get("goal") or {}
            if goal.get("seq", 0) > c["seqs"][r] and goal.get("name") == g and \
                    self.status[r]["state"] in ("ARRIVED", "FAILED"):
                results[r] = self.status[r]["state"]
        timed_out = now - c["start"] > self.timeout
        if (len(results) == len(c["goals"]) and self.quiet()) or timed_out:
            ok = len(results) == len(c["goals"]) and all(v == "ARRIVED" for v in results.values())
            row = {"rules": self.rules, "test": c["name"], "result": "OK" if ok else ("TIMEOUT" if timed_out else
                   "FAILED"), "time_s": round(now - c["start"], 1),
                   "closest_m": round(c["closest"], 2) if math.isfinite(c["closest"]) else None,
                   "waits": c["waits"], "wait_s": round(c["wait_s"], 1), "asides": c["asides"],
                   "robot1": results.get("robot1", "-" if "robot1" not in c["goals"] else "not done"),
                   "robot2": results.get("robot2", "-" if "robot2" not in c["goals"] else "not done")}
            self.get_logger().info(f"  {row['result']}: {row['time_s']} s, closest {row['closest_m']} m, "
                                   f"{row['waits']} waits, {row['asides']} made room")
            if c["name"] != "setup":
                self.rows.append(row)
            self.current = None
            if timed_out and self.index < len(TESTS):    # robots still stuck: the next tests can't start
                self.get_logger().warn("timed out - robots are stuck, skipping the remaining tests")
                self.skipped = [t for t, _ in TESTS[self.index:]]
                self.index = len(TESTS)

    def report(self):
        self.finished = True
        print(f"\n{'test':15s} {'result':8s} {'time s':>7s} {'closest m':>10s} {'waits':>6s} {'wait s':>7s} "
              f"{'made room':>10s}")
        for r in self.rows:
            print(f"{r['test']:15s} {r['result']:8s} {r['time_s']:7.1f} {r['closest_m'] or 0:10.2f} "
                  f"{r['waits']:6d} {r['wait_s']:7.1f} {r['asides']:10d}")
        if self.skipped:
            print(f"skipped (robots stuck): {', '.join(self.skipped)}")
        out = os.path.expanduser("~/factory_ws/results/traffic_test.csv")
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
    node = TrafficTest()
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
