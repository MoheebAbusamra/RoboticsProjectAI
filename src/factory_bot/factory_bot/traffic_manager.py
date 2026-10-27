"""traffic_manager: keeps the two robots from blocking each other (rules in traffic.py).

In:   /robotN/nav_status (pose, navigator state)    /robotN/plan (the route it is following)
      /fleet/state (each robot's job priority and fleet state - optional: without it, robot1 simply
      goes before robot2)
Out:  /robotN/traffic (String JSON)  {"cmd": "go" | "wait" | "aside", "because": "robot1",
                                      "rule": "give way", "spot": [x, y]}
      sent when it changes and repeated every second (the navigator carries on if we go silent)
      /traffic/state (String JSON, 1 Hz, for the dashboard)
      ~/factory_ws/results/traffic.csv  one row per event: who waited / made room, for whom, why, how long
"""
import csv
import json
import math
import os
import uuid

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from factory_bot.planning_grid import PlanningGrid
from factory_bot.map_source import resolve_map_yaml
from factory_bot.traffic import MOVING, RobotView, TrafficRules, priority, remaining_route

CSV_COLUMNS = ["time", "robot", "event", "because", "rule", "duration_s", "robots_apart_m", "spot"]


class TrafficManager(Node):
    def __init__(self):
        super().__init__("traffic_manager")
        share = get_package_share_directory("factory_bot")
        self.declare_parameter("robots", ["robot1", "robot2"])
        self.declare_parameter("map_yaml", os.path.join(share, "maps", "factory_map.yaml"))
        self.declare_parameter("lookahead_m", 2.5)       # m of each route compared
        self.declare_parameter("clearance_m", 0.9)       # routes closer than this = conflict
        self.declare_parameter("release_s", 1.0)         # way clear this long before a waiting robot goes
        self.declare_parameter("blocked_s", 2.0)         # blocked this long by a standing robot -> it makes room
        self.declare_parameter("aside_timeout_s", 60.0)  # never keep a robot aside longer than this
        self.declare_parameter("backup_status_timeout", 0.6)
        self.declare_parameter("results_csv", os.path.expanduser("~/factory_ws/results/traffic.csv"))
        get = lambda n: self.get_parameter(n).value     # noqa: E731
        self.get = get
        map_yaml = resolve_map_yaml(get("map_yaml"))
        self.map_yaml = map_yaml; self.instance_id = uuid.uuid4().hex
        grid = PlanningGrid.from_files(map_yaml,
                                       os.path.join(share, "config", "planning.yaml"),
                                       os.path.join(share, "config", "zones.yaml"))
        self.rules = TrafficRules(grid, get("lookahead_m"), get("clearance_m"), get("release_s"),
                                  get("blocked_s"), get("aside_timeout_s"))
        self.robots = list(get("robots"))
        self.status = {r: None for r in self.robots}
        self.received = {r: -math.inf for r in self.robots}
        self.path = {r: [] for r in self.robots}
        self.blocked_since = {r: None for r in self.robots}
        self.fleet = {}
        self.survey_owner = ''
        self.sent, self.last_send = {}, {}
        self.totals = {r: {"waits": 0, "asides": 0, "wait_s": 0.0} for r in self.robots}
        self.closest_m = math.inf
        self.pub = {r: self.create_publisher(String, f"/{r}/traffic", 10) for r in self.robots}
        plan_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        for r in self.robots:
            self.create_subscription(String, f"/{r}/nav_status", lambda m, r=r: self.on_status(r, m), 10)
            self.create_subscription(Path, f"/{r}/plan", lambda m, r=r: self.on_path(r, m), plan_qos)
        self.create_subscription(String, "/fleet/state", self.on_fleet, 10)
        self.create_subscription(String, "/mapping/active_robot", self.on_survey_owner, plan_qos)
        self.state_pub = self.create_publisher(String, "/traffic/state", 10)
        self.create_timer(0.25, self.tick)
        self.create_timer(1.0, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"traffic manager: {self.robots}, map {map_yaml}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, robot, msg):
        self.status[robot] = json.loads(msg.data)
        self.received[robot] = self.now()

    def on_path(self, robot, msg):
        self.path[robot] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_fleet(self, msg):
        self.fleet = json.loads(msg.data).get("robots", {})

    def on_survey_owner(self, msg):
        if msg.data in ('', 'robot1'): self.survey_owner = msg.data

    def view(self, robot, now):
        st = self.status[robot] or {}
        pose = st.get("pose")
        nav = st.get("state")
        if nav == "BLOCKED":
            self.blocked_since[robot] = self.blocked_since[robot] or now
        else:
            self.blocked_since[robot] = None
        active = nav in MOVING
        survey_parked = bool(getattr(self, 'survey_owner', '') and robot != self.survey_owner)
        if survey_parked: active = False
        recovery=st.get('recovery') if nav=='RECOVERING' else None
        route=(recovery.get('path') if recovery else self.path[robot]) or []
        f = self.fleet.get(robot, {})
        return RobotView(robot, tuple(pose) if pose else None, active,
                         remaining_route(route, pose) if active else [],
                         "LOADING" if survey_parked else f.get("state") or "IDLE", f.get("priority"),
                         now - self.blocked_since[robot] if self.blocked_since[robot] else 0.0,
                         recovery,0<=now-self.received[robot]<=self.get('backup_status_timeout'),
                         st.get('stopped',False),st.get('traffic_hold'))

    def tick(self):
        now = self.now()
        views = {r: self.view(r, now) for r in self.robots}
        old = self.rules.cmd
        cmds = self.rules.update(views, now)
        placed = [v for v in views.values() if v.pose is not None]
        apart = math.dist(placed[0].xy, placed[1].xy) if len(placed) == 2 else None
        if apart is not None and apart < self.closest_m:
            self.closest_m = apart
        for r, c in cmds.items():
            prev = old.get(r, {"cmd": "go"})
            if (c["cmd"], c["because"]) != (prev["cmd"], prev.get("because")):
                self.report(r, prev, c, views, now, apart)
            msg = {k: c[k] for k in ("cmd", "because", "rule")}
            msg.update({k:c[k] for k in ('backup_granted','backup_id','backup_hold') if k in c})
            if c["spot"] is not None:
                msg["spot"] = [round(c["spot"][0], 2), round(c["spot"][1], 2)]
            repeat=.25 if any(k in c for k in ('backup_id','backup_hold')) else 1.0
            if msg != self.sent.get(r) or now - self.last_send.get(r, 0.0) >= repeat:
                self.pub[r].publish(String(data=json.dumps(msg)))
                self.sent[r], self.last_send[r] = msg, now

    def report(self, robot, prev, cmd, views, now, apart):
        """Log a change of instruction and write it to the CSV."""
        if prev["cmd"] != "go":                                  # the end of a wait / an aside
            took = now - prev["since"]
            self.totals[robot]["wait_s"] += took
            self.get_logger().info(f"{robot}: done {'waiting' if prev['cmd'] == 'wait' else 'making room'} "
                                   f"for {prev['because']} after {took:.1f} s")
            self.save(now, robot, "end_" + prev["cmd"], prev["because"], prev["rule"], took, apart, None)
        if cmd["cmd"] == "wait":
            self.totals[robot]["waits"] += 1
            other = cmd["because"]
            why = (cmd['rule'] if cmd['rule'].startswith('backup') else
                   f"priority {priority(views[other])} vs {priority(views[robot])}" if cmd["rule"] == "give way"
                   else f"{other} is {views[other].fleet_state.lower()} there")
            self.get_logger().warn(f"{robot} WAITS for {other} - {cmd['rule']} ({why})")
            self.save(now, robot, "wait", other, cmd["rule"], None, apart, None)
        elif cmd["cmd"] == "aside":
            self.totals[robot]["asides"] += 1
            x, y = cmd["spot"]
            self.get_logger().warn(f"{robot} MAKES ROOM for {cmd['because']} ({cmd['rule']}): "
                                   f"moves to ({x:.2f}, {y:.2f})")
            self.save(now, robot, "aside", cmd["because"], cmd["rule"], None, apart, f"{x:.2f} {y:.2f}")

    def save(self, now, robot, event, because, rule, duration, apart, spot):
        path = self.get("results_csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            if new:
                w.writeheader()
            w.writerow({"time": round(now, 1), "robot": robot, "event": event, "because": because, "rule": rule,
                        "duration_s": round(duration, 1) if duration is not None else None,
                        "robots_apart_m": round(apart, 2) if apart is not None else None, "spot": spot})

    def publish_state(self):
        out = {"robots": {r: {"cmd": c["cmd"], "because": c["because"], "rule": c["rule"],
                              "spot": c["spot"],**{k:c[k] for k in ('backup_granted','backup_id','backup_hold') if k in c},
                              **self.totals[r]} for r, c in self.rules.cmd.items()},
               "map_yaml": self.map_yaml, "instance_id": self.instance_id,
               "closest_m": round(self.closest_m, 2) if math.isfinite(self.closest_m) else None}
        self.state_pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficManager()
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
