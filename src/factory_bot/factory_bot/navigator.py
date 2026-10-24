"""navigator: drives ONE robot to a goal on the factory map. Run one per robot, in its namespace.

Goal in:   /robotN/goto (std_msgs/String), either
             "STORES"                                   a station name from stations.yaml
             "3.5 2.0 1.57"                             x y [yaw] in metres on the map
             '{"goal": "STORES", "algorithm": "dijkstra", "controller": "pid"}'   JSON with options
Out:       /robotN/cmd_vel (TwistStamped)   /robotN/plan (nav_msgs/Path)
           /robotN/nav_status (String JSON: state, goal, pose ... and, on arrival, the metrics)

States: IDLE -> DRIVING -> ALIGNING (turn to the station's direction) -> ARRIVED
                  |  something closer than stop_distance in front  -> BLOCKED (wait; resume when clear)
                  |  no route / blocked too long                   -> FAILED
Step 14 - dynamic replanning: lidar points that are not on the saved map go into a LiveLayer.
  DRIVING: every 0.5 s, if the next check_ahead_m of the route crosses a live obstacle -> replan now
  BLOCKED: after wait_before_replan seconds -> replan around it (retry every retry_period if no way round)
Pose = TF map -> robotN/base_footprint (ICP correction + wheel odometry, Step 9).
Ground truth is read ONLY to measure how well the robot did (goal error, cross-track), never to drive.
AI-assisted (Claude): cite in report."""
import json
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from factory_bot.controllers import make_follower, wrap
from factory_bot.live_layer import LiveLayer
from factory_bot.planners import path_to_world, plan
from factory_bot.planning_grid import PlanningGrid
from factory_bot.track_sim import cross_track


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        share = get_package_share_directory("factory_bot")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("algorithm", "astar")          # default if the goal doesn't say
        self.declare_parameter("controller", "pure_pursuit")  # default if the goal doesn't say
        self.declare_parameter("map_yaml", os.path.join(share, "maps", "factory_map.yaml"))
        self.declare_parameter("stop_distance", 0.35)         # m from the lidar: obstacle -> stop
        self.declare_parameter("slow_distance", 0.80)         # m: obstacle -> slow down
        self.declare_parameter("front_angle_deg", 35.0)       # +- this much = "in front"
        self.declare_parameter("blocked_timeout", 30.0)       # s blocked with no way round -> give up
        self.declare_parameter("live_memory_s", 2.0)          # s a live obstacle is remembered
        self.declare_parameter("live_range", 3.0)             # m: lidar points further away are ignored
        self.declare_parameter("check_ahead_m", 2.5)          # m of route checked for live obstacles
        self.declare_parameter("wait_before_replan", 2.0)     # s blocked before planning around it
        self.declare_parameter("retry_period", 2.0)           # s between attempts when there's no way round
        self.declare_parameter("map_origin_in_gazebo", [1.0, 1.0])   # for ground-truth metrics only
        get = lambda n: self.get_parameter(n).value          # noqa: E731
        self.get = get
        self.robot = get("robot")

        planning_yaml = os.path.join(share, "config", "planning.yaml")
        self.grid = PlanningGrid.from_files(get("map_yaml"), planning_yaml,
                                            os.path.join(share, "config", "zones.yaml"))
        with open(planning_yaml) as f:
            self.ctrl_settings = (yaml.safe_load(f) or {}).get("controller", {})
        with open(os.path.join(share, "config", "stations.yaml")) as f:
            self.stations = yaml.safe_load(f)["stations"]
        with open(os.path.join(share, "config", "zones.yaml")) as f:
            zones = yaml.safe_load(f)["zones"]
        self.slow_speed = min([z.get("max_speed", 0.15) for z in zones.values() if z["rule"] == "slow"] or [0.15])

        self.live = LiveLayer(self.grid, memory_s=get("live_memory_s"), max_range=get("live_range"))
        self.state, self.goal, self.follower = "IDLE", None, None
        self.seq = 0                              # goal counter, so listeners can tell goals apart
        self.front_min = math.inf                 # nearest lidar reading in front (m)
        self.truth = None                         # true (x, y, yaw) on the map, for metrics only
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        r = self.robot
        self.create_subscription(String, f"/{r}/goto", self.on_goal, 10)
        self.create_subscription(LaserScan, f"/{r}/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{r}/ground_truth", self.on_truth, 20)
        self.loc_ok = True                        # False while ICP says it is lost
        self.create_subscription(Bool, f"/{r}/localization_ok", lambda m: setattr(self, "loc_ok", m.data), 10)
        self.cmd_pub = self.create_publisher(TwistStamped, f"/{r}/cmd_vel", 10)
        self.plan_pub = self.create_publisher(Path, f"/{r}/plan", 10)
        self.status_pub = self.create_publisher(String, f"/{r}/nav_status", 10)
        self.create_timer(0.05, self.loop)        # 20 Hz control
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(f"{r}: navigator ready (send a goal to /{r}/goto)")

    # ------------------------------------------------------------------ inputs
    def on_scan(self, msg):
        """Nearest obstacle within +- front_angle of straight ahead."""
        half = math.radians(self.get("front_angle_deg"))
        best = math.inf
        for i, rng in enumerate(msg.ranges):
            a = wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= half and msg.range_min < rng < msg.range_max:
                best = min(best, rng)
        self.front_min = best
        pose = self.pose()
        if pose is not None:                      # remember obstacles that are not on the map
            self.live.update(pose, msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, self.now())

    def on_truth(self, msg):
        p, o = msg.pose.pose.position, self.get("map_origin_in_gazebo")
        self.truth = (p.x - o[0], p.y - o[1], yaw_of(msg.pose.pose.orientation))
        if self.state in ("DRIVING", "BLOCKED", "ALIGNING") and self.goal:
            self.goal["truth_track"].append(self.truth)

    def pose(self):
        """Where the robot thinks it is: TF map -> robotN/base_footprint, or None."""
        try:
            t = self.tf_buffer.lookup_transform("map", f"{self.robot}/base_footprint", Time()).transform
        except TransformException:
            return None
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def parse_goal(self, text):
        """-> dict(name, x, y, yaw or None, algorithm, controller) or raises ValueError."""
        text = text.strip()
        opts = json.loads(text) if text.startswith("{") else {"goal": text}
        g = str(opts.get("goal", "")).strip()
        if g in self.stations:
            s = self.stations[g]
            x, y, yaw = s["x"], s["y"], s.get("yaw")
        else:
            parts = [float(v) for v in g.replace(",", " ").split()]
            if len(parts) not in (2, 3):
                raise ValueError(f"unknown station or bad coordinates: {g!r}")
            x, y, yaw = parts[0], parts[1], (parts[2] if len(parts) == 3 else None)
        return {"name": g, "x": x, "y": y, "yaw": yaw,
                "algorithm": opts.get("algorithm", self.get("algorithm")),
                "controller": opts.get("controller", self.get("controller"))}

    def on_goal(self, msg):
        try:
            goal = self.parse_goal(msg.data)
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"{self.robot}: bad goal {msg.data!r}: {e}")
            return
        self.seq += 1
        goal["seq"] = self.seq
        pose = self.pose()
        if pose is None:
            self.get_logger().warn(f"{self.robot}: no pose yet (is localization running?) - goal ignored")
            return
        row, col = self.grid.world_to_cell(goal["x"], goal["y"])
        goal["allow"] = self.grid.in_bounds(row, col) and bool(self.grid.restricted[row, col])   # INTO the QC lab
        result = plan(self.live.overlay(self.now()), pose[:2], (goal["x"], goal["y"]), goal["algorithm"],
                      allow_restricted=goal["allow"])
        goal.update(plan_ms=round(result.time_ms, 1), expanded=result.expanded, path_m=round(result.length_m, 2),
                    replans=0, replan_ms=0.0)
        if not result.success:
            self.goal, self.state = goal, "FAILED"
            goal["reason"] = result.reason
            self.get_logger().warn(f"{self.robot}: cannot reach {goal['name']}: {result.reason}")
            self.stop()
            return
        path = path_to_world(self.grid, result.path)
        self.follower = make_follower(goal["controller"], self.ctrl_settings)
        self.follower.set_path(path)
        goal.update(path=path, all_paths=[path], start_time=self.now(), blocked_count=0, blocked_since=None,
                    min_front=math.inf, truth_track=[], next_check=self.now(), retry_at=0.0)
        self.goal, self.state = goal, "DRIVING"
        self.publish_plan(path)
        self.get_logger().info(f"{self.robot}: going to {goal['name']} with {goal['algorithm']} + "
                               f"{goal['controller']}: {result.length_m:.2f} m route, "
                               f"{result.expanded} cells expanded in {result.time_ms:.0f} ms")

    # ------------------------------------------------------------------ control loop
    def loop(self):
        if self.state not in ("DRIVING", "BLOCKED", "ALIGNING"):
            return
        pose = self.pose()
        if pose is None:
            self.stop()
            return
        if not self.loc_ok:                       # don't drive on a pose we don't trust
            self.get_logger().warn(f"{self.robot}: localization lost - stopped until it recovers",
                                   throttle_duration_sec=3.0)
            self.stop()
            return
        g = self.goal
        g["min_front"] = min(g["min_front"], self.front_min)
        if self.state == "ALIGNING":
            self.align(pose)
            return
        v, w, done = self.follower.step(pose, 0.05, speed_limit=self.speed_limit(pose))
        if done:
            self.state = "ALIGNING"
            self.stop()
            return
        now = self.now()
        if v > 0.0 and self.front_min < self.get("stop_distance"):       # something right in front
            if self.state != "BLOCKED":
                self.state, g["blocked_since"] = "BLOCKED", now
                g["blocked_count"] += 1
                self.get_logger().warn(f"{self.robot}: obstacle {self.front_min:.2f} m ahead - waiting")
            elif now - g["blocked_since"] > self.get("blocked_timeout"):
                self.finish("FAILED", reason="blocked too long, no way round")
            elif now - g["blocked_since"] > self.get("wait_before_replan") and now >= g["retry_at"]:
                self.replan("still blocked")             # try to go round it
            self.stop()
            return
        if self.state == "DRIVING" and now >= g["next_check"] and now >= g["retry_at"]:
            g["next_check"] = now + 0.5
            if self.route_blocked(now):
                self.replan("obstacle on the route ahead")
        if self.state == "BLOCKED":
            self.get_logger().info(f"{self.robot}: path clear again - continuing")
            self.state = "DRIVING"
        self.drive(v, w)

    def route_blocked(self, now):
        """Does the next check_ahead_m of the route (from the target point on) cross a live obstacle?"""
        overlay = self.live.overlay(now)
        path, i, travelled = self.follower.path, self.follower.index, 0.0
        while i < len(path) - 1 and travelled < self.get("check_ahead_m"):
            if not overlay.passable(*overlay.world_to_cell(*path[i]), self.goal["allow"]):
                return True
            travelled += math.dist(path[i], path[i + 1])
            i += 1
        return False

    def replan(self, reason):
        """Plan again from where we are, around the live obstacles. True if a new route was found."""
        g, now, pose = self.goal, self.now(), self.pose()
        if pose is None:
            return False
        result = plan(self.live.overlay(now), pose[:2], (g["x"], g["y"]), g["algorithm"], allow_restricted=g["allow"])
        g["replan_ms"] = round(g["replan_ms"] + result.time_ms, 1)
        if not result.success:
            g["retry_at"] = now + self.get("retry_period")
            self.get_logger().warn(f"{self.robot}: {reason}, no way round yet - retrying in "
                                   f"{self.get('retry_period'):.0f} s")
            return False
        path = path_to_world(self.grid, result.path)
        self.follower.set_path(path)
        g.update(path=path, replans=g["replans"] + 1, next_check=now + 1.0)
        g["all_paths"].append(path)
        self.state = "DRIVING"
        self.publish_plan(path)
        self.get_logger().info(f"{self.robot}: {reason} -> replanned ({result.length_m:.2f} m, "
                               f"{result.time_ms:.0f} ms), replan #{g['replans']}")
        return True

    def speed_limit(self, pose):
        """Slower near obstacles and inside the slow zone."""
        limit = None
        if self.front_min < self.get("slow_distance"):
            span = self.get("slow_distance") - self.get("stop_distance")
            frac = max(0.0, (self.front_min - self.get("stop_distance")) / span)
            limit = 0.08 + frac * (self.follower.speed - 0.08)
        row, col = self.grid.world_to_cell(pose[0], pose[1])
        if self.grid.in_bounds(row, col) and self.grid.slow[row, col]:
            limit = self.slow_speed if limit is None else min(limit, self.slow_speed)
        return limit

    def align(self, pose):
        """Turn on the spot to the station's direction (if it has one), then finish."""
        yaw = self.goal["yaw"]
        err = 0.0 if yaw is None else wrap(yaw - pose[2])
        if abs(err) < math.radians(6):
            self.stop()
            self.finish("ARRIVED")
            return
        self.drive(0.0, max(-0.8, min(0.8, 1.5 * err)))

    # ------------------------------------------------------------------ outputs
    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.robot}/base_footprint"
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def stop(self):
        self.drive(0.0, 0.0)

    def finish(self, state, reason=""):
        """End of a goal: compute the metrics (ground truth = measuring only) and report."""
        g = self.goal
        self.state = state
        self.stop()
        g["travel_s"] = round(self.now() - g["start_time"], 1)
        g["reason"] = reason
        track = np.array(g["truth_track"]) if g["truth_track"] else None
        if track is not None and len(track) > 1:
            end = track[-1]
            g["goal_error_m"] = round(math.hypot(end[0] - g["x"], end[1] - g["y"]), 3)
            g["heading_error_deg"] = (round(abs(math.degrees(wrap(g["yaw"] - end[2]))), 1)
                                      if g["yaw"] is not None else None)
            # distance to the nearest of all the routes this goal used (the first one and every replan)
            ct = np.min([cross_track(p, track) for p in g["all_paths"]], axis=0)
            g["cross_track_mean_m"], g["cross_track_max_m"] = round(float(ct.mean()), 3), round(float(ct.max()), 3)
            g["driven_m"] = round(float(np.sum(np.linalg.norm(np.diff(track[:, :2], axis=0), axis=1))), 2)
        g["min_front_m"] = round(g["min_front"], 2) if math.isfinite(g["min_front"]) else None
        self.get_logger().info(f"{self.robot}: {state} {g['name']} in {g['travel_s']} s "
                               f"(goal error {g.get('goal_error_m')} m) {reason}")
        self.publish_status()

    def publish_plan(self, path):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y = x, y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.plan_pub.publish(msg)

    def publish_status(self):
        pose = self.pose()
        out = {"robot": self.robot, "state": self.state,
               "pose": [round(v, 3) for v in pose] if pose else None}
        if self.goal:
            keep = ("seq", "name", "algorithm", "controller", "plan_ms", "expanded", "path_m", "travel_s",
                    "driven_m", "goal_error_m", "heading_error_deg", "cross_track_mean_m",
                    "cross_track_max_m", "min_front_m", "blocked_count", "replans", "replan_ms", "reason")
            out["goal"] = {k: self.goal.get(k) for k in keep if k in self.goal}
        self.status_pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
