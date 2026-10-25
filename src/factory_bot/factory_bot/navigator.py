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
Step 17 - traffic rules: /robotN/traffic (String JSON from traffic_manager)
  {"cmd": "go"}                          drive normally
  {"cmd": "wait", "because": "robot1"}   give way: stand still (the goal is kept), continue on "go"
  {"cmd": "aside", "spot": [x, y], ...}  make room: drive to that spot and wait there; on "go",
                                         carry on with the goal we had (or a goal that came meanwhile)
  The manager repeats its instruction every second; if it goes silent for 3 s, we carry on (fail-safe).
Step 14 - dynamic replanning: lidar points that are not on the saved map go into a LiveLayer.
  Normal and emergency missions: a dynamic obstacle within +-20 degrees / 0.8 m -> stop for 3 seconds.
    Recheck a fresh scan: cleared -> resume the same route; still blocked -> replan.
  A replacement path gets a bounded attempt to detour, while close-range stopping stays active.
  Commanded motion without pose progress triggers another stop/replan and has a finite timeout.
  Traffic ASIDE maneuvers retain the existing obstacle policy.
  BLOCKED: after wait_before_replan seconds -> replan around it (retry every retry_period if no way round)
  New goal with no way round what the lidar sees right now (e.g. the other robot standing in a narrow
  aisle) -> keep the goal, stay BLOCKED and retry; it may move (Step 17: the traffic manager makes it move)
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from factory_bot.controllers import make_follower, wrap
from factory_bot.live_layer import LiveLayer
from factory_bot.normal_obstacle_policy import NormalObstaclePolicy
from factory_bot.motion_progress import MotionProgress
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
        self.declare_parameter("normal_front_angle_deg", 20.0)  # half-angle: 40 degrees total
        self.declare_parameter("dynamic_stop_distance", 0.80)   # normal front dynamic-obstacle range
        self.declare_parameter("normal_obstacle_wait", 3.0)     # simulated seconds before rechecking
        self.declare_parameter("obstacle_laser_x", 0.205)      # low front safety lidar, outside own shell
        self.declare_parameter("scan_timeout", 0.6)            # both scan streams must stay fresh
        self.declare_parameter("motion_stall_s", 3.0)          # commanded motion without pose progress
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
        self.goal_request = None                  # the text of the goal we are working on
        self.traffic, self.traffic_until = {"cmd": "go"}, 0.0     # latest traffic instruction (Step 17)
        self.aside, self.resume_request = None, None              # stepping aside: spot, goal to resume
        self.seq = 0                              # goal counter, so listeners can tell goals apart
        self.front_min = math.inf                 # nearest lidar reading in front (m)
        self.front_dynamic_min = math.inf
        self.front_scan_at = -math.inf
        self.normal_obstacle = NormalObstaclePolicy(get("normal_obstacle_wait"))
        self.front_scans = {name: {"min": math.inf, "dynamic": math.inf, "at": -math.inf}
                            for name in ("mapping", "safety")}
        self.detour_active = False
        self.last_command = (0.0, 0.0)
        self.motion_progress = MotionProgress(get("motion_stall_s"), get("blocked_timeout"))
        self.truth = None                         # true (x, y, yaw) on the map, for metrics only
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        r = self.robot
        self.create_subscription(String, f"/{r}/goto", self.on_goal, 10)
        self.create_subscription(LaserScan, f"/{r}/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(LaserScan, f"/{r}/obstacle_scan", self.on_obstacle_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, f"/{r}/ground_truth", self.on_truth, 20)
        self.loc_ok = True                        # False while ICP says it is lost
        self.create_subscription(Bool, f"/{r}/localization_ok", lambda m: setattr(self, "loc_ok", m.data), 10)
        self.create_subscription(String, f"/{r}/traffic", self.on_traffic, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, f"/{r}/cmd_vel", 10)
        # Retain the latest path so a traffic manager joining later can get it.
        plan_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.plan_pub = self.create_publisher(Path, f"/{r}/plan", plan_qos)
        self.status_pub = self.create_publisher(String, f"/{r}/nav_status", 10)
        self.create_timer(0.05, self.loop)        # 20 Hz control
        self.create_timer(0.2, self.publish_status)   # 5 Hz: the traffic manager needs fresh poses
        self.get_logger().info(f"{r}: navigator ready (send a goal to /{r}/goto)")

    # ------------------------------------------------------------------ inputs
    def on_scan(self, msg):
        self.record_scan(msg, "mapping", 0.10)

    def on_obstacle_scan(self, msg):
        self.record_scan(msg, "safety", self.get("obstacle_laser_x"))

    def record_scan(self, msg, source, laser_x):
        """Combine the original map-plane scan with the low front body-height scan."""
        half = math.radians(self.get("normal_front_angle_deg") if self.normal_obstacle_mode()
                            else self.get("front_angle_deg"))
        best = min((rng for i, rng in enumerate(msg.ranges)
                    if abs(wrap(msg.angle_min+i*msg.angle_increment)) <= half
                    and msg.range_min < rng < msg.range_max), default=math.inf)
        pose = self.pose()
        if pose is None:
            return
        dynamic = self.live.front_unmapped_distance(
            pose, msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
            half_angle_deg=self.get("normal_front_angle_deg"),
            max_distance=self.get("dynamic_stop_distance"), laser_x=laser_x)
        self.front_scans[source] = {"min": best, "dynamic": dynamic, "at": self.now()}
        self.front_min = min(scan["min"] for scan in self.front_scans.values())
        self.front_dynamic_min = min(scan["dynamic"] for scan in self.front_scans.values())
        self.front_scan_at = min(scan["at"] for scan in self.front_scans.values())
        self.live.update(pose, msg.ranges, msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max, self.now(), laser_x=laser_x)

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
                "controller": opts.get("controller", self.get("controller")),
                "priority": opts.get("priority", "routine"),
                "speed": opts.get("speed")}                        # e.g. faster for emergencies

    def on_goal(self, msg):
        if self.aside is not None:                # making room for the other robot: do it afterwards
            self.resume_request = msg.data
            self.get_logger().info(f"{self.robot}: new goal noted - will go once the other robot has passed")
            return
        self.goal_request = msg.data
        self.start_goal(msg.data)

    def start_goal(self, text, name=None):
        try:
            goal = self.parse_goal(text)
        except (ValueError, json.JSONDecodeError) as e:
            self.get_logger().error(f"{self.robot}: bad goal {text!r}: {e}")
            return
        if name:
            goal["name"] = name
        self.seq += 1
        goal["seq"] = self.seq
        pose = self.pose()
        if pose is None:
            self.get_logger().warn(f"{self.robot}: no pose yet (is localization running?) - goal ignored")
            return
        goal["allow"] = self.grid.restricted_access(pose[:2], (goal["x"], goal["y"]))
        result = plan(self.live.overlay(self.now()), pose[:2], (goal["x"], goal["y"]), goal["algorithm"],
                      allow_restricted=goal["allow"])
        wait_for_way = False
        if not result.success:                    # blocked by something the lidar sees right now (e.g. the
            normal = plan(self.grid, pose[:2], (goal["x"], goal["y"]), goal["algorithm"],   # other robot)?
                          allow_restricted=goal["allow"])
            if normal.success:                    # keep the goal and wait (BLOCKED) for the way to clear
                self.get_logger().warn(f"{self.robot}: the way to {goal['name']} is blocked right now - "
                                       f"waiting for it to clear")
                result, wait_for_way = normal, True
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
        if goal["speed"]:                         # emergency: drive faster (never above 0.45 m/s)
            self.follower.speed = min(float(goal["speed"]), 0.45)
        self.follower.set_path(path)
        self.normal_obstacle = NormalObstaclePolicy(self.get("normal_obstacle_wait"))
        self.motion_progress.reset()
        self.detour_active = False
        goal.update(path=path, all_paths=[path], start_time=self.now(), blocked_count=0, blocked_since=None,
                    min_front=math.inf, truth_track=[], next_check=self.now(), retry_at=0.0, traffic_wait_s=0.0)
        self.goal, self.state = goal, "DRIVING"
        if wait_for_way:
            goal.update(wait_for_way=True, blocked_since=self.now(), blocked_count=1,
                        retry_at=self.now() + self.get("retry_period"))
            self.state = "BLOCKED"
        self.publish_plan(path)
        self.get_logger().info(f"{self.robot}: going to {goal['name']} with {goal['algorithm']} + "
                               f"{goal['controller']}: {result.length_m:.2f} m route, "
                               f"{result.expanded} cells expanded in {result.time_ms:.0f} ms")

    # ------------------------------------------------------------------ traffic (Step 17)
    def on_traffic(self, msg):
        t = json.loads(msg.data)
        cmd, old = t.get("cmd", "go"), self.traffic.get("cmd", "go")
        self.traffic, self.traffic_until = t, self.now() + 3.0
        if cmd == "aside" and self.aside is None:
            self.step_aside(t["spot"], t.get("because", "?"))
        elif cmd != "aside" and self.aside is not None:
            self.end_aside()
        if cmd == "wait" and old != "wait":
            self.get_logger().info(f"{self.robot}: giving way to {t.get('because')} ({t.get('rule')}) - waiting")
        elif cmd == "go" and old == "wait":
            self.get_logger().info(f"{self.robot}: way clear - continuing")

    def step_aside(self, spot, because):
        """Make room: remember the goal we were on, drive to the free spot and wait there."""
        busy = self.state in ("DRIVING", "BLOCKED", "ALIGNING")
        self.resume_request = self.goal_request if busy else None
        self.aside = (float(spot[0]), float(spot[1]))
        self.get_logger().warn(f"{self.robot}: making room for {because} - moving to "
                               f"({self.aside[0]:.2f}, {self.aside[1]:.2f})")
        self.start_goal(f"{self.aside[0]} {self.aside[1]}", name="ASIDE")

    def end_aside(self):
        """The other robot has passed: carry on with the goal we had (or got meanwhile)."""
        self.aside = None
        if self.resume_request:
            self.goal_request, self.resume_request = self.resume_request, None
            self.get_logger().info(f"{self.robot}: the other robot has passed - back to my goal")
            self.start_goal(self.goal_request)
        elif self.state in ("DRIVING", "BLOCKED", "ALIGNING"):
            self.finish("ARRIVED", reason="room no longer needed")

    def traffic_wait(self):
        """True while the traffic manager tells us to give way (and is still talking to us)."""
        if self.traffic.get("cmd", "go") != "go" and self.now() > self.traffic_until:
            self.get_logger().warn(f"{self.robot}: no word from the traffic manager for 3 s - carrying on")
            self.traffic = {"cmd": "go"}
            if self.aside is not None:
                self.end_aside()
        return self.traffic.get("cmd") == "wait"

    # ------------------------------------------------------------------ control loop
    def normal_obstacle_mode(self):
        """Normal and emergency navigation share the policy; traffic ASIDE moves stay separate."""
        return self.aside is None

    def handle_normal_obstacle(self, now):
        """Hold for three seconds, retry planning, or follow a bounded, guarded detour."""
        self.detour_active = False
        g = self.goal
        blocked = self.front_dynamic_min <= self.get("dynamic_stop_distance") or g.get("motion_stalled", False)
        decision = self.normal_obstacle.decide(blocked, now, self.front_scan_at)
        if decision == "drive":
            if g.pop("dynamic_wait", False):
                self.state = "DRIVING"
                g.update(blocked_since=None, wait_for_way=False)
                self.get_logger().info(f"{self.robot}: front obstacle cleared - resuming the route")
            return False
        if not g.get("dynamic_wait"):
            g.update(dynamic_wait=True, blocked_since=now)
            g["blocked_count"] += 1
            self.get_logger().info(f"{self.robot}: obstacle/stall - stopping for "
                                   f"{self.get('normal_obstacle_wait'):.1f} s before checking again")
        self.state = "BLOCKED"
        if now-g["blocked_since"] > self.get("blocked_timeout"):
            self.finish("FAILED", reason="obstacle did not clear after detour attempts")
            return True
        if decision == "detour":
            # Follow the new path for at most three seconds. The hard proximity
            # guard below still prevents advancing into a close obstacle.
            self.detour_active = True
            self.state = "DRIVING"
            return False
        self.stop()
        if decision == "replan":
            success = self.replan("front obstacle/stall still present after waiting")
            self.normal_obstacle.rerouted(success, now)
            if success:
                g.update(wait_for_way=False, motion_stalled=False)
                # Keep blocked_since: finding a route has not yet cleared the obstacle.
        return True

    def loop(self):
        waiting = self.traffic_wait()
        if self.state not in ("DRIVING", "BLOCKED", "ALIGNING"):
            return
        pose = self.pose()
        if pose is None:
            self.motion_progress.reset()
            self.stop()
            return
        if not self.loc_ok:                       # don't drive on a pose we don't trust
            self.motion_progress.reset()
            self.get_logger().warn(f"{self.robot}: localization lost - stopped until it recovers",
                                   throttle_duration_sec=3.0)
            self.stop()
            return
        g = self.goal
        if waiting:                               # giving way to the other robot: stand still
            self.motion_progress.reset()
            self.stop()
            g["traffic_wait_s"] += 0.05
            if self.state == "BLOCKED":
                g["blocked_since"] = self.now()   # waiting for traffic is not "blocked too long"
            return
        now = self.now()
        if now-self.front_scan_at > self.get("scan_timeout"):
            self.stop()
            self.motion_progress.reset()
            self.get_logger().warn(f"{self.robot}: waiting for fresh mapping and safety scans",
                                   throttle_duration_sec=3.0)
            return
        if self.normal_obstacle_mode():
            progress = self.motion_progress.observe(pose, now, *self.last_command)
            if progress == "failed":
                self.finish("FAILED", reason="commanded movement stayed stalled after recovery attempts")
                return
            if progress == "stalled" and not g.get("motion_stalled"):
                g["motion_stalled"] = True
                self.normal_obstacle.start_wait(now)
                self.get_logger().warn(f"{self.robot}: motion commanded without pose progress - stopping to replan")
        g["min_front"] = min(g["min_front"], self.front_min)
        if self.state != "ALIGNING" and self.normal_obstacle_mode() and self.handle_normal_obstacle(self.now()):
            return
        if g.get("wait_for_way"):                 # the way was blocked when the goal came: stay put, retry
            now = self.now()
            self.stop()
            if now - g["blocked_since"] > self.get("blocked_timeout"):
                self.finish("FAILED", reason="blocked too long, no way round")
            elif now >= g["retry_at"] and self.replan("waiting for the way to clear"):
                g["wait_for_way"] = False
            return
        if self.state == "ALIGNING":
            self.align(pose)
            return
        v, w, done = self.follower.step(pose, 0.05, speed_limit=self.speed_limit(pose))
        if done:
            self.state = "ALIGNING"
            self.stop()
            return
        now = self.now()
        if self.detour_active and self.front_min < self.get("stop_distance"):
            v = 0.0
            self.state = "BLOCKED"  # steering away remains possible at safe standoff
        if v > 0.0 and self.front_min < self.get("stop_distance"):       # something right in front
            if self.state != "BLOCKED":
                self.state = "BLOCKED"
                if g["blocked_since"] is None:
                    g["blocked_since"] = now
                    g["blocked_count"] += 1
                self.get_logger().warn(f"{self.robot}: obstacle {self.front_min:.2f} m ahead - waiting")
            elif now - g["blocked_since"] > self.get("blocked_timeout"):
                self.finish("FAILED", reason="blocked too long, no way round")
            elif now - g["blocked_since"] > (self.get("normal_obstacle_wait") if self.normal_obstacle_mode()
                                             else self.get("wait_before_replan")) and now >= g["retry_at"]:
                self.replan("still blocked")             # try to go round it
            self.stop()
            return
        if (not self.normal_obstacle_mode() and self.state == "DRIVING"
                and now >= g["next_check"] and now >= g["retry_at"]):
            g["next_check"] = now + 0.5
            if self.route_blocked(now):
                self.replan("obstacle on the route ahead")
        if self.state == "BLOCKED" and not self.detour_active:
            self.get_logger().info(f"{self.robot}: path clear again - continuing")
            self.state = "DRIVING"
            g["blocked_since"] = None
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
        self.last_command = (float(v), float(w))
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
               "pose": [round(v, 3) for v in pose] if pose else None,
               "traffic": self.traffic.get("cmd", "go"), "aside": self.aside is not None}
        if self.goal:
            keep = ("seq", "name", "algorithm", "controller", "plan_ms", "expanded", "path_m", "travel_s",
                    "driven_m", "goal_error_m", "heading_error_deg", "cross_track_mean_m",
                    "cross_track_max_m", "min_front_m", "blocked_count", "replans", "replan_ms", "traffic_wait_s",
                    "reason")
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
