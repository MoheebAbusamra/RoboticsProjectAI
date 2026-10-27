"""fleet_manager: takes delivery requests and runs both robots through them.

Request in:  /delivery_request (std_msgs/String JSON)
               {"pickup": "STORES", "dropoff": "ASSEMBLY_A", "priority": "routine" | "urgent", "id": "J7"}
Emergency:   /emergency_call (std_msgs/String JSON)  {"station": "ASSEMBLY_A", "kit": "TOOL_CRIB"}
               = bring the emergency kit (default from emergency_kit) to that station, NOW:
               it goes to the front of the queue; if no robot is free, the nearest busy robot is
               interrupted (scheduler.pick_robot_to_interrupt). Its job goes back in the queue -
               or, if it is already carrying the cargo, it finishes that delivery after the emergency.
               Emergency legs are driven faster (emergency_speed).
Robots:      sends station names to /robotN/goto and watches /robotN/nav_status (the navigators)
Out:         /fleet/state (String JSON, 1 Hz: robots + queue, for the dashboard later)
             ~/factory_ws/results/jobs.csv (one row per finished job: waiting, driving, total times)

Each robot:  IDLE -> TO_PICKUP -> LOADING (dwell) -> TO_DROPOFF -> UNLOADING (dwell) -> IDLE
             IDLE for return_after_s with nothing to do -> RETURNING (home); a new job can take it
Every second the waiting jobs are given to idle robots by scheduler.assign():
  cost(robot, job) = A* route length from where the robot is to the job's pickup.
"""
import csv
import json
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav2_msgs.srv import LoadMap
from rclpy.parameter import Parameter

from factory_bot.planners import plan
from factory_bot.planning_grid import PlanningGrid
from factory_bot.map_source import resolve_map_yaml
from factory_bot.scheduler import Job, assign, pick_robot_to_interrupt, waiting_in_order

CSV_COLUMNS = ["id", "priority", "pickup", "dropoff", "robot", "state", "allocation", "created", "assigned",
               "picked", "delivered", "wait_s", "to_pickup_s", "to_dropoff_s", "total_s", "distance_m", "attempts",
               "interrupted"]


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        share = get_package_share_directory("factory_bot")
        self.declare_parameter("robots", ["robot1", "robot2"])
        self.declare_parameter("allocation", "greedy")           # or "optimal"
        self.declare_parameter("algorithm", "astar")             # passed on to the navigators
        self.declare_parameter("map_yaml", os.path.join(share, "maps", "factory_map.yaml"))
        self.declare_parameter("start_paused", False)
        self.declare_parameter("dwell_s", 3.0)                   # loading / unloading time
        self.declare_parameter("return_after_s", 10.0)           # idle this long -> go home
        self.declare_parameter("emergency_kit", "TOOL_CRIB")     # where the emergency kit is kept
        self.declare_parameter("emergency_speed", 0.40)          # m/s on emergency legs (normal 0.25)
        self.declare_parameter("results_csv", os.path.expanduser("~/factory_ws/results/jobs.csv"))
        get = lambda n: self.get_parameter(n).value             # noqa: E731
        self.get = get
        map_yaml = resolve_map_yaml(get("map_yaml"))
        self.grid = PlanningGrid.from_files(map_yaml,
                                            os.path.join(share, "config", "planning.yaml"),
                                            os.path.join(share, "config", "zones.yaml"))
        self.map_yaml = map_yaml
        self.map_paused = get("start_paused")
        self.safe_snapshot = None
        self.status_at = {}
        self.map_share = share
        with open(os.path.join(share, "config", "stations.yaml")) as f:
            self.stations = yaml.safe_load(f)["stations"]

        self.jobs, self.counter = [], 0
        self.robots = {}
        for i, r in enumerate(get("robots")):
            self.robots[r] = {"state": "IDLE", "job": None, "status": None, "home": f"HOME_{i + 1}",
                              "target": None, "sent_seq": 0, "until": 0.0, "idle_since": 0.0, "leg_m": 0.0,
                              "suspended": None}          # a delivery put on hold for an emergency
            self.create_subscription(String, f"/{r}/nav_status", lambda m, r=r: self.on_status(r, m), 10)
        self.goto = {r: self.create_publisher(String, f"/{r}/goto", 10) for r in self.robots}
        self.state_pub = self.create_publisher(String, "/fleet/state", 10)
        self.create_subscription(String, "/delivery_request", self.on_request, 10)
        self.create_subscription(String, "/emergency_call", self.on_emergency, 10)
        self.create_service(SetBool, "~/map_pause", self.on_map_pause)
        self.create_service(LoadMap, "~/activate_map", self.on_map_load)
        self.create_timer(0.5, self.step)
        self.create_timer(1.0, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f"fleet manager: robots {list(self.robots)}, allocation {get('allocation')}, map {map_yaml}")

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, robot, msg):
        self.robots[robot]["status"] = json.loads(msg.data)
        self.status_at[robot] = time.monotonic()

    def on_map_pause(self, request, response):
        """Serialize the idle check and dispatch pause in the fleet's own executor."""
        if not request.data:
            self.map_paused = False; self.safe_snapshot = None
            response.success = True; response.message = "Dispatch resumed"
        else:
            try:
                if not self.safe_snapshot:
                    poses, profiles = {}, {}
                    for name, rec in self.robots.items():
                        st = rec["status"] or {}
                        if (rec["state"] != "IDLE" or rec["job"] or rec["suspended"] or
                            time.monotonic()-self.status_at.get(name, -math.inf) > 1.5 or
                            st.get("state") not in ("IDLE", "ARRIVED", "FAILED") or
                            not st.get("stopped") or st.get("aside")):
                            raise ValueError("Wait until both robots are idle and measured stopped")
                        pose = st.get("pose")
                        if not pose or len(pose) != 3 or not all(math.isfinite(v) for v in pose):
                            raise ValueError("Fresh robot map poses are required")
                        poses[name] = pose; profiles[name] = st["defaults"]
                    self.safe_snapshot = {"poses": poses, "profiles": profiles}
                self.map_paused = True
                response.success = True; response.message = json.dumps(self.safe_snapshot)
            except (KeyError, TypeError, ValueError) as error:
                response.success = False; response.message = str(error)
        self.publish_state()
        return response

    def on_map_load(self, request, response):
        try:
            if not self.map_paused or any(r["state"] != "IDLE" or r["job"] for r in self.robots.values()):
                raise ValueError("Pause an idle fleet before changing its map")
            path = resolve_map_yaml(request.map_url)
            grid = PlanningGrid.from_files(path, os.path.join(self.map_share, "config", "planning.yaml"),
                                            os.path.join(self.map_share, "config", "zones.yaml"))
            result = self.set_parameters([Parameter("map_yaml", value=path)])[0]
            if not result.successful: raise ValueError(result.reason)
            self.grid, self.map_yaml = grid, path
            response.result = LoadMap.Response.RESULT_SUCCESS
        except Exception as error:
            response.result = LoadMap.Response.RESULT_INVALID_MAP_DATA
            self.get_logger().error("Map activation rejected: "+str(error))
        self.publish_state()
        return response

    def on_request(self, msg):
        try:
            req = json.loads(msg.data)
            pickup, dropoff = req["pickup"], req["dropoff"]
            priority = req.get("priority", "routine")
            if pickup not in self.stations or dropoff not in self.stations:
                raise ValueError(f"unknown station in {pickup} -> {dropoff}")
            if priority not in ("routine", "urgent"):
                raise ValueError(f"priority must be routine or urgent, not {priority!r}")
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"bad delivery request {msg.data!r}: {e}")
            return
        self.counter += 1
        job = Job(req.get("id", f"J{self.counter:03d}"), pickup, dropoff, priority, created=self.now())
        self.jobs.append(job)
        self.get_logger().info(f"new job {job.id}: {priority} {pickup} -> {dropoff} "
                               f"({len(waiting_in_order(self.jobs))} waiting)")

    def on_emergency(self, msg):
        try:
            req = json.loads(msg.data)
            station, kit = req["station"], req.get("kit", self.get("emergency_kit"))
            if station not in self.stations or kit not in self.stations:
                raise ValueError(f"unknown station in {kit} -> {station}")
        except (ValueError, KeyError) as e:
            self.get_logger().error(f"bad emergency call {msg.data!r}: {e}")
            return
        self.counter += 1
        job = Job(req.get("id", f"E{self.counter:03d}"), kit, station, "emergency", created=self.now())
        self.jobs.append(job)
        self.get_logger().warn(f"EMERGENCY {job.id} at {station}: bring the kit from {kit}")
        self.allocate(self.now())                     # don't wait for the next tick

    # ------------------------------------------------------------------ talking to navigators
    def send(self, robot, station):
        rec = self.robots[robot]
        goal = (rec["status"] or {}).get("goal") or {}
        rec["sent_seq"], rec["target"] = goal.get("seq", 0), station
        request = {"goal": station, "algorithm": self.get("algorithm")}
        request["priority"] = rec["job"].priority if rec["job"] is not None else "routine"
        if rec["job"] is not None and rec["job"].priority == "emergency":
            request["speed"] = self.get("emergency_speed")
        self.goto[robot].publish(String(data=json.dumps(request)))

    def leg_result(self, robot):
        """'ARRIVED', 'FAILED' or None (still going) for the last goal sent to this robot."""
        rec = self.robots[robot]
        status = rec["status"] or {}
        goal = status.get("goal") or {}
        if goal.get("seq", 0) <= rec["sent_seq"] or goal.get("name") != rec["target"]:
            return None
        if status.get("state") in ("ARRIVED", "FAILED"):
            rec["leg_m"] = goal.get("driven_m") or goal.get("path_m") or 0.0
            return status["state"]
        return None

    def pose_of(self, robot):
        status = self.robots[robot]["status"] or {}
        return status.get("pose")

    # ------------------------------------------------------------------ the main loop
    def step(self):
        if getattr(self, "map_paused", False): return
        now = self.now()
        for r, rec in self.robots.items():
            job, state = rec["job"], rec["state"]
            if state == "TO_PICKUP":
                result = self.leg_result(r)
                if result == "ARRIVED":
                    job.distance_m += rec["leg_m"]
                    rec["state"], rec["until"] = "LOADING", now + self.get("dwell_s")
                    self.get_logger().info(f"{r} at {job.pickup}: loading {job.id}")
                elif result == "FAILED":
                    self.leg_failed(r, now)
            elif state == "LOADING" and now >= rec["until"]:
                job.state, job.picked = "PICKED", now
                rec["state"] = "TO_DROPOFF"
                self.publish_state()  # show the loaded prop before starting the delivery leg
                self.send(r, job.dropoff)
            elif state == "TO_DROPOFF":
                result = self.leg_result(r)
                if result == "ARRIVED":
                    job.distance_m += rec["leg_m"]
                    rec["state"], rec["until"] = "UNLOADING", now + self.get("dwell_s")
                    self.get_logger().info(f"{r} at {job.dropoff}: unloading {job.id}")
                elif result == "FAILED":
                    self.leg_failed(r, now)
            elif state == "UNLOADING" and now >= rec["until"]:
                job.state, job.delivered = "DELIVERED", now
                self.publish_state()  # deposit the prop at the end of the existing unload dwell
                self.get_logger().info(f"DELIVERED {job.id} by {r}: waited {job.assigned - job.created:.0f} s, "
                                       f"total {job.delivered - job.created:.0f} s, {job.distance_m:.1f} m")
                if job.priority == "emergency":
                    self.get_logger().warn(f"EMERGENCY {job.id} handled: kit at {job.dropoff} "
                                           f"{job.delivered - job.created:.0f} s after the call")
                self.save(job)
                self.make_idle(r, now)
            elif (state == "IDLE" and now - rec["idle_since"] > self.get("return_after_s")
                  and self.pose_of(r) is not None and not self.at_home(r)):
                rec["state"] = "RETURNING"
                self.send(r, rec["home"])
            elif state == "RETURNING" and self.leg_result(r) in ("ARRIVED", "FAILED"):
                self.make_idle(r, now)
        self.allocate(now)

    def leg_failed(self, robot, now):
        """The navigator gave up (no route / blocked too long): try the job again later, max 3 times."""
        rec = self.robots[robot]
        job = rec["job"]
        job.attempts += 1
        if job.attempts >= 3:
            job.state = "FAILED"
            self.get_logger().error(f"job {job.id} FAILED after {job.attempts} attempts")
            self.save(job)
        elif job.state == "PICKED":                 # already carrying it: try the drop-off again
            self.get_logger().warn(f"{robot} could not reach {job.dropoff} - trying again")
            self.send(robot, job.dropoff)
            return
        else:
            job.state, job.robot = "WAITING", ""      # back in the queue, maybe another robot can
            self.get_logger().warn(f"{robot} could not reach {job.pickup} - {job.id} back in the queue")
        self.make_idle(robot, now)

    def make_idle(self, robot, now):
        """Job finished (or given up). If a delivery was put on hold for an emergency, resume it."""
        rec = self.robots[robot]
        if rec["suspended"]:
            held = rec["suspended"]
            rec.update(job=held, suspended=None, state="TO_DROPOFF")
            self.send(robot, held.dropoff)
            self.get_logger().info(f"{robot} resumes {held.id}: on to {held.dropoff}")
            return
        rec.update(state="IDLE", job=None, idle_since=now)

    def at_home(self, robot):
        pose, home = self.pose_of(robot), self.stations[self.robots[robot]["home"]]
        return pose is not None and math.hypot(pose[0] - home["x"], pose[1] - home["y"]) < 0.3

    def allocate(self, now):
        if getattr(self, "map_paused", False): return
        idle = [r for r, rec in self.robots.items()
                if rec["state"] in ("IDLE", "RETURNING") and self.pose_of(r) is not None]
        if not waiting_in_order(self.jobs):
            return
        costs = {}

        def cost(robot, job):                       # A* route length robot -> pickup (cached per round)
            key = (robot, job.pickup)
            if key not in costs:
                pose, s = self.pose_of(robot), self.stations[job.pickup]
                allow = self.grid.restricted_access(pose[:2], (s["x"], s["y"]))
                result = plan(self.grid, pose[:2], (s["x"], s["y"]), "astar", allow_restricted=allow)
                costs[key] = result.length_m if result.success else math.inf
            return costs[key]

        for robot, job in assign(idle, self.jobs, cost, self.get("allocation")):   # free robots first
            self.start(robot, job, now, cost(robot, job))
        self.interrupt_for_emergencies(now, cost)                                    # then, if needed, busy ones

    def start(self, robot, job, now, metres):
        job.state, job.robot, job.assigned = "ASSIGNED", robot, now
        self.robots[robot].update(state="TO_PICKUP", job=job)
        self.send(robot, job.pickup)
        self.get_logger().info(f"{job.id} ({job.priority} {job.pickup} -> {job.dropoff}) -> {robot}, "
                               f"{metres:.1f} m from the pickup")

    def interrupt_for_emergencies(self, now, cost):
        """Emergencies still waiting (no robot was free): take the best busy robot off its job."""
        for job in waiting_in_order(self.jobs):
            if job.priority != "emergency":
                break                                 # the queue is sorted: no more emergencies
            busy = [r for r, rec in self.robots.items()
                    if rec["state"] in ("TO_PICKUP", "TO_DROPOFF") and rec["job"].priority != "emergency"
                    and self.pose_of(r) is not None]
            robot = pick_robot_to_interrupt(busy, lambda r: cost(r, job),
                                            lambda r: self.robots[r]["job"].state == "PICKED")
            if robot is None:
                return
            rec, old = self.robots[robot], self.robots[robot]["job"]
            old.interrupted += 1
            if old.state == "PICKED":                 # cargo on board: finish it after the emergency
                rec["suspended"] = old
                self.get_logger().warn(f"{robot} interrupted: holds {old.id} (carrying it), "
                                       f"will deliver it after {job.id}")
            else:                                     # not picked up yet: someone else can do it
                old.state, old.robot = "WAITING", ""
                self.get_logger().warn(f"{robot} interrupted: {old.id} goes back in the queue")
            self.start(robot, job, now, cost(robot, job))

    # ------------------------------------------------------------------ outputs
    def save(self, job):
        path = self.get("results_csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        span = lambda a, b: round(b - a, 1) if a is not None and b is not None else None   # noqa: E731
        rnd = lambda t: round(t, 1) if t is not None else None                               # noqa: E731
        row = {"id": job.id, "priority": job.priority, "pickup": job.pickup, "dropoff": job.dropoff,
               "robot": job.robot, "state": job.state, "allocation": self.get("allocation"),
               "created": rnd(job.created), "assigned": rnd(job.assigned), "picked": rnd(job.picked),
               "delivered": rnd(job.delivered), "wait_s": span(job.created, job.assigned),
               "to_pickup_s": span(job.assigned, job.picked), "to_dropoff_s": span(job.picked, job.delivered),
               "total_s": span(job.created, job.delivered), "distance_m": round(job.distance_m, 2),
               "attempts": job.attempts, "interrupted": job.interrupted}
        with open(path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            if new:
                writer.writeheader()
            writer.writerow(row)

    def publish_state(self):
        now = self.now()
        out = {"robots": {r: {"state": rec["state"], "job": rec["job"].id if rec["job"] else None,
                              "priority": rec["job"].priority if rec["job"] else None,
                              "suspended": rec["suspended"].id if rec["suspended"] else None,
                              "target": rec["target"], "pose": self.pose_of(r),
                              "nav": (rec["status"] or {}).get("state")} for r, rec in self.robots.items()},
               "queue": [{"id": j.id, "priority": j.priority, "pickup": j.pickup, "dropoff": j.dropoff,
                          "waiting_s": round(now - j.created, 1)} for j in waiting_in_order(self.jobs)],
               "delivered": sum(j.state == "DELIVERED" for j in self.jobs),
               "map_yaml": self.map_yaml, "map_paused": self.map_paused,
               "failed": sum(j.state == "FAILED" for j in self.jobs),
               # Additive scene metadata only: no cargo policy affects job scheduling/navigation.
               "cargo_jobs": [{"id": j.id, "robot": j.robot, "pickup": j.pickup, "dropoff": j.dropoff,
                               "created": j.created, "state": j.state}
                              for j in ([j for j in self.jobs if j.state in ("ASSIGNED", "PICKED")]
                                        + [j for j in self.jobs if j.state in ("DELIVERED", "FAILED")][-16:])]}
        self.state_pub.publish(String(data=json.dumps(out)))


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
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
