"""obstacle_mover: moves the forklift and the worker back and forth along their routes, and places
the pallet when asked - through Gazebo's /world/factory/set_pose service (bridged to ROS).

The forklift and worker are "kinematic": Gazebo puts them exactly where we say, and nothing can
push them. So if they walked INTO a robot, Gazebo would shove the robot away (its wheels never
turned, so odometry and ICP would suddenly be wrong). Real drivers and workers stop for a robot,
so these do too: a mover holds still while any robot (true Gazebo position) is within its
give_way distance of where it is about to step, and continues when the robot has moved away.

Parameters: enabled (bool), robots (list), config (path)
Subscribes: /fleet/obstacle_event  {"model": "pallet_1", "place": "main_aisle" | "left_aisle" | "parked"}
                                   {"model": "movers", "place": "on" | "off"}
            /robotN/ground_truth   (only to give way - this is the simulated world, not the robot's brain)
AI-assisted (Claude): cite in report."""
import json
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

from factory_bot.routes import Route


class ObstacleMover(Node):
    def __init__(self):
        super().__init__("obstacle_mover")
        share = get_package_share_directory("factory_bot")
        self.declare_parameter("enabled", True)
        self.declare_parameter("robots", ["robot1", "robot2"])
        self.declare_parameter("config", os.path.join(share, "config", "obstacles.yaml"))
        with open(self.get_parameter("config").value) as f:
            cfg = yaml.safe_load(f)
        self.dt = 1.0 / float(cfg.get("rate_hz", 5.0))
        self.movers = {name: {"route": Route(m["route"], m["speed"]), "z": m["z"], "parked": m["parked"],
                              "give_way": m.get("give_way", 1.0), "progress": 0.0, "waiting": False}
                       for name, m in cfg["movers"].items()}
        self.placeable = cfg.get("placeable", {})
        self.active = bool(self.get_parameter("enabled").value)
        self.robot_xy = {}                                   # robot -> true (x, y) in Gazebo
        for r in self.get_parameter("robots").value:
            self.create_subscription(Odometry, f"/{r}/ground_truth",
                                     lambda msg, r=r: self.robot_xy.__setitem__(
                                         r, (msg.pose.pose.position.x, msg.pose.pose.position.y)), 10)
        self.client = self.create_client(SetEntityPose, f"/world/{cfg.get('world', 'factory')}/set_pose")
        self.create_subscription(String, "/fleet/obstacle_event", self.on_event, 10)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(f"obstacle mover: {list(self.movers)}, enabled={self.active}, "
                               f"they give way to robots")

    def set_pose(self, name, x, y, z, yaw=0.0):
        """Ask Gazebo to put model `name` at (x, y, z, yaw). Fire and forget (no waiting)."""
        if not self.client.service_is_ready():
            self.get_logger().warn("set_pose service not available yet", throttle_duration_sec=5.0)
            return
        req = SetEntityPose.Request()
        req.entity.name, req.entity.type = name, Entity.MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = float(x), float(y), float(z)
        req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.client.call_async(req)

    def robot_near(self, x, y, distance):
        """Name of a robot within `distance` of (x, y), or None."""
        for r, (rx, ry) in self.robot_xy.items():
            if math.hypot(rx - x, ry - y) < distance:
                return r
        return None

    def tick(self):
        if not self.active:
            return
        for name, m in self.movers.items():
            x, y, yaw = m["route"].at(m["progress"] + self.dt)            # where it wants to step next
            blocker = self.robot_near(x, y, m["give_way"])
            if blocker:                                                   # a robot is there: hold still
                if not m["waiting"]:
                    self.get_logger().info(f"{name} stops for {blocker}")
                m["waiting"] = True
                continue
            if m["waiting"]:
                self.get_logger().info(f"{name} moves on")
            m["waiting"] = False
            m["progress"] += self.dt
            self.set_pose(name, x, y, m["z"], yaw)

    def on_event(self, msg):
        ev = json.loads(msg.data)
        name, place = ev.get("model"), ev.get("place")
        if name == "movers":                                              # switch all movers on / off
            self.active = place == "on"
            if not self.active:
                for mover, m in self.movers.items():
                    self.set_pose(mover, m["parked"][0], m["parked"][1], m["z"])
            self.get_logger().info(f"movers {place}")
            return
        spec = self.placeable.get(name)
        if spec is None:
            self.get_logger().error(f"unknown placeable model {name}")
            return
        xy = spec["parked"] if place == "parked" else spec["drop_points"].get(place)
        if xy is None:
            self.get_logger().error(f"unknown place {place} for {name}")
            return
        self.set_pose(name, xy[0], xy[1], spec["z"])
        self.get_logger().info(f"{name} -> {place} {xy}")


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()



def relocalize(scan_points, guess, walls, xy_range=3.0, xy_step=0.25, yaw_range=1.0, yaw_step=0.1, keep=10):
    """'I am lost' recovery (e.g. the robot was pushed): search many poses around `guess`
    (+-xy_range metres, +-yaw_range radians) for the one where the scan best lands on the walls.
      1. quick score for every candidate: fraction of scan points within 0.15 m of a wall
      2. full ICP from the `keep` best candidates; return the best result
    About 0.5-1 s, so it is only used after normal ICP has failed for a while."""
    offsets = np.arange(-xy_range, xy_range + 1e-9, xy_step)
    candidates = []
    for dyaw in np.arange(-yaw_range, yaw_range + 1e-9, yaw_step):
        yaw = guess[2] + dyaw
        turned = transform(scan_points, (0.0, 0.0, yaw))          # rotate once per heading ...
        for dx in offsets:
            for dy in offsets:
                x, y = guess[0] + dx, guess[1] + dy
                moved = turned + np.array([x, y])                 # ... then just shift
                partners, found = walls.lookup(moved)
                near = found & (np.linalg.norm(moved - partners, axis=1) < 0.15)
                candidates.append((near.mean(), (x, y, yaw)))
    candidates.sort(key=lambda c: -c[0])
    best_pose, best_info = guess, {"rms": float("inf"), "matched": 0.0, "iterations": 0}
    for _, start in candidates[:keep]:
        pose, info = icp_localize(scan_points, start, walls)
        if (info["matched"], -info["rms"]) > (best_info["matched"], -best_info["rms"]):
            best_pose, best_info = pose, info
    return best_pose, best_info

if __name__ == "__main__":
    main()
