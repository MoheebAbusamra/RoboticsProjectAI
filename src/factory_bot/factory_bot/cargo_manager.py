"""Draw delivery props using existing fleet milestones and Gazebo pose service.

Kinematic scene props have no contacts or added robot mass. Loading/unloading
timing, allocation, paths, obstacle handling and traffic remain in existing nodes.
"""
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

from factory_bot.cargo_scene import CargoScene


class CargoManager(Node):
    def __init__(self):
        super().__init__('cargo_manager')
        share = get_package_share_directory('factory_bot')
        self.declare_parameter('config', os.path.join(share, 'config', 'cargo.yaml'))
        with open(self.get_parameter('config').value) as stream:
            self.config = yaml.safe_load(stream)
        self.scene = CargoScene(self.config)
        self.truth, self.pending, self.sent = {}, {}, {}
        self.client = self.create_client(SetEntityPose, '/world/'+self.config['world']+'/set_pose')
        self.create_subscription(String, '/fleet/state', self.on_fleet, 10)
        for robot in self.config['robots']:
            self.create_subscription(Odometry, '/'+robot+'/ground_truth',
                                     lambda msg, r=robot: self.on_truth(r, msg), 10)
        self.create_timer(1.0 / self.config['rate_hz'], self.tick)
        self.get_logger().info('Visible cargo ready: replenishing station supplies, two deck slots per robot')

    def on_fleet(self, msg):
        try:
            self.scene.update_jobs(json.loads(msg.data).get('cargo_jobs', []))
        except (ValueError, KeyError, TypeError) as error:
            self.get_logger().warn('Ignored invalid cargo state: '+str(error), throttle_duration_sec=5.)

    def on_truth(self, robot, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.truth[robot] = (p.x, p.y, yaw)

    def completed(self, name, pose, future):
        self.pending.pop(name, None)
        try:
            if future.result().success:
                self.sent[name] = pose
            else:
                self.get_logger().warn('Gazebo could not move cargo '+name, throttle_duration_sec=5.)
        except Exception as error:
            self.get_logger().warn('Cargo pose request failed: '+str(error), throttle_duration_sec=5.)

    def tick(self):
        if not self.client.service_is_ready():
            return
        for robot, pose in self.truth.items():
            self.scene.update_robot_pose(robot, pose)
        # One outstanding request per prop. Carrying props take precedence over
        # occasional stock/deposit updates; no subprocesses or growing command queue.
        names = list(self.scene.carried) + [n for n in self.scene.desired if n not in self.scene.carried]
        issued = 0
        for name in names:
            pose = self.scene.desired[name]
            if name in self.pending or pose == self.sent.get(name):
                continue
            request = SetEntityPose.Request()
            request.entity.name, request.entity.type = name, Entity.MODEL
            x, y, z, yaw = pose
            request.pose.position.x, request.pose.position.y, request.pose.position.z = float(x), float(y), float(z)
            request.pose.orientation.z, request.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            future = self.client.call_async(request)
            self.pending[name] = future
            future.add_done_callback(lambda f, n=name, p=pose: self.completed(n, p, f))
            issued += 1
            if issued >= 6:
                break


def main(args=None):
    rclpy.init(args=args)
    node = CargoManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
