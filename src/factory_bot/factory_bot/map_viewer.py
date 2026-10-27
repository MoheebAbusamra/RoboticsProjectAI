"""map_viewer: a simple live window showing the map SLAM is drawing and where robot1 is.
Use it instead of RViz's Map display when that display doesn't draw on the VM.
"""
import threading

import numpy as np
import pygame
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

WINDOW = 700  # window size in pixels


class MapListener(Node):
    """The ROS part: listens for the latest map and the robot's position, and remembers them."""

    def __init__(self):
        super().__init__("map_viewer")
        self.declare_parameter("robot", "robot1")
        self.declare_parameter("start_in_gazebo", [1.0, 1.0])   # map (0,0) = robot1's start
        self.declare_parameter("map_topic", "/map")             # which map to show
        self.robot = self.get_parameter("robot").value
        self.start = self.get_parameter("start_in_gazebo").value
        self.map = None          # latest map message
        self.robot_xy = None     # latest robot position, in map coordinates

        # The map is published once in a while and KEPT: ask for the kept copy (Transient Local)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.get_parameter("map_topic").value,
                                 self.on_map, map_qos)
        self.create_subscription(Odometry, f"/{self.robot}/ground_truth", self.on_pose, 10)

    def on_map(self, msg):
        self.map = msg

    def on_pose(self, msg):
        p = msg.pose.pose.position
        # Gazebo coordinates -> map coordinates (the map starts where robot1 started)
        self.robot_xy = (p.x - self.start[0], p.y - self.start[1])


def map_to_surface(msg):
    """Turn a map message into a picture: unknown = grey, free = white, obstacle = black."""
    h, w = msg.info.height, msg.info.width
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)   # -1 unknown, 0 free, 100 obstacle
    rgb = np.full((h, w, 3), 160, dtype=np.uint8)
    rgb[data == 0] = 255
    rgb[data >= 50] = 0
    rgb = np.flipud(rgb)              # map row 0 is the BOTTOM; on screen row 0 is the TOP
    return pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))


def main(args=None):
    rclpy.init(args=args)
    node = MapListener()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()   # ROS listens in the background

    pygame.init()
    screen = pygame.display.set_mode((WINDOW, WINDOW))
    pygame.display.set_caption("Live SLAM map")
    font = pygame.font.SysFont(None, 22)
    clock = pygame.time.Clock()

    running = True
    while running and rclpy.ok():
        for event in pygame.event.get():          # closing the window ends the program
            if event.type == pygame.QUIT:
                running = False

        screen.fill((48, 48, 48))
        msg = node.map
        if msg is None:
            screen.blit(font.render("waiting for /map ...", True, (255, 255, 255)), (10, 10))
        else:
            info = msg.info
            scale = WINDOW / max(info.width, info.height)          # fit the map in the window
            picture = pygame.transform.scale(map_to_surface(msg),
                                             (int(info.width * scale), int(info.height * scale)))
            screen.blit(picture, (0, 0))
            if node.robot_xy is not None:                           # red dot = robot1
                col = (node.robot_xy[0] - info.origin.position.x) / info.resolution
                row = (node.robot_xy[1] - info.origin.position.y) / info.resolution
                pygame.draw.circle(screen, (230, 40, 40),
                                   (int(col * scale), int((info.height - row) * scale)), 6)
            text = (f"map {info.width} x {info.height} cells = "
                    f"{info.width * info.resolution:.1f} x {info.height * info.resolution:.1f} m")
            screen.blit(font.render(text, True, (255, 255, 0)), (10, WINDOW - 25))

        pygame.display.flip()
        clock.tick(5)                             # redraw 5 times per second

    pygame.quit()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
