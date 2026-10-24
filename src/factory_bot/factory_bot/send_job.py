"""send_job: sends delivery requests to the fleet manager (/delivery_request).
  ros2 run factory_bot send_job STORES ASSEMBLY_A
  ros2 run factory_bot send_job TOOL_CRIB QC_LAB --priority urgent
  ros2 run factory_bot send_job --demo          # 4 mixed jobs, one per second
  ros2 run factory_bot send_job --demo --gap 0  # the same 4 jobs all at once
"""
import argparse
import json
import time

import rclpy
from std_msgs.msg import String

DEMO = [("STORES", "ASSEMBLY_A", "routine"), ("TOOL_CRIB", "ASSEMBLY_B", "routine"),
        ("STORES", "QC_LAB", "urgent"), ("ASSEMBLY_B", "TOOL_CRIB", "routine")]


def main():
    ap = argparse.ArgumentParser(description="Send delivery requests")
    ap.add_argument("pickup", nargs="?")
    ap.add_argument("dropoff", nargs="?")
    ap.add_argument("--priority", default="routine", choices=["routine", "urgent"])
    ap.add_argument("--demo", action="store_true", help="send 4 example jobs")
    ap.add_argument("--gap", type=float, default=1.0, help="seconds between demo jobs")
    a, ros_args = ap.parse_known_args()
    jobs = DEMO if a.demo else [(a.pickup, a.dropoff, a.priority)]
    if not a.demo and not (a.pickup and a.dropoff):
        ap.error("give PICKUP and DROPOFF stations, or --demo")

    rclpy.init(args=ros_args)
    node = rclpy.create_node("send_job")
    pub = node.create_publisher(String, "/delivery_request", 10)
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:   # wait for the fleet manager
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("nobody is listening on /delivery_request - is the fleet manager running?")
    for pickup, dropoff, priority in jobs:
        pub.publish(String(data=json.dumps({"pickup": pickup, "dropoff": dropoff, "priority": priority})))
        print(f"sent: {priority} {pickup} -> {dropoff}")
        time.sleep(a.gap if a.demo else 0.3)
    time.sleep(0.5)                                   # let the last message leave before we exit
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
