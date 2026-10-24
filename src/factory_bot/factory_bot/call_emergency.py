"""call_emergency: raise an emergency at a station - a robot brings the emergency kit there NOW.
  ros2 run factory_bot call_emergency ASSEMBLY_A
  ros2 run factory_bot call_emergency QC_LAB --kit STORES
"""
import argparse
import json
import time

import rclpy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser(description="Raise an emergency call")
    ap.add_argument("station", help="where the emergency is, e.g. ASSEMBLY_A")
    ap.add_argument("--kit", default=None, help="where to fetch the kit (default: the fleet manager's emergency_kit)")
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node("call_emergency")
    pub = node.create_publisher(String, "/emergency_call", 10)
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:   # wait for the fleet manager
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print("nobody is listening on /emergency_call - is the fleet manager running?")
    request = {"station": a.station}
    if a.kit:
        request["kit"] = a.kit
    pub.publish(String(data=json.dumps(request)))
    print(f"EMERGENCY sent: {request}")
    time.sleep(0.5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
