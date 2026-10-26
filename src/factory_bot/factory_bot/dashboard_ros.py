"""ROS transport runs outside the Qt GUI thread. Commands reuse existing fleet topics."""
import json
import math
from pathlib import Path
import queue
import threading
import time

from factory_bot.dashboard_state import ROBOT_NAMES, delivery_request
from factory_bot.dashboard_navigation import NavigationSettingsClient, TARGETS


class RosBridge(threading.Thread):
    def __init__(self, state, stations, ros_args=None):
        super().__init__(name='dashboard_ros', daemon=True)
        self.state, self.stations = state, stations
        self.ros_args = ros_args
        self.commands = queue.Queue(maxsize=32)
        self.stop_event = threading.Event()

    def request(self, kind, payload):
        try:
            self.commands.put_nowait((kind, payload, time.monotonic()))
        except queue.Full:
            raise ValueError('Command queue is busy. Wait for the pending requests.')

    def stop(self):
        self.stop_event.set()

    def set_camera(self, robot):
        if robot is not None and robot not in ROBOT_NAMES:
            raise ValueError('Unknown camera robot')
        with self.state.lock:
            self.state.active_camera = robot

    def run(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import Path as RosPath, Odometry
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import String, Bool
        from rosgraph_msgs.msg import Clock
        from rcl_interfaces.msg import Log

        context = Context()
        node = executor = None
        try:
            rclpy.init(args=self.ros_args, context=context, signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('factory_dashboard', context=context,
                                    parameter_overrides=[Parameter('use_sim_time', value=True)])
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            publishers = {kind: node.create_publisher(String, topic, 10) for kind, topic in
                          [('delivery', '/delivery_request'), ('emergency', '/emergency_call')]}
            navigation = NavigationSettingsClient(self.state,
                {name: AsyncParameterClient(node, name) for name in TARGETS},
                lambda name, value: Parameter(name, value=value))

            def json_message(topic, msg):
                try:
                    value = json.loads(msg.data)
                    if isinstance(value, dict): self.state.update(topic, value)
                except (ValueError, TypeError):
                    self.state.event('warning', 'Invalid JSON on '+topic)

            for topic in ['/fleet/state', '/traffic/state']:
                node.create_subscription(String, topic, lambda m, t=topic: json_message(t, m), 10)
            path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
            image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for robot in ROBOT_NAMES:
                t = f'/{robot}/nav_status'
                node.create_subscription(String, t, lambda m, t=t: json_message(t, m), 10)
                t = f'/{robot}/plan'
                node.create_subscription(RosPath, t, lambda m, t=t: self.state.update(
                    t, [(p.pose.position.x, p.pose.position.y) for p in m.poses]), path_qos)
                t = f'/{robot}/localization_ok'
                node.create_subscription(Bool, t, lambda m, t=t: self.state.update(t, bool(m.data)), 10)
                t = f'/{robot}/odom'
                node.create_subscription(Odometry, t, lambda m, t=t: self.state.update(
                    t, {'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z}), qos_profile_sensor_data)
                t = f'/{robot}/scan'

                def scan(msg, t=t):
                    valid = [v for v in msg.ranges if math.isfinite(v) and msg.range_min < v < msg.range_max]
                    self.state.update(t, {'nearest': min(valid) if valid else None,
                                          'beams': len(msg.ranges), 'max_range': msg.range_max})
                node.create_subscription(LaserScan, t, scan, qos_profile_sensor_data)
                rear=f'/{robot}/rear_obstacle_scan'
                node.create_subscription(LaserScan,rear,lambda m,t=rear,cb=scan:cb(m,t),
                                         qos_profile_sensor_data)
            node.create_subscription(Clock, '/clock', lambda m: self.state.clock(
                m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)

            def log(msg):
                if 'icp_localizer' in msg.name:
                    self.state.update('/diagnostics/'+msg.name, {'message': msg.msg, 'level': msg.level})
                if msg.level >= 40 and 'dashboard' not in msg.name:
                    self.state.event('warning', msg.name+' · '+msg.msg[:180])
            node.create_subscription(Log, '/rosout', log, 100)
            self.state.event('system', 'ROS connected · waiting for fleet telemetry')
            last_sample = 0.
            previous_cpu = None
            camera_sub, camera_robot = None, None
            while context.ok() and not self.stop_event.is_set():
                navigation.tick()
                with self.state.lock:
                    desired = self.state.active_camera
                if desired != camera_robot:
                    if camera_sub is not None:
                        node.destroy_subscription(camera_sub)
                        camera_sub = None
                    camera_robot = desired
                    if desired is not None:
                        topic = f'/{desired}/camera/image_raw'
                        camera_sub = node.create_subscription(Image, topic, lambda m, t=topic: self.state.update(
                            t, {'width': m.width, 'height': m.height, 'step': m.step, 'encoding': m.encoding,
                                'data': bytes(m.data), 'stamp': (m.header.stamp.sec, m.header.stamp.nanosec),
                                'frame': m.header.frame_id}), image_qos)
                executor.spin_once(timeout_sec=.02)
                now = time.monotonic()
                for _ in range(4):
                    try: kind, payload, submitted = self.commands.get_nowait()
                    except queue.Empty: break
                    try:
                        with self.state.lock:
                            stamp = self.state.seen.get('/fleet/state')
                        if stamp is None or now-stamp > 3.:
                            raise ValueError('Fleet telemetry is offline or stale; request was not sent.')
                        if now-submitted > 5.:
                            raise ValueError('Request expired before it could be sent.')
                        if kind == 'navigation_settings':
                            navigation.apply(payload)
                            continue
                        if kind == 'delivery':
                            payload = delivery_request(payload['pickup'], payload['dropoff'], payload['priority'], self.stations)
                        elif kind == 'emergency':
                            if payload.get('station') not in self.stations: raise ValueError('Choose a valid emergency station.')
                        elif kind == 'demo':
                            from factory_bot.send_job import DEMO
                            for pickup, dropoff, priority in DEMO:
                                if publishers['delivery'].get_subscription_count() == 0:
                                    raise ValueError('Fleet manager is not listening; demo was not sent.')
                                publishers['delivery'].publish(String(data=json.dumps(
                                    delivery_request(pickup, dropoff, priority, self.stations))))
                            self.state.event('request', 'Four demo requests sent · watch the fleet queue')
                            continue
                        else: raise ValueError('Unknown dashboard request')
                        if publishers[kind].get_subscription_count() == 0:
                            raise ValueError('Fleet manager is not listening; request was not sent.')
                        publishers[kind].publish(String(data=json.dumps(payload)))
                        self.state.event('request', 'Request sent · '+json.dumps(payload)+' · awaiting fleet update')
                    except (ValueError, KeyError, TypeError) as e:
                        self.state.event('error', str(e))
                if now-last_sample >= 1.:
                    last_sample = now
                    with self.state.lock:
                        for r in ROBOT_NAMES:
                            odom = self.state.values.get(f'/{r}/odom')
                            stamp = self.state.seen.get(f'/{r}/odom', -math.inf)
                            self.state.speed_history[r].append((now, odom['v'] if odom and now-stamp < 3 else None))
                    try:
                        cpu = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
                        total, idle = sum(cpu), cpu[3]+cpu[4]
                        percent = None
                        if previous_cpu and total > previous_cpu[0]:
                            percent = 100*(1-(idle-previous_cpu[1])/(total-previous_cpu[0]))
                        previous_cpu = total, idle
                        mem = {l.split(':')[0]: int(l.split()[1]) for l in Path('/proc/meminfo').read_text().splitlines()}
                        self.state.update('/system/resources', {'cpu': percent, 'ram_used_gb': (mem['MemTotal']-mem['MemAvailable'])/1048576,
                                                                'ram_total_gb': mem['MemTotal']/1048576})
                    except (OSError, ValueError, IndexError, KeyError):
                        pass
        except Exception as e:
            with self.state.lock: self.state.backend_error = str(e)
            self.state.event('error', 'ROS connection stopped · '+str(e))
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=2)
            if node is not None:
                node.destroy_node()
            if context.ok(): context.try_shutdown()
