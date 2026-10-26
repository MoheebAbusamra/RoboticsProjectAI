"""Own only restartable navigation processes; preserve Gazebo, dashboard and fleet queue."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from nav2_msgs.srv import LoadMap
from lifecycle_msgs.srv import GetState

from factory_bot.map_catalog import inspect_map, freeze_map, navigation_command, check_robot_poses
from factory_bot.map_transition import MapTransition
from factory_bot.mapping_controls import MappingControls
from factory_bot.mapping_session import MappingSession
from factory_bot.planning_grid import PlanningGrid


class NavigationProcess:
    """Signals are restricted to a process group created and retained by this owner."""
    def __init__(self, directory):
        self.process = None; self.stopping = None; self.log = None; self.directory = Path(directory)
        self.owned = {}

    @staticmethod
    def identity(pid):
        try:
            fields = Path(f'/proc/{pid}/stat').read_text().rsplit(') ', 1)[1].split()
            return None if fields[0] == 'Z' else fields[19]
        except (OSError, IndexError): return None

    def track(self):
        if not self.process: return
        pending = [self.process.pid]; seen = set()
        while pending:
            pid = pending.pop()
            if pid in seen: continue
            seen.add(pid); identity = self.identity(pid)
            if identity is None: continue
            if pid in self.owned and self.owned[pid] != identity: continue
            self.owned[pid] = identity
            try:
                for task in Path(f'/proc/{pid}/task').iterdir():
                    try: pending.extend(int(p) for p in (task/'children').read_text().split())
                    except OSError: pass
            except OSError: pass

    def signal_owned(self, sig):
        for pid, identity in self.owned.items():
            if self.identity(pid) == identity:
                try: os.kill(pid, sig)
                except ProcessLookupError: pass

    def start(self, command):
        if self.process and not self.stopped(): raise RuntimeError('Previous navigation processes are still running')
        if self.log: self.log.close()
        self.log = (self.directory/('navigation-'+str(time.time_ns())+'.log')).open('wb')
        self.process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)
        self.stopping = None; self.owned = {}; self.track()

    def stop(self):
        if self.process and self.stopping is None:
            self.track(); self.signal_owned(signal.SIGINT); self.stopping = time.monotonic()

    def stopped(self):
        self.track()
        parent_done = self.process is None or self.process.poll() is not None
        descendants_done = not any(self.identity(p) == identity for p, identity in self.owned.items())
        if parent_done and descendants_done: return True
        if self.stopping is not None:
            elapsed = time.monotonic()-self.stopping
            if elapsed > 6: self.signal_owned(signal.SIGKILL)
            elif elapsed > 4: self.signal_owned(signal.SIGTERM)
        return False

    def alive(self):
        self.track()
        return self.process is not None and self.process.poll() is None

    def close(self):
        self.stop()
        limit = time.monotonic()+7
        while not self.stopped() and time.monotonic() < limit: time.sleep(.05)
        if self.log: self.log.close()


class MapCoordinator(Node):
    def __init__(self):
        super().__init__('map_coordinator')
        self.share = Path(get_package_share_directory('factory_bot'))
        defaults = {'robots': ['robot1', 'robot2'], 'map_yaml': str(self.share/'maps/factory_map.yaml'),
            'algorithm': 'astar', 'controller': 'pure_pursuit', 'traffic': True,
            'ready_timeout': 60.0,
            'state_dir': str(Path.home()/'factory_ws/work/map_control')}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.get = lambda key: self.get_parameter(key).value
        self.robots = list(self.get('robots')); self.directory = Path(self.get('state_dir')).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.generated = str((self.share/'maps/factory_map.yaml').resolve())
        self.history_path = self.directory/'recent.json'
        try:
            recent = json.loads(self.history_path.read_text())
            self.recent = [p for p in recent if isinstance(p, str)][:12] if isinstance(recent, list) else []
        except (OSError, ValueError, TypeError): self.recent = []
        self.data = {}; self.old_instances = {}; self.group = NavigationProcess(self.directory)
        self.motion_held = True; self.server_query = None; self.server_ok = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='map_inspection')
        self.preparing = None; self.preparation_id = ''; self.request_error = ''
        self.received_request_id = ''
        self.last_phase = ''; self.last_active = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/maps/status', latched)
        self.hold_pub = self.create_publisher(Bool, '/maps/hold', latched)
        self.mapping_preview_pub = self.create_publisher(OccupancyGrid, '/mapping/preview', latched)
        self.mapping_path_pub = self.create_publisher(RosPath, '/mapping/preview_path', latched)
        self.create_subscription(OccupancyGrid,'/mapping/live_map',self.on_mapping_grid,latched)
        self.create_subscription(RosPath,'/mapping/trajectory',self.on_mapping_path,latched)
        self.stops = {r: self.create_publisher(TwistStamped, '/'+r+'/cmd_vel', 10) for r in self.robots}
        self.pause_client = self.create_client(SetBool, '/fleet_manager/map_pause')
        self.load_client = self.create_client(LoadMap, '/fleet_manager/activate_map')
        self.server_client = AsyncParameterClient(self, '/map_server')
        self.lifecycle = self.create_client(GetState, '/map_server/get_state')
        self.create_subscription(String, '/maps/request', self.on_request, 10)
        for topic in ['/fleet/state', '/traffic/state', '/fleet/obstacles_state', '/mapping/session_status'] + [f'/{r}/{t}' for r in self.robots for t in ('nav_status', 'icp_status')]:
            self.create_subscription(String, topic, lambda m, t=topic: self.receive(t, m), 10)
        self.machine = MapTransition(self, inspect_map(self.get('map_yaml')), self.get('ready_timeout'))
        self.mapping_io = MappingControls(self, NavigationProcess)
        self.mapping = MappingSession(self.mapping_io)
        self.saved_session = ''
        self.last_mapping_phase = ''
        self.start_pending = True  # Start children only after construction succeeds and spin begins.
        self.create_timer(.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, topic, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict): self.data[topic] = (value, time.monotonic())
        except (ValueError, TypeError): pass

    def on_mapping_grid(self,msg):
        if msg.header.frame_id=='map' and 0<len(msg.data)<=2_000_000 and len(msg.data)==msg.info.width*msg.info.height:
            self.mapping_preview_pub.publish(msg)

    def on_mapping_path(self,msg):
        if msg.header.frame_id=='map' and len(msg.poses)<=2000:self.mapping_path_pub.publish(msg)

    def clear_mapping_view(self):
        grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=self.get_clock().now().to_msg()
        grid.info.width=332;grid.info.height=292;grid.info.resolution=.05
        grid.info.origin.position.x=-1.3;grid.info.origin.position.y=-1.3;grid.info.origin.orientation.w=1.
        grid.data=[-1]*(332*292);self.mapping_preview_pub.publish(grid)
        path=RosPath();path.header=grid.header;self.mapping_path_pub.publish(path)

    def value(self, topic, since=0.):
        value, stamp = self.data.get(topic, ({}, -math.inf))
        return value if stamp >= since and time.monotonic()-stamp < 2.5 else {}

    def check_idle(self):
        fleet = self.value('/fleet/state')
        records = fleet.get('robots', {})
        if set(records) != set(self.robots) or any(v.get('state') != 'IDLE' or v.get('job') or
                v.get('suspended') for v in records.values()):
            raise ValueError('Wait for both robots to finish their missions and return idle')
        # A held fleet retains its verified pose snapshot even while its navigation group is stopped.
        if self.machine.phase == 'ERROR_HELD' and fleet.get('map_paused') and self.machine.snapshot: return
        for r in self.robots:
            st = self.value('/'+r+'/nav_status')
            if st.get('state') not in ('IDLE', 'ARRIVED', 'FAILED') or not st.get('stopped') or st.get('aside'):
                raise ValueError('Fresh idle, measured-stopped robot telemetry is required')

    def freeze(self, asset): return freeze_map(asset, self.directory/'snapshots')

    def prepare(self, path, fingerprint):
        asset = inspect_map(path)
        if fingerprint and asset.digest != fingerprint: raise ValueError('Map changed since preview; browse it again')
        # Avoid the existing inflation implementation's unsupported smaller-than-padding grids.
        settings = __import__('yaml').safe_load((self.share/'config/planning.yaml').read_text())['grid']
        radius = sum(settings[k] for k in ('robot_radius', 'safety_margin', 'soft_band')) / asset.meta['resolution']
        if radius >= min(asset.pixels.shape): raise ValueError('Map is too small for the configured robot clearance')
        if asset.pixels.size*math.pi*radius*radius > 250_000_000:
            raise ValueError('Map is too large/fine for this Python planner; use 0.05 m cells or a smaller extent')
        frozen = self.freeze(asset)
        grid = PlanningGrid.from_files(frozen.path, str(self.share/'config/planning.yaml'), str(self.share/'config/zones.yaml'))
        return frozen, grid

    def on_request(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict): raise ValueError('Map request must be a JSON object')
            self.received_request_id = str(request.get('id', ''))
            action = request.get('action')
            if action == 'mapping_visualize':
                self.mapping_io.open_visualizer();self.request_error='';return
            if action in ('finish_mapping', 'cancel_mapping'):
                if not self.mapping.busy: raise ValueError('No mapping session is running')
                if action == 'finish_mapping': self.mapping.finish()
                else: self.mapping.cancel(time.monotonic())
                self.request_error = ''; return
            if self.preparing or self.machine.busy: raise ValueError('A map operation is already in progress')
            if self.mapping.busy: raise ValueError('Finish or cancel the mapping survey first')
            if action == 'generate':
                self.mapping_io.auto_visualize = request.get('rviz',True) is True
                self.mapping.request(request.get('mode'), time.monotonic())
                self.request_error = ''; return
            self.check_idle()
            path = self.generated if request.get('action') == 'restore' else request.get('path')
            if request.get('action') not in ('activate', 'restore') or not isinstance(path, str):
                raise ValueError('Invalid map selection')
            self.preparation_id = str(request.get('id', ''))
            self.preparing = self.worker.submit(self.prepare, path, request.get('fingerprint'))
            self.request_error = ''
        except (ValueError, TypeError, OSError, RuntimeError) as error: self.request_error = str(error)

    def pause(self, enabled):
        if not self.pause_client.service_is_ready(): raise RuntimeError('Fleet pause service is unavailable')
        return self.pause_client.call_async(SetBool.Request(data=enabled))

    def load_fleet(self, path):
        if not self.load_client.service_is_ready(): raise RuntimeError('Fleet map service is unavailable')
        return self.load_client.call_async(LoadMap.Request(map_url=path))

    def resumed(self, path):
        fleet = self.value('/fleet/state')
        return fleet.get('map_yaml') == path and fleet.get('map_paused') is False

    def hold(self, enabled):
        self.motion_held = enabled; self.hold_pub.publish(Bool(data=enabled))

    def held(self):
        if self.group.process is not None and self.group.stopped(): return True
        return all((s := self.value('/'+r+'/nav_status')).get('map_hold') and s.get('stopped') for r in self.robots)

    def capture(self, previous):
        if not self.machine.was_ready: return previous
        poses, profiles = {}, {}
        for r in self.robots:
            st = self.value('/'+r+'/nav_status')
            if not st.get('map_hold') or not st.get('stopped') or st.get('state') not in ('IDLE', 'ARRIVED', 'FAILED'):
                raise ValueError('Robot changed state before navigation could be held')
            poses[r] = st['pose']; profiles[r] = st['defaults']
        return {'poses': poses, 'profiles': profiles}

    def released(self):
        return all((s := self.value('/'+r+'/nav_status')) and s.get('map_hold') is False for r in self.robots)

    def stop(self): self.group.stop()
    def stopped(self): return self.group.stopped()

    def start(self, path, snapshot):
        self.old_instances = {t: v.get('instance_id') for t, (v, _) in self.data.items() if 'instance_id' in v}
        self.server_ok = None; self.server_query = None
        self.group.start(navigation_command(path, self.robots, self.get('algorithm'), self.get('controller'),
            self.get('traffic'), snapshot.get('poses'), snapshot.get('profiles')))

    def server_ready(self, path):
        now = time.monotonic()
        if self.server_ok and self.server_ok[0] == path and now-self.server_ok[1] < 2.: return True
        if self.server_query:
            a, b, stamp = self.server_query
            if now-stamp > 3.: self.server_query = None; return False
            if not (a.done() and b.done()): return False
            self.server_query = None
            if (a.result().values[0].string_value == path and b.result().current_state.id == 3):
                self.server_ok = (path, now); return True
            return False
        if self.server_client.services_are_ready() and self.lifecycle.service_is_ready():
            self.server_query = (self.server_client.get_parameters(['yaml_filename']),
                                 self.lifecycle.call_async(GetState.Request()), now)
        return False

    def ready(self, path, since):
        if not self.group.alive(): raise RuntimeError('Navigation group exited; inspect '+str(self.group.log.name))
        fleet = self.value('/fleet/state')
        if fleet.get('map_yaml') != path or not fleet.get('map_paused'): return False, 'waiting for fleet map acknowledgement'
        for r in self.robots:
            for suffix in ('nav_status', 'icp_status'):
                topic = '/'+r+'/'+suffix; st = self.value(topic, since)
                if (st.get('map_yaml') != path or not st.get('instance_id') or
                    st['instance_id'] == self.old_instances.get(topic)):
                    return False, 'waiting for new '+r+' '+suffix
                if suffix == 'nav_status' and (not st.get('stopped') or not st.get('map_hold')):
                    return False, 'waiting for measured stop and navigation hold'
                if suffix == 'icp_status' and (not st.get('ok') or st.get('accepted', 0) < 2):
                    return False, 'waiting for fresh accepted ICP matches on '+r
        if self.get('traffic'):
            st = self.value('/traffic/state', since)
            if st.get('map_yaml') != path or not st.get('instance_id') or st['instance_id'] == self.old_instances.get('/traffic/state'):
                return False, 'waiting for traffic map acknowledgement'
        if not self.server_ready(path): return False, 'waiting for active map server and map-path readback'
        return True, 'all consumers verified'

    def tick(self):
        now = time.monotonic()
        if self.start_pending:
            self.start_pending = False
            try: self.machine.bootstrap(now)
            except Exception as error: self.machine.fail(error, now)
        self.group.track()
        if self.machine.phase == 'READY' and not self.group.alive():
            self.machine.fail('Navigation group exited; robots held', now)
        if self.preparing and self.preparing.done():
            future, self.preparing = self.preparing, None
            try:
                asset, grid = future.result()
                poses = self.machine.snapshot.get('poses', {}) if self.machine.phase == 'ERROR_HELD' else {}
                poses = {r: poses.get(r) or self.value('/'+r+'/nav_status').get('pose') for r in self.robots}
                check_robot_poses(grid, poses)
                self.machine.request(asset, now, self.preparation_id)
            except Exception as error: self.request_error = str(error)
        self.machine.tick(now)
        if self.machine.phase != 'READY' and self.mapping.busy:
            self.mapping.cancel(now)
        self.mapping.tick(now)
        self.hold_pub.publish(Bool(data=self.motion_held))
        if self.motion_held:
            for r, publisher in self.stops.items():
                msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = r+'/base_footprint'; publisher.publish(msg)
        status = self.machine.status()
        mapping = self.mapping.status()
        mapping['engine'] = self.mapping_io.engine_status(self.mapping.identifier) or self.mapping.last_engine
        mapping.update(self.mapping_io.visualizer_status())
        if self.mapping.phase != self.last_mapping_phase:
            self.last_mapping_phase=self.mapping.phase
            self.get_logger().info('Mapping: '+self.mapping.phase+' '+self.mapping.error)
            if self.mapping.directory:
                try: (Path(self.mapping.directory)/'state.json').write_text(json.dumps(mapping,indent=2))
                except OSError as error:self.get_logger().warn('Could not save mapping state: '+str(error))
        status['mapping'] = mapping
        if self.mapping.busy:
            status.update(phase='MAPPING/'+self.mapping.phase, busy=True,
                          message='Mapping / '+mapping['message'])
        if self.mapping.phase == 'DONE' and self.mapping.identifier != self.saved_session:
            self.saved_session = self.mapping.identifier
            self.recent = list(dict.fromkeys([m['path'] for m in self.mapping.maps]+self.recent))[:12]
            try: self.history_path.write_text(json.dumps(self.recent))
            except OSError as error: self.get_logger().warn(str(error))
        if self.preparing: status.update(phase='PREPARING', busy=True, message='Inspecting and copying selected map')
        status.update(request_error=self.request_error, generated=self.generated, recent=self.recent,
                      received_request_id=self.received_request_id)
        try: self.check_idle(); status['can_activate'] = not self.machine.busy and not self.mapping.busy and not bool(self.preparing)
        except ValueError as error: status['can_activate'] = False; status['idle_reason'] = str(error)
        try:
            self.mapping_io.check_start()
            status['can_generate'] = not self.mapping.busy
        except (ValueError, RuntimeError) as error:
            status['can_generate'] = False; status['generation_reason'] = str(error)
        active_key = (self.machine.active.digest, self.machine.active.source)
        if self.machine.phase == 'READY' and active_key != self.last_active:
            self.last_active = active_key
            source = self.machine.active.source
            self.recent = ([source]+[p for p in self.recent if p != source])[:12]
            try:
                temp = self.history_path.with_suffix('.tmp'); temp.write_text(json.dumps(self.recent)); temp.replace(self.history_path)
            except OSError as error: self.get_logger().warn('Could not save map history: '+str(error))
        if self.machine.phase != self.last_phase:
            self.last_phase = self.machine.phase
            self.get_logger().info('Map activation: '+self.machine.phase+' '+self.machine.note)
        self.status_pub.publish(String(data=json.dumps(status)))

    def close(self):
        try:
            self.hold(True)
            for r, publisher in self.stops.items():
                msg = TwistStamped(); msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = r+'/base_footprint'; publisher.publish(msg)
        except Exception: pass  # ROS may already be shutting down; controller timeout is still active.
        self.worker.shutdown(wait=False, cancel_futures=True)
        self.mapping_io.group.close()
        self.mapping_io.visualizer.close()
        self.group.close()


def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = MapCoordinator(); rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node: node.close(); node.destroy_node()
        rclpy.try_shutdown()
