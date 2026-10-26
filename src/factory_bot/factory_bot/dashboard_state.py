"""Bounded dashboard data and geometry helpers. No Qt or ROS dependencies."""
from collections import deque
import copy
import math
import threading
import time

import numpy as np

ROBOT_NAMES = ('robot1', 'robot2')


def route_remaining(path, pose):
    if not path or pose is None:
        return None
    i = min(range(len(path)), key=lambda j: math.dist(path[j][:2], pose[:2]))
    return math.dist(pose[:2], path[i][:2]) + sum(
        math.dist(a[:2], b[:2]) for a, b in zip(path[i:], path[i+1:]))


def project_map(x, y, projection):
    """Map meters -> normalized Blender atlas coordinates (world = map + offset)."""
    ox, oy = projection.get('world_offset', [1, 1])
    origin, px, py = [projection[k] for k in ('origin', 'x', 'y')]
    return tuple(origin[i] + (x+ox)*(px[i]-origin[i]) + (y+oy)*(py[i]-origin[i]) for i in (0, 1))


def image_rgb(width, height, step, encoding, data):
    """Decode common ROS camera formats, respecting padded rows and owning the result."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if encoding not in channels:
        raise ValueError('Unsupported camera encoding: '+str(encoding))
    n = channels[encoding]
    if width <= 0 or height <= 0 or width > 4096 or height > 4096:
        raise ValueError('Invalid camera dimensions')
    if step < width*n or len(data) < step*height:
        raise ValueError('Truncated camera image or invalid row stride')
    pixels = np.frombuffer(data, dtype=np.uint8, count=height*step).reshape(height, step)
    pixels = pixels[:, :width*n].reshape(height, width, n)
    if n == 1:
        return np.repeat(pixels, 3, axis=2)
    if encoding.startswith('bgr'):
        return np.ascontiguousarray(pixels[:, :, [2, 1, 0]])
    return pixels[:, :, :3].copy()


def delivery_request(pickup, dropoff, priority, stations):
    if pickup not in stations or dropoff not in stations:
        raise ValueError('Select a valid pickup and destination.')
    if pickup == dropoff:
        raise ValueError('Pickup and destination must be different.')
    if priority not in ('routine', 'urgent'):
        raise ValueError('Use the emergency call for emergency-kit delivery.')
    return {'pickup': pickup, 'dropoff': dropoff, 'priority': priority}


class DashboardState:
    def __init__(self):
        self.lock = threading.RLock()
        self.values, self.seen, self.rates = {}, {}, {}
        self.events = deque(maxlen=120)
        self.speed_history = {r: deque(maxlen=120) for r in ROBOT_NAMES}
        self.clock_history = deque(maxlen=80)
        self.sim_time = None
        self.backend_error = None
        self.preview = False
        self.active_camera = 'robot1'

    def event(self, kind, message, now=None):
        with self.lock:
            self.events.appendleft({'kind': kind, 'message': message,
                                    'sim': self.sim_time, 'when': time.monotonic() if now is None else now})

    def update(self, topic, value, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            previous = self.values.get(topic)
            self.values[topic], self.seen[topic] = value, now
            samples = self.rates.setdefault(topic, deque(maxlen=240))
            samples.append(now)
            if topic == '/fleet/state':
                for r, rec in value.get('robots', {}).items():
                    old = (previous or {}).get('robots', {}).get(r, {})
                    if rec.get('job') and (rec.get('job'), rec.get('target')) != (old.get('job'), old.get('target')):
                        self.event('mission', f"{r.upper()} · {rec['job']} → {rec.get('target') or 'assigned'}", now)
                if previous and value.get('delivered', 0) > previous.get('delivered', 0):
                    self.event('success', 'Delivery complete · '+str(value['delivered'])+' delivered this session', now)
            elif topic == '/traffic/state':
                for r, rec in value.get('robots', {}).items():
                    old = (previous or {}).get('robots', {}).get(r, {})
                    if (rec.get('cmd'), rec.get('because')) != (old.get('cmd'), old.get('because')):
                        text = f"{r.upper()} · {rec.get('cmd', 'go').upper()}"
                        if rec.get('because'): text += ' for '+rec['because'].upper()
                        if rec.get('rule'): text += ' · '+rec['rule']
                        self.event('traffic', text, now)
            elif topic.endswith('/nav_status') and previous and value.get('state') != previous.get('state'):
                if value.get('state') in ('FAILED', 'BLOCKED', 'ARRIVED', 'RECOVERING'):
                    kind='recovery' if value['state']=='RECOVERING' else 'success' if value['state']=='ARRIVED' else 'warning'
                    self.event(kind,
                               topic.split('/')[1].upper()+' · '+value['state'], now)
            elif topic == '/maps/status' and previous and value.get('phase') != previous.get('phase'):
                self.event('map', 'Map / '+value.get('phase','unknown')+' · '+value.get('message',''), now)

    def clock(self, sim, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.sim_time is not None and sim < self.sim_time:
                self.clock_history.clear()
                self.event('warning', 'Simulation clock reset', now)
            self.sim_time = sim
            if not self.clock_history or now-self.clock_history[-1][0] > .2:
                self.clock_history.append((now, sim))
            self.seen['/clock'] = now

    def snapshot(self):
        with self.lock:
            return {'values': copy.deepcopy(self.values), 'seen': dict(self.seen),
                    'rates': {t: tuple(q) for t, q in self.rates.items()},
                    'events': list(self.events), 'sim': self.sim_time,
                    'clock': tuple(self.clock_history),
                    'speed_history': {r: tuple(q) for r, q in self.speed_history.items()},
                    'error': self.backend_error, 'preview': self.preview,
                    'active_camera': self.active_camera}


def age(snapshot, topic, now=None):
    stamp = snapshot['seen'].get(topic)
    return math.inf if stamp is None else (time.monotonic() if now is None else now)-stamp


def topic_hz(snapshot, topic, now=None):
    now = time.monotonic() if now is None else now
    recent = [t for t in snapshot.get('rates', {}).get(topic, ()) if now-t <= 3.]
    return (len(recent)-1)/(recent[-1]-recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.


def real_time_factor(snapshot):
    h = snapshot.get('clock', ())
    if len(h) < 2 or h[-1][0] <= h[0][0]:
        return None
    return max(0., (h[-1][1]-h[0][1])/(h[-1][0]-h[0][0]))
