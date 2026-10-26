"""Dashboard-only access to existing navigation parameters; no motion commands."""
import time
from concurrent.futures import Future

PLANNERS = (('A*', 'astar'), ('Dijkstra', 'dijkstra'), ('BFS', 'bfs'))
CONTROLLERS = (('Pure pursuit', 'pure_pursuit'), ('PID', 'pid'))
TARGETS = {'/fleet_manager': ('algorithm',),
           '/robot1/navigator': ('algorithm', 'controller'),
           '/robot2/navigator': ('algorithm', 'controller')}
SETTINGS_TOPIC = '/dashboard/navigation_settings'


def navigation_request(algorithm, controller):
    if algorithm not in dict((value, label) for label, value in PLANNERS):
        raise ValueError('Choose A*, Dijkstra or BFS.')
    if controller not in dict((value, label) for label, value in CONTROLLERS):
        raise ValueError('Choose PID or Pure pursuit.')
    return {'algorithm': algorithm, 'controller': controller}


def display_option(value, options):
    return dict((key, label) for label, key in options).get(value, 'Unknown')


def settings_consensus(nodes):
    def common(key):
        values = [nodes.get(target, {}).get(key) for target, names in TARGETS.items() if key in names]
        return values[0] if values and values[0] is not None and len(set(values)) == 1 else None
    return {'algorithm': common('algorithm'), 'controller': common('controller')}


class NavigationSettingsClient:
    """Poll, apply and read back parameter services without blocking the ROS thread.

    Changes are atomic per node, not across three nodes. Partial changes and
    uncertain timeouts remain visible; a request is never reported applied until
    all three nodes have acknowledged it and read-back matches the selection.
    Clients and parameter construction are injected to keep tests independent of ROS.
    """
    def __init__(self, state, clients, parameter, now=time.monotonic):
        self.state, self.clients, self.parameter, self.now = state, clients, parameter, now
        self.nodes = {};self.ready = False;self.error = '';self.error_is_apply = False
        self.pending = None;self.requested = None;self.verifying = None
        self.next_poll = 0.;self.started = 0.

    def publish(self):
        self.state.update(SETTINGS_TOPIC, {'nodes': self.nodes, 'ready': self.ready,
            'applying': self.requested is not None or self.verifying is not None or
                        bool(self.pending and self.pending[0] == 'write'), 'error': self.error})

    def apply(self, payload):
        request = navigation_request(payload.get('algorithm'), payload.get('controller'))
        if self.requested or self.verifying or (self.pending and self.pending[0] == 'write'):
            raise ValueError('Navigation settings are already being applied.')
        if not self.ready or not all(c.services_are_ready() for c in self.clients.values()):
            self.ready = False
            self.error = 'Wait for the fleet manager and both navigators before applying settings.'
            self.publish();raise ValueError(self.error)
        self.requested = request;self.error = '';self.error_is_apply = False;self.publish()

    def begin_read(self):
        self.started = self.now()
        self.pending = ('read', {name: self.call(self.clients[name].get_parameters, list(keys))
                                 for name, keys in TARGETS.items()})

    @staticmethod
    def call(method, values):
        try:return method(values)
        except Exception as error:
            future=Future();future.set_exception(error);return future

    def begin_write(self):
        request = self.requested;self.requested = None;self.verifying = request
        self.started = self.now()
        futures = {}
        # Controllers/default algorithms on both robots, then the fleet default.
        for name in ['/robot1/navigator', '/robot2/navigator', '/fleet_manager']:
            values = [self.parameter(key, request[key]) for key in TARGETS[name]]
            futures[name] = self.call(self.clients[name].set_parameters_atomically, values)
        self.pending = ('write', futures);self.publish()

    def tick(self):
        now = self.now()
        if self.pending:
            mode, futures = self.pending
            if not all(f.done() for f in futures.values()):
                if now-self.started <= 5.:return
                for f in futures.values():
                    if not f.done():f.cancel()
                self.error_is_apply = mode == 'write' or self.verifying is not None
                self.pending = None;self.requested = None;self.verifying = None
                self.ready = False
                self.error = 'Parameter confirmation timed out. Defaults may be partially changed; waiting to read them again.'
                self.state.event('error', self.error);self.next_poll = now+1;self.publish();return
            self.pending = None
            failures = []
            if mode == 'write':
                for name, f in futures.items():
                    try:
                        result = f.result().result
                        if not result.successful:failures.append(name+': '+(result.reason or 'request rejected'))
                    except Exception as e:failures.append(name+': '+str(e))
                self.error = 'Some defaults may have changed. '+ '; '.join(failures) if failures else ''
                self.error_is_apply = bool(failures)
                self.begin_read();return
            nodes = {}
            for name, f in futures.items():
                try:
                    values = f.result().values
                    keys = TARGETS[name]
                    if len(values) != len(keys) or any(v.type != 4 for v in values):
                        raise ValueError('expected declared string parameters')
                    nodes[name] = dict(zip(keys, (v.string_value for v in values)))
                except Exception as e:failures.append(name+': '+str(e))
            self.ready = not failures
            if failures:self.error = 'Unable to read navigation defaults. '+'; '.join(failures)
            else:
                self.nodes = nodes
                if not self.error_is_apply:self.error = ''
            if self.verifying:
                expected = self.verifying;self.verifying = None
                matches = not failures and all(nodes[name].get(key) == expected[key]
                    for name, keys in TARGETS.items() for key in keys)
                if matches and not self.error:
                    self.state.event('settings', 'Navigation defaults confirmed: '+
                        display_option(expected['algorithm'], PLANNERS)+' / '+
                        display_option(expected['controller'], CONTROLLERS)+' · new route legs')
                else:
                    self.error = self.error or 'Read-back differs between nodes. Review defaults and retry.'
                    self.error_is_apply = True
                    self.state.event('error', self.error)
            self.next_poll = now+3;self.publish()
        if self.requested:
            if self.ready and all(c.services_are_ready() for c in self.clients.values()):
                self.begin_write()
            else:
                self.requested = None;self.ready = False
                self.error = 'A parameter service went offline; settings were not sent.';self.publish()
        elif self.pending is None and now >= self.next_poll:
            if all(c.services_are_ready() for c in self.clients.values()):self.begin_read()
            else:
                self.ready = False;self.publish();self.next_poll = now+1
