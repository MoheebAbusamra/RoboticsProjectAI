"""Bounded map-activation transaction with injected I/O, clocks and process owner."""
import json


class PauseDenied(ValueError):
    pass


class MapTransition:
    def __init__(self, backend, active, verify_timeout=30.):
        if not 1 <= verify_timeout <= 180: raise ValueError('Invalid map readiness timeout')
        self.verify_timeout = verify_timeout
        self.io = backend; self.active = active; self.target = active; self.previous = active
        self.phase = 'OFFLINE'; self.error = ''; self.note = ''; self.future = None
        self.snapshot = {}; self.rollback = False; self.started = 0.; self.deadline = 0.
        self.last_request = ''; self.changed = False
        self.was_ready = False

    @property
    def busy(self): return self.phase not in ('READY', 'ERROR_HELD', 'OFFLINE')

    @property
    def hold(self): return self.phase not in ('READY', 'PAUSING', 'OFFLINE', 'RELEASING', 'RESUMING')

    def stage(self, name, now, seconds=8.):
        self.phase = name; self.deadline = now+seconds; self.note = name.replace('_', ' ').lower()

    def bootstrap(self, now):
        self.snapshot = {}; self.rollback = False; self.changed = False
        self.io.hold(True); self.io.start(self.active.path, self.snapshot)
        self.started = now; self.stage('VERIFYING', now, self.verify_timeout)

    def request(self, target, now, request_id=''):
        if self.busy: raise ValueError('A map activation is already in progress')
        if self.phase not in ('READY', 'ERROR_HELD'): raise ValueError('Map coordinator is not ready')
        self.io.check_idle()
        self.was_ready = self.phase == 'READY'
        # Retain a verified rollback copy before touching the active navigation group.
        self.previous = self.io.freeze(self.active)
        self.target = target; self.rollback = False; self.error = ''; self.changed = False
        self.last_request = request_id; self.future = self.io.pause(True)
        self.stage('PAUSING', now)

    def fail(self, reason, now):
        self.error = str(reason); self.io.hold(True)
        if self.changed and not self.rollback:
            self.rollback = True; self.target = self.previous
            self.io.stop(); self.stage('STOPPING', now)
            self.note = 'activation failed; restoring the previous map'
        else:
            self.phase = 'ERROR_HELD'; self.note = 'Robots held. '+str(reason)

    def future_result(self):
        if not self.future.done(): return None
        result = self.future.result()
        if hasattr(result, 'success') and not result.success: raise PauseDenied(result.message)
        if hasattr(result, 'result') and result.result != 0: raise ValueError('Fleet could not load the map')
        return result

    def tick(self, now):
        if not self.busy: return
        try:
            if self.phase == 'RESUMING' and getattr(self.io, 'resumed', lambda path: False)(self.target.path):
                self.finish(); return
            if now > self.deadline:
                detail='; '+self.note if self.phase=='VERIFYING' and self.note else ''
                raise TimeoutError(self.phase.replace('_', ' ').lower()+' timed out'+detail)
            if self.phase == 'PAUSING':
                result = self.future_result()
                if result is None: return
                paused = json.loads(result.message)
                if self.was_ready or not self.snapshot: self.snapshot = paused
                self.io.hold(True); self.stage('HOLDING', now)
            elif self.phase == 'HOLDING':
                if not self.io.held(): return
                self.snapshot = getattr(self.io, 'capture', lambda snapshot: snapshot)(self.snapshot)
                self.io.stop(); self.changed = True; self.stage('STOPPING', now)
            elif self.phase == 'STOPPING':
                if not self.io.stopped(): return
                self.future = self.io.load_fleet(self.target.path); self.stage('LOADING_FLEET', now)
            elif self.phase == 'LOADING_FLEET':
                if self.future_result() is None: return
                self.io.start(self.target.path, self.snapshot); self.started = now
                self.stage('VERIFYING', now, self.verify_timeout)
            elif self.phase == 'VERIFYING':
                ready, self.note = self.io.ready(self.target.path, self.started)
                if not ready: return
                self.io.hold(False); self.stage('RELEASING', now)
            elif self.phase == 'RELEASING':
                if not self.io.released(): return
                self.future = self.io.pause(False); self.stage('RESUMING', now)
            elif self.phase == 'RESUMING':
                if self.future_result() is None: return
                self.finish()
        except Exception as error:
            # A denied pause has not taken control or changed any map.
            if self.phase == 'PAUSING' and isinstance(error, PauseDenied) and self.was_ready:
                self.phase = 'READY'; self.error = str(error); self.note = str(error)
            elif self.phase == 'RESUMING':
                # Dispatch may have resumed despite a lost reply. Never restart
                # navigation underneath a possibly newly assigned delivery.
                self.active = self.target; self.changed = False; self.fail(error, now)
            else: self.fail(error, now)

    def finish(self):
        self.active = self.target; self.phase = 'READY'; self.changed = False
        self.note = 'Previous map restored; activation failed' if self.rollback else 'Map verified; deliveries enabled'

    def status(self):
        return {'phase': self.phase, 'busy': self.busy, 'verified': self.phase == 'READY',
                'active': self.active.info(), 'target': self.target.info(), 'error': self.error,
                'message': self.note, 'request_id': self.last_request, 'rolling_back': self.rollback}
