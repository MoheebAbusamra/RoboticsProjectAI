"""Small pose-based watchdog for commanded motion. No ROS or planner dependency."""
import math


class MotionProgress:
    def __init__(self, window=3.0, timeout=30.0, distance=.03, angle=.10):
        self.window, self.timeout = window, timeout
        self.distance, self.angle = distance, angle
        self.anchor = self.since = self.stalled_since = None

    def reset(self):
        self.anchor = self.since = self.stalled_since = None

    def observe(self, pose, now, v, w):
        """ok/stalled/failed. A new plan does not count as physical progress.

        Translation must change while driving; rotating wheels / publishing a
        command cannot satisfy this check. Intentional stops pause the short
        window while retaining a previously detected stall's finite deadline.
        The caller resets the watchdog for intentional traffic/localization holds.
        """
        if self.anchor is None:
            self.anchor, self.since = tuple(pose), now
        driving, turning = abs(v) > .02, abs(w) > .05
        moved = math.dist(pose[:2], self.anchor[:2]) >= self.distance
        turned = abs(math.remainder(pose[2]-self.anchor[2], 2*math.pi)) >= self.angle
        if moved or (turning and not driving and turned):
            self.anchor, self.since, self.stalled_since = tuple(pose), now, None
            return 'ok'
        if self.stalled_since is not None and now-self.stalled_since >= self.timeout:
            return 'failed'
        if not driving and not turning:
            self.anchor, self.since = tuple(pose), now
            return 'ok'
        if now-self.since >= self.window:
            if self.stalled_since is None:
                self.stalled_since = now
            self.anchor, self.since = tuple(pose), now
            return 'stalled'
        return 'ok'
