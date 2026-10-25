"""Normal and emergency missions: wait, recheck, then make a bounded detour."""


class NormalObstaclePolicy:
    def __init__(self, wait_seconds=3.0):
        self.wait_seconds = wait_seconds
        self.wait_until = None
        self.detour_until = None

    def start_wait(self, now):
        """Also used when a commanded movement made no progress."""
        self.detour_until = None
        self.wait_until = now + self.wait_seconds

    def decide(self, blocked, now, scan_time):
        """drive/wait/replan/detour; forward motion is never exempt from detection.

        A replacement route gets at most one wait period to leave the blocked
        sector, with the navigator's close-range stop still enforced. If the
        obstacle remains ahead, stop and try planning again.
        """
        if self.detour_until is not None:
            if not blocked:
                self.detour_until = self.wait_until = None
                return "drive"
            if now < self.detour_until:
                return "detour"
            self.start_wait(now)
        if self.wait_until is None:
            if not blocked:
                return "drive"
            self.start_wait(now)
        if now < self.wait_until or scan_time < self.wait_until:
            return "wait"
        if not blocked:
            self.wait_until = None
            return "drive"
        return "replan"

    def rerouted(self, success, now):
        if success:
            self.wait_until = None
            self.detour_until = now + self.wait_seconds
        else:
            self.start_wait(now)
