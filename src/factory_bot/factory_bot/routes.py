"""Back-and-forth motion along a route at constant speed. Plain Python, no ROS.
Used by obstacle_mover.py."""
import math


class Route:
    """A model that walks from the first point to the last and back, forever."""

    def __init__(self, points, speed):
        self.points = [(float(x), float(y)) for x, y in points]
        self.speed = float(speed)
        # length of each straight piece between two consecutive points
        self.lengths = [math.dist(a, b) for a, b in zip(self.points, self.points[1:])]
        self.total = sum(self.lengths)          # one-way length of the whole route

    def at(self, t):
        """Where the model is after t seconds: (x, y, yaw)."""
        if self.total == 0:                     # a route with one point: stand still
            return (*self.points[0], 0.0)
        # distance travelled, folded into one round trip (there and back)
        s = (self.speed * t) % (2 * self.total)
        forward = s <= self.total               # first half of the trip = going forward
        if not forward:
            s = 2 * self.total - s              # coming back: mirror the distance
        segments = list(zip(self.points, self.points[1:]))
        for i, ((a, b), length) in enumerate(zip(segments, self.lengths)):
            if s <= length or i == len(segments) - 1:
                f = min(1.0, s / length) if length > 0 else 0.0   # fraction along this piece
                yaw = math.atan2(b[1] - a[1], b[0] - a[0]) + (0.0 if forward else math.pi)
                return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, yaw)
            s -= length                         # not on this piece: move to the next one
