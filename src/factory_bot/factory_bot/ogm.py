"""Occupancy grid mapping with known poses (the Lab 3 method). Plain Python + NumPy, no ROS.

Each lidar beam gives two kinds of evidence:
  * the cells the beam passed through are probably FREE
  * the cell where the beam stopped is probably OCCUPIED
Evidence is stored as log-odds and simply added up, so many scans can agree or
disagree. A cell nobody has looked at stays UNKNOWN.
. Compare it with your own Lab 3 mapper."""
import numpy as np


class OccupancyGridMapper:
    def __init__(self, width_m=13.0, height_m=13.0, resolution=0.05, origin=(-1.5, -1.5),
                l_occ=0.85, l_free=-0.4, l_min=-5.0, l_max=5.0, max_range=10.0,
                 sensor_offset=0.10):
        self.res = resolution
        self.origin = origin                     # map (x, y) of the bottom-left corner of cell (0, 0)
        self.width = int(round(width_m / resolution))    # number of columns
        self.height = int(round(height_m / resolution))  # number of rows
        self.l_occ, self.l_free = l_occ, l_free
        self.l_min, self.l_max = l_min, l_max
        self.max_range = max_range
        self.sensor_offset = sensor_offset       # lidar is this far in front of the robot centre
        self.logodds = np.zeros((self.height, self.width), dtype=np.float32)
        self.seen = np.zeros((self.height, self.width), dtype=bool)

    def to_cells(self, xs, ys):
        """Map coordinates (metres) -> flat cell indices, dropping points outside the grid."""
        cols = np.floor((xs - self.origin[0]) / self.res).astype(int)
        rows = np.floor((ys - self.origin[1]) / self.res).astype(int)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)
        return np.unique(rows[inside] * self.width + cols[inside])

    def update(self, x, y, yaw, ranges, angle_min, angle_increment, range_min, range_max):
        """Add one lidar scan taken with the robot at (x, y, yaw) in the map frame."""
        r = np.asarray(ranges, dtype=float)
        angles = yaw + angle_min + angle_increment * np.arange(len(r))
        limit = min(range_max, self.max_range)
        hit = np.isfinite(r) & (r > range_min) & (r < limit)          # beam stopped on something
        far = np.isposinf(r) | (np.isfinite(r) & (r >= limit))       # nothing within `limit`
        sx = x + self.sensor_offset * np.cos(yaw)                     # lidar position
        sy = y + self.sensor_offset * np.sin(yaw)

        # FREE: sample every beam every half cell, up to just before its end
        free_len = np.where(hit, r - self.res, np.where(far, limit, 0.0))
        steps = np.arange(0.0, limit, self.res / 2.0)
        inside = steps[None, :] < free_len[:, None]
        px = sx + steps[None, :] * np.cos(angles)[:, None]
        py = sy + steps[None, :] * np.sin(angles)[:, None]
        free = self.to_cells(px[inside], py[inside])

        # OCCUPIED: the end point of every beam that hit something
        occ = self.to_cells(sx + r[hit] * np.cos(angles[hit]), sy + r[hit] * np.sin(angles[hit]))
        free = np.setdiff1d(free, occ)            # never call a hit cell free in the same scan

        flat = self.logodds.reshape(-1)
        flat[free] += self.l_free
        flat[occ] += self.l_occ
        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)
        seen = self.seen.reshape(-1)
        seen[free] = True
        seen[occ] = True

    def to_occupancy(self):
        """Grid of values for a ROS map: -1 unknown, else 0..100 = chance of being occupied (%)."""
        p = 1.0 / (1.0 + np.exp(-self.logodds))
        out = np.full(self.logodds.shape, -1, dtype=np.int8)
        out[self.seen] = np.round(p[self.seen] * 100).astype(np.int8)
        return out
