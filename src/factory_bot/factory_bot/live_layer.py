"""LiveLayer: remembers obstacles the lidar sees that are NOT on the saved map (a dropped pallet,
the forklift, a worker, the other robot), so the planner can route around them.
Plain Python + NumPy, no ROS.

Every scan: each lidar point (within max_range) is placed on the map at the robot's pose.
  close to a wall of the saved map (within wall_margin)  -> it IS that wall: ignored
  anywhere else                                           -> a live obstacle: its cell is stamped "seen now"
A cell counts as blocked while it was seen in the last memory_s seconds, so things that move
away disappear by themselves after a moment. The planner uses grid.with_blocked(layer.mask(now)).
"""
import math

import numpy as np

from factory_bot.planning_grid import inflate


class LiveLayer:
    def __init__(self, grid, memory_s=2.0, max_range=3.0, wall_margin=0.15, laser_x=0.10):
        self.grid = grid
        self.memory_s = memory_s              # s: how long a seen cell stays blocked
        self.max_range = max_range            # m: only nearby points matter (and it's cheaper)
        self.laser_x = laser_x                # m: lidar is this far ahead of the robot centre
        self.last_seen = np.full((grid.height, grid.width), -np.inf)
        # cells within wall_margin of a mapped wall: points there are the wall itself
        self.near_wall = inflate(grid.occupied | grid.unknown, wall_margin / grid.res + 1e-6)

    def update(self, pose, ranges, angle_min, angle_increment, range_min, range_max, now, laser_x=None):
        """Add one scan taken at robot pose (x, y, yaw) on the map. Returns how many cells were stamped."""
        r = np.asarray(ranges, dtype=float)
        angles = pose[2] + angle_min + angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, self.max_range))
        offset = self.laser_x if laser_x is None else laser_x
        sx = pose[0] + offset * math.cos(pose[2])
        sy = pose[1] + offset * math.sin(pose[2])
        xs, ys = sx + r[ok] * np.cos(angles[ok]), sy + r[ok] * np.sin(angles[ok])
        g = self.grid
        cols = np.floor((xs - g.origin[0]) / g.res).astype(int)
        rows = np.floor((ys - g.origin[1]) / g.res).astype(int)
        inside = (rows >= 0) & (rows < g.height) & (cols >= 0) & (cols < g.width)
        rows, cols = rows[inside], cols[inside]
        new = ~self.near_wall[rows, cols]                     # not explained by the saved map
        self.last_seen[rows[new], cols[new]] = now
        return int(new.sum())

    def mask(self, now):
        """Cells seen recently enough to count as obstacles."""
        return self.last_seen >= now - self.memory_s

    def front_unmapped_distance(self, pose, ranges, angle_min, angle_increment,
                                range_min, range_max, half_angle_deg=20.0, max_distance=0.8, laser_x=None):
        """Nearest current front return not explained by the saved map.

        Uses this scan, rather than remembered points, to check whether the way
        has cleared. 'Dynamic' here means unmapped, not a recognized object type.
        """
        r = np.asarray(ranges, dtype=float)
        angles = angle_min + angle_increment * np.arange(len(r))
        angles = np.arctan2(np.sin(angles), np.cos(angles))
        valid = (np.isfinite(r) & (r > range_min) & (r < range_max)
                 & (r <= max_distance) & (np.abs(angles) <= math.radians(half_angle_deg)))
        r, angles = r[valid], angles[valid] + pose[2]
        offset = self.laser_x if laser_x is None else laser_x
        sx = pose[0] + offset * math.cos(pose[2])
        sy = pose[1] + offset * math.sin(pose[2])
        cols = np.floor((sx + r * np.cos(angles) - self.grid.origin[0]) / self.grid.res).astype(int)
        rows = np.floor((sy + r * np.sin(angles) - self.grid.origin[1]) / self.grid.res).astype(int)
        inside = ((rows >= 0) & (rows < self.grid.height) & (cols >= 0) & (cols < self.grid.width))
        r, rows, cols = r[inside], rows[inside], cols[inside]
        unmapped = ~self.near_wall[rows, cols]
        return float(r[unmapped].min()) if unmapped.any() else math.inf

    def overlay(self, now):
        """The planning grid with the live obstacles added (inflated like walls)."""
        return self.grid.with_blocked(self.mask(now))
