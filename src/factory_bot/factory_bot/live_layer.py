"""LiveLayer: remembers obstacles the lidar sees that are NOT on the saved map (a dropped pallet,
the forklift, a worker, the other robot), so the planner can route around them.
Plain Python + NumPy, no ROS.

Every scan: each lidar point (within max_range) is placed on the map at the robot's pose.
  close to a wall of the saved map (within wall_margin)  -> it IS that wall: ignored
  anywhere else                                           -> a live obstacle: its cell is stamped "seen now"
A cell counts as blocked while it was seen in the last memory_s seconds, so things that move
away disappear by themselves after a moment. The planner uses grid.with_blocked(layer.mask(now)).
AI-assisted (Claude): cite in report."""
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

    def update(self, pose, ranges, angle_min, angle_increment, range_min, range_max, now):
        """Add one scan taken at robot pose (x, y, yaw) on the map. Returns how many cells were stamped."""
        r = np.asarray(ranges, dtype=float)
        angles = pose[2] + angle_min + angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, self.max_range))
        sx = pose[0] + self.laser_x * math.cos(pose[2])
        sy = pose[1] + self.laser_x * math.sin(pose[2])
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

    def overlay(self, now):
        """The planning grid with the live obstacles added (inflated like walls)."""
        return self.grid.with_blocked(self.mask(now))
