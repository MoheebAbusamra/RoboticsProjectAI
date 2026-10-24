"""PlanningGrid: turns the saved map into the grid the planners search (Lab 4: "a planning
grid describes where the robot's CENTRE may go"). Plain Python + NumPy, no ROS.

Every cell gets a COST:
  1.0          free floor
  > 1.0        allowed but discouraged: close to walls (soft band), slow zone
  inf          never: walls, inflation (robot radius + margin), unknown, forbidden zones
Grid convention (same as the course's Lab 4 code): grid[row, col]; row grows with map y,
col grows with map x; cell (0, 0) has its corner at the map origin.
"""
import copy
import math
import os

import numpy as np
import yaml

from factory_bot.compare_maps import read_pgm


def disk_offsets(radius_cells):
    """All (dr, dc) within a circle of radius_cells."""
    r = int(math.ceil(radius_cells))
    return [(dr, dc) for dr in range(-r, r + 1) for dc in range(-r, r + 1)
            if dr * dr + dc * dc <= radius_cells * radius_cells]


def inflate(mask, radius_cells):
    """Grow every True cell into a disk of radius_cells (no wrap-around at the edges)."""
    out = mask.copy()
    h, w = mask.shape
    for dr, dc in disk_offsets(radius_cells):
        src = mask[max(-dr, 0):h + min(-dr, 0), max(-dc, 0):w + min(-dc, 0)]
        dst = out[max(dr, 0):h + min(dr, 0), max(dc, 0):w + min(dc, 0)]
        dst |= src
    return out


class PlanningGrid:
    def __init__(self, occupied, unknown, resolution, origin, robot_radius=0.25, safety_margin=0.05,
                 soft_band=0.20, near_wall_cost=2.0, zones=None, slow_cost=2.0):
        self.res = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.height, self.width = occupied.shape
        self.occupied, self.unknown = occupied, unknown

        # cells the centre must stay away (+1e-6 so 0.30 / 0.05 counts as 6, not 5.999...)
        hard = (robot_radius + safety_margin) / self.res + 1e-6
        self.hard = hard    
        self.inflated = inflate(occupied, hard) & ~occupied
        self.near_wall = inflate(occupied, hard + soft_band / self.res) & ~occupied & ~self.inflated

        self.forbidden = np.zeros_like(occupied)
        self.restricted = np.zeros_like(occupied)                     # authorized_only zones
        self.slow = np.zeros_like(occupied)
        self.zones = zones or []
        for z in self.zones:
            m = self.rect_mask(z["x_min"], z["x_max"], z["y_min"], z["y_max"])
            if z["rule"] == "forbidden":
                self.forbidden |= m
            elif z["rule"] == "authorized_only":
                self.restricted |= m
            elif z["rule"] == "slow":
                self.slow |= m
        self.forbidden = inflate(self.forbidden, hard)    # keep the robot's whole BODY out, not just its centre
        self.cost = np.ones(occupied.shape)
        self.cost[self.near_wall] = near_wall_cost
        self.cost[self.slow] = np.maximum(self.cost[self.slow], slow_cost)
        self.cost[occupied | self.inflated | unknown | self.forbidden] = np.inf

    # ------------------------------------------------------------ building it
    @classmethod
    def from_files(cls, map_yaml, planning_yaml=None, zones_yaml=None):
        """Read factory_map.yaml (+ .pgm), the robot/cost settings and the zones."""
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        image = meta["image"]
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image)
        pix = read_pgm(image).astype(float)
        p = pix / 255.0 if meta.get("negate", 0) else (255.0 - pix) / 255.0   # chance occupied
        occupied = (p > float(meta.get("occupied_thresh", 0.65)))[::-1, :]   # flip: row 0 = bottom
        free = (p < float(meta.get("free_thresh", 0.25)))[::-1, :]
        settings = {}
        if planning_yaml:
            with open(planning_yaml) as f:
                settings = (yaml.safe_load(f) or {}).get("grid", {})
        zones = []
        if zones_yaml:
            with open(zones_yaml) as f:
                zones = [dict(name=n, **z) for n, z in (yaml.safe_load(f) or {})["zones"].items()]
        return cls(occupied, ~occupied & ~free, meta["resolution"], meta["origin"][:2], zones=zones, **settings)

    def rect_mask(self, x_min, x_max, y_min, y_max):
        """Cells whose centre lies inside a rectangle given in metres (map frame)."""
        xs = self.origin[0] + (np.arange(self.width) + 0.5) * self.res
        ys = self.origin[1] + (np.arange(self.height) + 0.5) * self.res
        return ((ys >= y_min) & (ys <= y_max))[:, None] & ((xs >= x_min) & (xs <= x_max))[None, :]

    # ------------------------------------------------------------ using it
    def world_to_cell(self, x, y):
        """Metres (map frame) -> (row, col)."""
        return (int(math.floor((y - self.origin[1]) / self.res)),
                int(math.floor((x - self.origin[0]) / self.res)))

    def cell_to_world(self, row, col):
        """(row, col) -> metres of the cell's CENTRE."""
        return (self.origin[0] + (col + 0.5) * self.res, self.origin[1] + (row + 0.5) * self.res)

    def in_bounds(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def passable(self, row, col, allow_restricted=False):
        """Can the robot's centre be in this cell?"""
        if not self.in_bounds(row, col) or math.isinf(self.cost[row, col]):
            return False
        return allow_restricted or not self.restricted[row, col]

    def with_blocked(self, extra):
        """A copy of this grid where the cells in `extra` (bool array, e.g. live obstacles seen by
        the lidar) are also blocked, widened by the robot's radius + margin like walls are.
        The original grid is not changed."""
        g = copy.copy(self)                                           # shares the big arrays ...
        g.cost = self.cost.copy()                                     # ... except the costs
        g.cost[inflate(extra, self.hard)] = np.inf
        return g

    def nearest_passable(self, row, col, max_cells=20, allow_restricted=False):
        """Closest passable cell (e.g. a robot that ended up inside the inflation band)."""
        for dr, dc in sorted(disk_offsets(max_cells), key=lambda o: o[0] ** 2 + o[1] ** 2):
            if self.passable(row + dr, col + dc, allow_restricted):
                return row + dr, col + dc
        return None
