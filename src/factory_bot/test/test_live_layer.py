"""Unit tests for live obstacles + replanning (live_layer.py, PlanningGrid.with_blocked).
Run without ROS."""
import math

import numpy as np

from factory_bot.live_layer import LiveLayer
from factory_bot.planners import plan
from factory_bot.planning_grid import PlanningGrid

RES = 0.05


def room():
    """6 x 6 m room, walls on all four sides (1 cell thick), origin (0, 0), normal inflation."""
    occ = np.zeros((120, 120), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return PlanningGrid(occ, np.zeros_like(occ), RES, (0.0, 0.0))


def scan_of_points(pose, points, n=360):
    """A fake scan that sees only the given map points (everything else: nothing)."""
    ranges = [math.inf] * n
    sx, sy = pose[0] + 0.10 * math.cos(pose[2]), pose[1] + 0.10 * math.sin(pose[2])
    for x, y in points:
        a = math.atan2(y - sy, x - sx) - pose[2]
        i = int(round((a + math.pi) / (2 * math.pi / n))) % n
        ranges[i] = min(ranges[i], math.hypot(x - sx, y - sy))
    return ranges


def update(layer, pose, points, now):
    return layer.update(pose, scan_of_points(pose, points), -math.pi, 2 * math.pi / 360, 0.12, 12.0, now)


def box(cx, cy, half=0.25):
    """Points on the outline of a square box (like a lidar would see a pallet)."""
    t = np.linspace(-half, half, 15)
    return ([(cx + v, cy - half) for v in t] + [(cx + v, cy + half) for v in t] +
            [(cx - half, cy + v) for v in t] + [(cx + half, cy + v) for v in t])


def test_points_on_mapped_walls_are_ignored():
    g = room()
    layer = LiveLayer(g)
    wall_points = [(x, 0.03) for x in np.linspace(1.0, 2.0, 20)]       # the bottom wall
    assert update(layer, (1.5, 1.0, -math.pi / 2), wall_points, now=0.0) == 0
    assert not layer.mask(0.0).any()


def test_a_box_in_free_space_becomes_an_obstacle():
    g = room()
    layer = LiveLayer(g)
    assert update(layer, (1.0, 3.0, 0.0), box(2.5, 3.0), now=0.0) > 0
    r, c = g.world_to_cell(2.25, 3.0)                                 # a point on the box outline
    assert layer.mask(0.0)[r, c]
    assert not layer.overlay(0.0).passable(*g.world_to_cell(2.0, 3.0))   # inflated around it
    assert g.passable(*g.world_to_cell(2.0, 3.0))                     # the saved grid is unchanged


def test_obstacles_are_forgotten_after_the_memory_time():
    g = room()
    layer = LiveLayer(g, memory_s=2.0)
    update(layer, (1.0, 3.0, 0.0), box(2.5, 3.0), now=10.0)
    assert layer.mask(11.5).any()
    assert not layer.mask(12.5).any()


def test_far_points_are_ignored():
    g = room()
    layer = LiveLayer(g, max_range=3.0)
    assert update(layer, (0.5, 3.0, 0.0), box(4.5, 3.0), now=0.0) == 0     # about 4 m away


def test_replanning_goes_around_the_obstacle():
    g = room()
    layer = LiveLayer(g)
    start, goal = (1.0, 3.0), (5.0, 3.0)
    straight = plan(g, start, goal, "astar")
    update(layer, (1.0, 3.0, 0.0), box(2.5, 3.0, half=0.4), now=0.0)  # a big pallet on the line
    around = plan(layer.overlay(0.0), start, goal, "astar")
    assert straight.success and around.success
    assert around.length_m > straight.length_m + 0.2                  # it had to detour
    blocked = layer.overlay(0.0)
    assert all(blocked.passable(r, c) for r, c in around.path)        # and never enters the obstacle
