"""Unit tests for the planning grid (planning_grid.py). Run without ROS."""
import math

import numpy as np

from factory_bot.planning_grid import PlanningGrid, inflate

RES = 0.05


def small_grid(zones=None):
    """A 3 x 2 m area: one wall cell at (row 20, col 30) = map (1.525, 1.025); a 0.5 m unknown
    strip along the top. Origin (0, 0)."""
    occ = np.zeros((40, 60), dtype=bool)
    unknown = np.zeros_like(occ)
    occ[20, 30] = True
    unknown[30:, :] = True
    return PlanningGrid(occ, unknown, RES, (0.0, 0.0), robot_radius=0.25, safety_margin=0.05,
                        soft_band=0.20, near_wall_cost=2.0, zones=zones or [], slow_cost=3.0)


def test_world_and_cell_conversions_agree():
    g = small_grid()
    row, col = g.world_to_cell(1.52, 1.03)
    assert (row, col) == (20, 30)
    x, y = g.cell_to_world(row, col)
    assert math.isclose(x, 1.525) and math.isclose(y, 1.025)


def test_inflation_blocks_robot_radius_plus_margin_only():
    g = small_grid()
    assert not g.passable(20, 30)          # the wall itself
    assert not g.passable(20, 36)          # 0.30 m away: inside radius + margin
    assert g.passable(20, 37)              # 0.35 m away: outside
    assert g.cost[20, 37] == 2.0           # ... but in the soft band near the wall
    assert g.cost[20, 45] == 1.0           # 0.75 m away: plain floor


def test_unknown_cells_are_blocked():
    g = small_grid()
    assert not g.passable(35, 5)


def test_forbidden_restricted_and_slow_zones():
    zones = [{"x_min": 0.0, "x_max": 0.5, "y_min": 0.0, "y_max": 0.5, "rule": "forbidden"},
             {"x_min": 2.5, "x_max": 3.0, "y_min": 0.0, "y_max": 0.5, "rule": "authorized_only"},
             {"x_min": 0.0, "x_max": 0.5, "y_min": 1.0, "y_max": 1.4, "rule": "slow"}]
    g = small_grid(zones)
    assert not g.passable(*g.world_to_cell(0.2, 0.2))                       # forbidden
    r, c = g.world_to_cell(2.7, 0.2)
    assert not g.passable(r, c) and g.passable(r, c, allow_restricted=True)  # QC lab
    assert g.cost[g.world_to_cell(0.2, 1.2)] == 3.0                          # slow zone


def test_nearest_passable_escapes_the_inflation_band():
    g = small_grid()
    cell = g.nearest_passable(20, 33)
    assert cell is not None and g.passable(*cell)
    assert math.hypot(cell[0] - 20, cell[1] - 30) > 6


def test_inflate_does_not_wrap_around_the_edges():
    m = np.zeros((10, 10), dtype=bool)
    m[0, 0] = True
    out = inflate(m, 2)
    assert out[0, 2] and out[2, 0] and not out[9, 9] and not out[0, 9]
