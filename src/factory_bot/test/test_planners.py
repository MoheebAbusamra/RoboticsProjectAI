"""Unit tests for BFS / Dijkstra / A* (planners.py). Run without ROS."""
import math

import numpy as np

from factory_bot.planning_grid import PlanningGrid
from factory_bot.planners import path_is_valid, plan

RES = 0.1                                          # 10 cm cells keep the numbers easy


def grid_with(walls=(), zones=(), slow_cost=10.0, size=(30, 30)):
    """Test grid with NO inflation (robot radius 0), so walls are exactly where we put them.
    walls: list of (row, col) cells. Cell (r, c) centre = ((c + 0.5) * 0.1, (r + 0.5) * 0.1)."""
    occ = np.zeros(size, dtype=bool)
    for r, c in walls:
        occ[r, c] = True
    return PlanningGrid(occ, np.zeros_like(occ), RES, (0.0, 0.0), robot_radius=0.0, safety_margin=0.0,
                        soft_band=0.0, zones=list(zones), slow_cost=slow_cost)


def xy(row, col):
    return ((col + 0.5) * RES, (row + 0.5) * RES)


def test_all_three_find_a_valid_route_and_astar_expands_least():
    g = grid_with()
    res = {a: plan(g, xy(2, 2), xy(25, 20), a) for a in ("bfs", "dijkstra", "astar")}
    for r in res.values():
        assert r.success and path_is_valid(g, r.path)
        assert r.path[0] == (2, 2) and r.path[-1] == (25, 20)
    assert math.isclose(res["dijkstra"].cost, res["astar"].cost)        # both optimal
    assert res["astar"].expanded < res["dijkstra"].expanded             # A* searches less


def test_route_goes_through_the_gap_in_a_wall():
    wall = [(15, c) for c in range(30) if c != 25]                       # gap at column 25
    g = grid_with(walls=wall)
    r = plan(g, xy(5, 5), xy(25, 5), "astar")
    assert r.success and path_is_valid(g, r.path)
    assert (15, 25) in r.path


def test_no_route_is_reported_not_invented():
    wall = [(15, c) for c in range(30)]                                  # complete wall
    g = grid_with(walls=wall)
    for a in ("bfs", "dijkstra", "astar"):
        r = plan(g, xy(5, 5), xy(25, 5), a)
        assert not r.success and "no route" in r.reason and r.path == []


def test_costs_change_dijkstra_and_astar_but_not_bfs():
    # an expensive "slow" band across the straight line, with a free way round on the right
    slow = {"x_min": 0.0, "x_max": 1.2, "y_min": 1.4, "y_max": 1.6, "rule": "slow"}
    g = grid_with(zones=[slow], slow_cost=10.0)
    bfs = plan(g, xy(5, 5), xy(25, 5), "bfs")
    ast = plan(g, xy(5, 5), xy(25, 5), "astar")
    crosses = lambda p: any(g.slow[r, c] for r, c in p)                  # noqa: E731
    assert crosses(bfs.path)                                             # BFS: fewest moves, straight on
    assert not crosses(ast.path)                                         # A*: goes round the slow band
    assert ast.cost < bfs.cost


def test_no_corner_cutting_between_two_diagonal_walls():
    g = grid_with(walls=[(10, 11), (11, 10)])                            # two cells touching at a corner
    r = plan(g, xy(10, 10), xy(11, 11), "astar")                         # diagonal neighbours
    assert r.success and len(r.path) > 2                                 # had to go round, not squeeze through


def test_qc_lab_needs_permission():
    qc = {"x_min": 2.0, "x_max": 3.0, "y_min": 2.0, "y_max": 3.0, "rule": "authorized_only"}
    g = grid_with(zones=[qc])
    goal = xy(25, 25)                                                    # inside the QC lab
    assert not plan(g, xy(2, 2), goal, "astar", snap_cells=0).success
    assert plan(g, xy(2, 2), goal, "astar", allow_restricted=True).success


def test_start_inside_a_wall_band_is_moved_to_free_space():
    g = grid_with(walls=[(5, 5)])
    r = plan(g, xy(5, 5), xy(20, 20), "dijkstra")                        # start ON the wall cell
    assert r.success and r.path[0] != (5, 5) and g.passable(*r.path[0])
