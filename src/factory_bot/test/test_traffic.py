"""Unit tests for the traffic rules (traffic.py). Run without ROS."""
import math

import numpy as np

from factory_bot.planning_grid import PlanningGrid
from factory_bot.planners import plan
from factory_bot.traffic import RobotView, TrafficRules, closest, find_aside_spot, right_of_way_order

RES = 0.05


def hall():
    """10 x 4 m hall, walls all round, origin (0, 0)."""
    occ = np.zeros((80, 200), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return PlanningGrid(occ, np.zeros_like(occ), RES, (0.0, 0.0))


def line(x0, x1, y=2.0):
    """A straight route along y, one point every 5 cm."""
    n = int(abs(x1 - x0) / RES) + 1
    return [(x0 + (x1 - x0) * i / (n - 1), y) for i in range(n)]


def driving(name, x, x_goal, y=2.0, **kw):
    yaw = 0.0 if x_goal > x else math.pi
    return RobotView(name, (x, y, yaw), True, line(x, x_goal, y), **kw)


def standing(name, x, y=2.0, **kw):
    return RobotView(name, (x, y, 0.0), False, [], **kw)


def test_priority_order():
    views = {"robot1": driving("robot1", 1, 9, job_priority="routine", fleet_state="TO_PICKUP"),
             "robot2": driving("robot2", 9, 1, job_priority="emergency", fleet_state="TO_PICKUP")}
    assert right_of_way_order(views) == ["robot2", "robot1"]                 # emergency first
    views["robot2"] = driving("robot2", 9, 1)                                # nothing to do, going home
    views["robot1"] = driving("robot1", 1, 9, job_priority="routine", fleet_state="TO_DROPOFF")
    assert right_of_way_order(views) == ["robot1", "robot2"]
    views["robot1"] = driving("robot1", 1, 9)
    assert right_of_way_order(views) == ["robot1", "robot2"]                 # tie -> by name


def test_head_on_the_lower_priority_robot_waits():
    rules = TrafficRules(hall())
    cmd = rules.update({"robot1": driving("robot1", 3.0, 9.0), "robot2": driving("robot2", 6.0, 1.0)}, now=0.0)
    assert cmd["robot1"]["cmd"] == "go"
    assert cmd["robot2"]["cmd"] == "wait" and cmd["robot2"]["because"] == "robot1"


def test_the_emergency_robot_goes_first():
    rules = TrafficRules(hall())
    cmd = rules.update({"robot1": driving("robot1", 3.0, 9.0, job_priority="routine"),
                        "robot2": driving("robot2", 6.0, 1.0, job_priority="emergency")}, now=0.0)
    assert cmd["robot2"]["cmd"] == "go" and cmd["robot1"]["cmd"] == "wait"


def test_no_waiting_when_far_apart_or_one_behind_the_other():
    rules = TrafficRules(hall())
    cmd = rules.update({"robot1": driving("robot1", 1.0, 3.0), "robot2": driving("robot2", 9.0, 7.0)}, now=0.0)
    assert cmd["robot1"]["cmd"] == cmd["robot2"]["cmd"] == "go"              # 4 m apart, routes don't meet
    cmd = rules.update({"robot1": driving("robot1", 3.0, 9.0), "robot2": driving("robot2", 2.0, 8.0)}, now=1.0)
    assert cmd["robot1"]["cmd"] == cmd["robot2"]["cmd"] == "go"              # same way: the rear one follows


def test_a_waiting_robot_goes_once_the_way_has_been_clear_for_a_moment():
    rules = TrafficRules(hall(), release_s=1.0)
    rules.update({"robot1": driving("robot1", 3.0, 9.0), "robot2": driving("robot2", 6.0, 1.0)}, now=0.0)
    passed = {"robot1": driving("robot1", 8.5, 9.0), "robot2": driving("robot2", 6.0, 1.0)}   # robot1 went by
    assert rules.update(passed, now=0.5)["robot2"]["cmd"] == "wait"          # clear for only 0 s
    assert rules.update(passed, now=1.0)["robot2"]["cmd"] == "wait"          # 0.5 s
    assert rules.update(passed, now=1.6)["robot2"]["cmd"] == "go"            # 1.1 s -> go


def test_a_standing_robot_that_blocks_another_makes_room():
    rules = TrafficRules(hall(), blocked_s=2.0)
    views = {"robot1": driving("robot1", 3.0, 9.0, blocked_s=0.5), "robot2": standing("robot2", 3.7)}
    assert rules.update(views, now=0.0)["robot2"]["cmd"] == "go"             # only blocked 0.5 s: patience
    views["robot1"].blocked_s = 2.5
    cmd = rules.update(views, now=2.0)
    assert cmd["robot2"]["cmd"] == "aside" and cmd["robot2"]["because"] == "robot1"
    spot = cmd["robot2"]["spot"]
    assert closest(views["robot1"].route, [spot]) >= 0.9                     # clear of robot1's route
    assert rules.update(views, now=3.0)["robot2"]["cmd"] == "aside"          # stays there ...
    views["robot1"] = driving("robot1", 8.8, 9.0)
    assert rules.update(views, now=20.0)["robot2"]["cmd"] == "go"            # ... until robot1 has passed


def test_station_busy():
    rules = TrafficRules(hall())
    views = {"robot1": driving("robot1", 4.0, 5.5), "robot2": standing("robot2", 5.5, fleet_state="UNLOADING")}
    cmd = rules.update(views, now=0.0)
    assert cmd["robot1"]["cmd"] == "wait" and cmd["robot1"]["rule"] == "station busy"
    views["robot2"].fleet_state = "IDLE"                                     # finished, just parked there
    cmd = TrafficRules(hall()).update(views, now=0.0)
    assert cmd["robot2"]["cmd"] == "aside" and cmd["robot1"]["cmd"] == "go"


def test_aside_spot_never_needs_driving_through_the_other_robot():
    # a 1 m wide dead-end corridor (x from 0 to 6) opening into a big room on the right
    occ = np.zeros((100, 200), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[:40, :120] = True
    occ[60:, :120] = True                                    # corridor: y 2.0 .. 3.0, x 0 .. 6
    g = PlanningGrid(occ, np.zeros_like(occ), RES, (0.0, 0.0), robot_radius=0.2, safety_margin=0.0)
    # the other robot is already IN the corridor, coming towards us: we can only back into the dead end
    spot = find_aside_spot(g, (3.0, 2.5), line(5.0, 0.8, y=2.5), (5.0, 2.5))
    assert spot is None                                      # no room anywhere: nothing sensible to ask
    # the other robot is still in the room: we back out of the corridor and wait beside its route
    route = line(8.0, 0.8, y=2.5)
    spot = find_aside_spot(g, (3.0, 2.5), route, (8.0, 2.5))
    assert spot is not None and spot[0] > 6.0                # out in the room ...
    assert closest(route, [spot]) >= 0.9                     # ... and off its route


def test_active_status_before_path_does_not_crash():
    """The status callback may run before the first Path callback."""
    rules = TrafficRules(hall())
    view = RobotView("robot1", (1.0, 2.0, 0.0), True, [])
    assert rules.update({"robot1": view}, now=0.0)["robot1"]["cmd"] == "go"
    view.route = line(1.0, 9.0)
    assert rules.update({"robot1": view}, now=1.0)["robot1"]["cmd"] == "go"


def test_aside_search_cannot_squeeze_through_blocked_diagonal_corners():
    """Old flood returned (3.5, 3.5), but A* could not reach that cell."""
    occ = np.ones((5, 5), dtype=bool)
    for i in (1, 2, 3):
        occ[i, i] = False
    g = PlanningGrid(occ, np.zeros_like(occ), 1.0, (0.0, 0.0),
                     robot_radius=0.0, safety_margin=0.0, soft_band=0.0)
    assert not plan(g, (1.5, 1.5), (3.5, 3.5), snap_cells=0).success
    assert find_aside_spot(g, (1.5, 1.5), [(1.5, 1.5)], (0.5, 0.5)) is None


def test_aside_spot_is_reachable_by_the_actual_planner():
    g = hall()
    spot = find_aside_spot(g, (3.7, 2.0), line(3.0, 9.0), (3.0, 2.0))
    assert spot is not None
    assert plan(g, (3.7, 2.0), spot, snap_cells=0).success


def test_aside_search_limit_counts_diagonal_distance_in_metres():
    # Exiting a >0.9 m exclusion disk is impossible within 0.8 m, including
    # diagonals. The old equal-step flood could incorrectly find an exit.
    assert find_aside_spot(hall(), (3.0, 2.0), [(3.0, 2.0)], (8.0, 2.0), max_m=0.8) is None
