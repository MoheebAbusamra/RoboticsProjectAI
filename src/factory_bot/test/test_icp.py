"""Unit tests for scan-to-map ICP (icp.py). Run without ROS."""
import math

import numpy as np

from factory_bot.icp import (WallLookup, best_rigid_transform, icp_localize, relocalize, scan_to_points,
                             transform)

RES = 0.05


def make_room():
    """A 6 x 4 m room (walls 1 cell thick) with an off-centre box, so no pose looks like another."""
    occ = np.zeros((90, 130), dtype=bool)              # rows = y, cols = x, origin (-0.25, -0.25)
    occ[5, 5:126] = occ[84, 5:126] = True              # bottom / top walls  (y = 0, y = 4)
    occ[5:85, 5] = occ[5:85, 125] = True               # left / right walls  (x = 0, x = 6)
    occ[25:45, 85:95] = True                           # a 0.5 x 1.0 m box near (4.25, 1.5)
    return occ, WallLookup(occ, RES, (-0.25, -0.25))


def fake_scan(occ, pose, n=360, max_range=12.0):
    """Ray-march through the grid from the lidar to make a realistic scan."""
    x, y, yaw = pose
    sx, sy = x + 0.10 * math.cos(yaw), y + 0.10 * math.sin(yaw)
    ranges = []
    for i in range(n):
        a = yaw - math.pi + 2 * math.pi * i / n
        r = 0.0
        while r < max_range:
            r += RES / 2
            col = int(math.floor((sx + r * math.cos(a) + 0.25) / RES))
            row = int(math.floor((sy + r * math.sin(a) + 0.25) / RES))
            if not (0 <= row < occ.shape[0] and 0 <= col < occ.shape[1]):
                r = float("inf")
                break
            if occ[row, col]:
                break
        ranges.append(r if r < max_range else float("inf"))
    return ranges


def points_of(ranges):
    return scan_to_points(ranges, -math.pi, 2 * math.pi / len(ranges), 0.12, 12.0)


def test_best_rigid_transform_recovers_a_known_motion():
    rng = np.random.default_rng(1)
    a = rng.uniform(-2, 2, (50, 2))
    c, s = math.cos(0.3), math.sin(0.3)
    b = a @ np.array([[c, s], [-s, c]]) + np.array([0.5, -0.2])
    rot, t = best_rigid_transform(a, b)
    assert abs(math.atan2(rot[1, 0], rot[0, 0]) - 0.3) < 1e-9
    assert np.allclose(t, [0.5, -0.2])


def test_scan_to_points_drops_bad_readings_and_adds_laser_offset():
    ranges = [1.0, float("inf"), float("nan"), 0.05]
    pts = scan_to_points(ranges, 0.0, 0.1, 0.12, 12.0, laser_x=0.10, step=1)
    assert pts.shape == (1, 2) and np.allclose(pts[0], [1.10, 0.0])


def test_icp_corrects_a_wrong_guess():
    occ, walls = make_room()
    true = (2.0, 2.0, 0.4)
    scan = points_of(fake_scan(occ, true))
    pose, info = icp_localize(scan, (2.2, 1.85, 0.4 + math.radians(6)), walls)
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.03
    assert abs(pose[2] - true[2]) < math.radians(1)
    assert info["matched"] > 0.9 and info["rms"] < 0.03


def test_points_that_are_not_walls_are_ignored():
    occ, walls = make_room()
    true = (2.0, 2.0, 0.0)
    ranges = fake_scan(occ, true)
    for i in range(170, 190):                       # something 0.8 m in front (a forklift)
        ranges[i] = 0.8
    pose, info = icp_localize(points_of(ranges), (2.1, 2.1, 0.05), walls)
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.03
    assert info["matched"] < 1.0                    # the forklift points found no wall partner


def test_a_hopeless_guess_is_reported_as_poor():
    occ, walls = make_room()
    scan = points_of(fake_scan(occ, (2.0, 2.0, 0.0)))
    _, info = icp_localize(scan, (4.5, 0.6, 2.5), walls)   # far away AND turned: little overlap
    assert info["matched"] < 0.6 or info["rms"] > 0.05


def test_transform_rotates_then_moves():
    pts = transform(np.array([[1.0, 0.0]]), (1.0, 2.0, math.pi / 2))
    assert np.allclose(pts, [[1.0, 3.0]])


def test_relocalize_finds_a_robot_that_was_pushed_far_away():
    occ, walls = make_room()
    true = (2.0, 2.0, 0.3)
    scan = points_of(fake_scan(occ, true))
    guess = (3.1, 1.3, 0.3 + 0.45)                          # 1.3 m and 26 deg wrong: normal ICP gives up
    _, plain = icp_localize(scan, guess, walls)
    assert plain["matched"] < 0.5 or plain["rms"] > 0.08
    pose, info = relocalize(scan, guess, walls)
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.05
    assert abs(pose[2] - true[2]) < math.radians(2) and info["matched"] > 0.9
