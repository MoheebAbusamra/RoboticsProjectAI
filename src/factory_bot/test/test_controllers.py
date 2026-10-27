"""Unit tests for the PID and Pure Pursuit followers (controllers.py) using the kinematic
test bench (track_sim.py). Run without ROS."""
import math

import numpy as np

from factory_bot.controllers import PIDFollower, PurePursuitFollower, make_follower, to_robot_frame
from factory_bot.track_sim import cross_track, simulate


def line(x0, y0, x1, y1, step=0.05):
    n = max(1, int(math.dist((x0, y0), (x1, y1)) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


L_PATH = line(0, 0, 3, 0) + line(3, 0, 3, 3)[1:]          # 3 m east, then 3 m north


def test_robot_frame_left_is_positive_y():
    assert np.allclose(to_robot_frame((1, 1, math.pi / 2), (1, 2)), (1, 0))   # ahead
    assert to_robot_frame((0, 0, 0), (1, 1))[1] > 0                           # left


def test_both_controllers_drive_an_l_shaped_path_to_the_goal():
    for follower in (PIDFollower(), PurePursuitFollower()):
        r = simulate(follower, L_PATH, (0, 0, 0))
        assert r["reached"], type(follower).__name__
        assert math.dist(r["trajectory"][-1][:2], L_PATH[-1]) <= 0.11
        assert cross_track(L_PATH, r["trajectory"]).max() < 0.30          # cuts the corner a bit, no more


def test_pure_pursuit_curvature_sign_and_straight_ahead():
    pp = PurePursuitFollower(regulate=False)
    v, w = pp.command(1.0, 0.0, 0.0, 0.25, 0.05)
    assert v == 0.25 and w == 0.0                                          # target dead ahead: no turn
    _, w_left = pp.command(0.4, 0.1, math.atan2(0.1, 0.4), 0.25, 0.05)
    assert math.isclose(w_left, 0.25 * 2 * 0.1 / (0.4 ** 2 + 0.1 ** 2))   # w = v * 2y/L^2, to the left


def test_target_behind_means_turn_on_the_spot():
    f = PurePursuitFollower()
    f.set_path(line(0, 0, -2, 0))                                          # path goes backwards
    v, w, done = f.step((0, 0, 0), 0.05)
    assert v == 0.0 and w != 0.0 and not done


def test_pid_integral_cannot_wind_up():
    pid = PIDFollower(ki=1.0, integral_limit=0.5)
    for _ in range(1000):
        pid.command(1.0, 1.0, 0.8, 0.25, 0.05)                             # a stubborn error
    assert abs(pid.integral) <= 0.5


def test_progress_only_moves_forward_on_a_path_that_loops_back():
    hairpin = line(0, 0, 2, 0) + line(2, 0, 2, 0.6)[1:] + line(2, 0.6, 0, 0.6)[1:]
    f = PurePursuitFollower()
    f.set_path(hairpin)
    f.target((1.0, 0.0, 0.0))
    first = f.index
    f.target((1.0, 0.1, 0.0))                                              # the return leg is only 0.5 m away
    assert f.index >= first and hairpin[f.index][1] < 0.3                  # still on the outgoing leg


def test_speed_limit_is_respected():
    f = make_follower("pure_pursuit", {"speed": 0.4})
    f.set_path(line(0, 0, 5, 0))
    v, _, _ = f.step((0, 0, 0), 0.05, speed_limit=0.15)
    assert v <= 0.15
