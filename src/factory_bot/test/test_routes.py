
import math

import pytest

from factory_bot.routes import Route


def test_goes_there_and_back():
    r = Route([[0, 0], [4, 0]], speed=1.0)       # 4 m long, 1 m/s
    assert r.at(0)[:2] == pytest.approx((0, 0))  # start
    assert r.at(2)[:2] == pytest.approx((2, 0))  # halfway out
    assert r.at(4)[:2] == pytest.approx((4, 0))  # far end
    assert r.at(6)[:2] == pytest.approx((2, 0))  # halfway back
    assert r.at(8)[:2] == pytest.approx((0, 0))  # home again
    assert r.at(6)[2] == pytest.approx(math.pi)  # facing backwards on the way back


def test_route_with_a_corner():
    r = Route([[0, 0], [2, 0], [2, 2]], speed=1.0)
    assert r.at(3)[:2] == pytest.approx((2, 1))  # 2 m along x, then 1 m up
