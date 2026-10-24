"""Unit tests for the occupancy grid mapper (ogm.py). Run without ROS. """
import math

from factory_bot.ogm import OccupancyGridMapper

ANGLE_MIN = -math.pi
INC = 2 * math.pi / 360          # index 180 points straight ahead (angle 0)


def one_beam(distance):
    """A scan where only the straight-ahead beam measured something."""
    ranges = [float("nan")] * 360
    ranges[180] = distance
    return ranges


def cell_value(m, x, y):
    col = int(math.floor((x - m.origin[0]) / m.res))
    row = int(math.floor((y - m.origin[1]) / m.res))
    return int(m.to_occupancy()[row, col])


def new_map():
    return OccupancyGridMapper(width_m=6, height_m=6, origin=(-3, -3), sensor_offset=0.0)


def test_wall_ahead_is_occupied_and_path_is_free():
    m = new_map()
    for _ in range(3):
        m.update(0, 0, 0, one_beam(2.0), ANGLE_MIN, INC, 0.12, 12.0)
    assert cell_value(m, 2.01, 0.01) > 65        # where the beam stopped
    assert cell_value(m, 1.0, 0.01) < 25         # where the beam passed


def test_cells_not_looked_at_stay_unknown():
    m = new_map()
    m.update(0, 0, 0, one_beam(2.0), ANGLE_MIN, INC, 0.12, 12.0)
    assert cell_value(m, 0.01, 2.0) == -1        # to the left: no beam went there


def test_robot_heading_rotates_the_beam():
    m = new_map()
    m.update(0, 0, math.pi / 2, one_beam(2.0), ANGLE_MIN, INC, 0.12, 12.0)   # facing +y
    assert cell_value(m, 0.01, 2.01) > 50
    assert cell_value(m, 2.01, 0.01) == -1


def test_evidence_is_clamped():
    m = new_map()
    for _ in range(50):
        m.update(0, 0, 0, one_beam(2.0), ANGLE_MIN, INC, 0.12, 12.0)
    assert m.logodds.max() <= m.l_max and m.logodds.min() >= m.l_min


def test_no_return_marks_free_up_to_max_range():
    m = new_map()
    ranges = [float("nan")] * 360
    ranges[180] = float("inf")
    m.update(0, 0, 0, ranges, ANGLE_MIN, INC, 0.12, 12.0)
    assert cell_value(m, 2.5, 0.01) < 50 and cell_value(m, 2.5, 0.01) != -1
