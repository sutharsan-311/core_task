"""Unit tests for the target-mover path math."""
import math

from core_task_perception.path import (
    loop_length, ping_pong, pose_at, segment_lengths,
)

SQUARE = [(0, 0), (4, 0), (4, 4), (0, 4)]   # 4x4 loop, perimeter 16


def test_loop_length_closes():
    assert loop_length(SQUARE) == 16.0


def test_segment_lengths():
    assert segment_lengths(SQUARE) == [4.0, 4.0, 4.0, 4.0]


def test_pose_at_start():
    x, y, yaw = pose_at(SQUARE, 0.0)
    assert (x, y) == (0, 0)
    assert math.isclose(yaw, 0.0)          # heading toward (4,0): +x


def test_pose_at_mid_first_segment():
    x, y, _ = pose_at(SQUARE, 2.0)
    assert math.isclose(x, 2.0) and math.isclose(y, 0.0)


def test_pose_at_second_segment_turns():
    x, y, yaw = pose_at(SQUARE, 6.0)       # 2 m up the second edge
    assert math.isclose(x, 4.0) and math.isclose(y, 2.0)
    assert math.isclose(yaw, math.pi / 2)  # heading +y


def test_pose_at_wraps():
    a = pose_at(SQUARE, 1.0)
    b = pose_at(SQUARE, 1.0 + 16.0)        # one full loop later
    assert math.isclose(a[0], b[0]) and math.isclose(a[1], b[1])


def test_pose_at_handles_degenerate_loop():
    x, y, yaw = pose_at([(3, 3)], 5.0)     # single point, zero length
    assert (x, y) == (3, 3) and yaw == 0.0


def test_ping_pong_avoids_closing_diagonal():
    wps = [(0, 0), (1, 0), (2, 0), (3, 0)]
    pp = ping_pong(wps)
    # out then back through the interior: 0,1,2,3,2,1 - closes 1->0, an edge.
    assert pp == [(0, 0), (1, 0), (2, 0), (3, 0), (2, 0), (1, 0)]


def test_ping_pong_short_lists_pass_through():
    assert ping_pong([(0, 0)]) == [(0, 0)]
    assert ping_pong([(0, 0), (1, 1)]) == [(0, 0), (1, 1)]
