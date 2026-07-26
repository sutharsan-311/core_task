from core_task_controller.function import split_waypoints

WPS = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0], [4, 0, 0]]


def test_even_split_is_contiguous_halves():
    wps = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]]
    assert split_waypoints(wps, 2, 0) == [[0, 0, 0], [1, 0, 0]]
    assert split_waypoints(wps, 2, 1) == [[2, 0, 0], [3, 0, 0]]


def test_odd_split_front_gets_the_extra():
    assert split_waypoints(WPS, 2, 0) == [[0, 0, 0], [1, 0, 0], [2, 0, 0]]
    assert split_waypoints(WPS, 2, 1) == [[3, 0, 0], [4, 0, 0]]


def test_halves_cover_everything_once():
    combined = split_waypoints(WPS, 2, 0) + split_waypoints(WPS, 2, 1)
    assert combined == WPS


def test_single_robot_returns_all():
    assert split_waypoints(WPS, 1, 0) == WPS


def test_more_robots_than_waypoints_gives_empty_tail():
    assert split_waypoints([[0, 0, 0]], 2, 0) == [[0, 0, 0]]
    assert split_waypoints([[0, 0, 0]], 2, 1) == []


def test_empty_input():
    assert split_waypoints([], 2, 0) == []
    assert split_waypoints([], 2, 1) == []
