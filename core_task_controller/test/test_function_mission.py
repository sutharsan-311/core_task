from core_task_controller.function import validate_mission


def test_valid_mapping():
    ok, _ = validate_mission({'mode': 'mapping', 'map_name': 'warehouse'})
    assert ok is True


def test_valid_navigation_with_loops():
    ok, _ = validate_mission(
        {'mode': 'navigation', 'map_name': 'warehouse', 'loops': 2})
    assert ok is True


def test_bad_mode():
    ok, reason = validate_mission({'mode': 'fly', 'map_name': 'w'})
    assert ok is False and 'mode' in reason


def test_missing_map_name():
    ok, reason = validate_mission({'mode': 'mapping'})
    assert ok is False and 'map_name' in reason


def test_bad_loops():
    ok, reason = validate_mission(
        {'mode': 'navigation', 'map_name': 'w', 'loops': 0})
    assert ok is False and 'loops' in reason


def test_not_a_dict():
    ok, _ = validate_mission("nope")
    assert ok is False


def test_valid_squad_navigation():
    ok, _ = validate_mission({'mode': 'squad_navigation', 'map_name': 'warehouse'})
    assert ok is True


def test_valid_navigation_with_robot_index():
    ok, _ = validate_mission(
        {'mode': 'navigation', 'map_name': 'w', 'robots': 2, 'robot_index': 1})
    assert ok is True


def test_robot_index_out_of_range():
    ok, reason = validate_mission(
        {'mode': 'navigation', 'map_name': 'w', 'robots': 2, 'robot_index': 2})
    assert ok is False and 'robot_index' in reason


def test_bad_robots_count():
    ok, reason = validate_mission(
        {'mode': 'navigation', 'map_name': 'w', 'robots': 0, 'robot_index': 0})
    assert ok is False and 'robots' in reason
