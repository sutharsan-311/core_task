"""Unit tests for the pure approach control law."""
from core_task_perception.approach import steering_command


def _report(found=True, cx=0.0, cy=0.0, area=0.1):
    return {'found': found, 'label': 'person', 'conf': 0.9,
            'cx': cx, 'cy': cy, 'area': area}


def test_nothing_found_stays_stopped():
    linear, angular, arrived = steering_command(_report(found=False))
    assert (linear, angular, arrived) == (0.0, 0.0, False)


def test_far_and_centred_drives_straight():
    linear, angular, arrived = steering_command(_report(cx=0.0, area=0.05))
    assert linear > 0.0
    assert angular == 0.0
    assert arrived is False


def test_target_on_the_right_turns_right():
    # cx > 0 means the target is right of centre; angular.z should be
    # negative (clockwise, i.e. turning right) for a standard ROS Twist.
    _, angular, _ = steering_command(_report(cx=0.5, area=0.05))
    assert angular < 0.0


def test_target_on_the_left_turns_left():
    _, angular, _ = steering_command(_report(cx=-0.5, area=0.05))
    assert angular > 0.0


def test_close_enough_reports_arrived_and_stops():
    linear, angular, arrived = steering_command(
        _report(area=0.4), arrived_area=0.35)
    assert (linear, angular, arrived) == (0.0, 0.0, True)


def test_area_boundary_counts_as_arrived():
    _, _, arrived = steering_command(_report(area=0.35), arrived_area=0.35)
    assert arrived is True


def test_gains_are_configurable():
    _, angular, _ = steering_command(
        _report(cx=1.0, area=0.05), k_angular=2.0)
    assert angular == -2.0
