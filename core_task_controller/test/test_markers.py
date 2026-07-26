"""Unit tests for waypoint_markers (no ROS graph needed - just message structs)."""
from core_task_controller.markers import waypoint_markers
from visualization_msgs.msg import Marker


def test_leading_deleteall_clears_stale():
    """First marker is always DELETEALL so shrinking sets leave no orphans."""
    arr = waypoint_markers([[1.0, 2.0, 0.0]])
    assert arr.markers[0].action == Marker.DELETEALL


def test_one_sphere_per_waypoint_plus_line():
    """Two waypoints -> DELETEALL + 2 spheres + 1 line strip."""
    arr = waypoint_markers([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]])
    spheres = [m for m in arr.markers if m.type == Marker.SPHERE]
    lines = [m for m in arr.markers if m.type == Marker.LINE_STRIP]
    assert len(spheres) == 2
    assert len(lines) == 1
    assert [p.x for p in lines[0].points] == [0.0, 1.0]
    assert spheres[0].pose.position.x == 0.0 and spheres[1].pose.position.x == 1.0


def test_single_waypoint_has_no_line():
    """A line strip needs >= 2 points."""
    arr = waypoint_markers([[3.0, 4.0, 0.0]])
    assert not [m for m in arr.markers if m.type == Marker.LINE_STRIP]


def test_empty_is_just_deleteall():
    """No waypoints -> only the clear marker."""
    arr = waypoint_markers([])
    assert len(arr.markers) == 1 and arr.markers[0].action == Marker.DELETEALL
