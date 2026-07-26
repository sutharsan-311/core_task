"""Regression tests for the dock/initialpose handoff between mapping and goal
collection (ROS graph not required, same stub pattern as test_operation_abort).

Bug: AMCL only ever got nav2_params.yaml's hardcoded initial_pose, which is
correct for the original default map and wrong for any map built since - and
goal_collector could pin+save a stale/zero dock before AMCL published a real
one. See Operation_controller._after_loc_for_goalpoints /
goal_collector._on_feedback.
"""
from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest
import rclpy


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def op_node(ros, tmp_path):
    with patch('core_task_controller.Operation_controller.'
               'get_package_share_directory', return_value=str(tmp_path)), \
            patch('core_task_controller.Operation_controller.ActionClient'), \
            patch('core_task_controller.Operation_controller.TransformListener'):
        from core_task_controller.Operation_controller import OperationController
        n = OperationController()
    yield n
    n.destroy_node()


@pytest.fixture
def collector(ros, tmp_path):
    with patch('core_task_controller.goal_collector.'
               'get_package_share_directory', return_value=str(tmp_path)):
        from core_task_controller.goal_collector import GoalCollector
        n = GoalCollector()
    yield n
    n.destroy_node()


def _fake_transform(x, y, yaw):
    """A stand-in for tf_buffer.lookup_transform's TransformStamped."""
    import math
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x, y=y, z=0.0),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


# ---- Operation_controller: navigation waits for the map swap too

def test_navigation_waits_for_map_swap_before_publishing_pose(op_node):
    op_node.dock = [1.0, 2.0, 0.0]
    op_node._publish_initialpose = Mock()
    op_node._manage_then = Mock()
    load_map_calls = []
    op_node._load_map = lambda cb: load_map_calls.append(cb)

    op_node._after_loc_startup(True)

    op_node._publish_initialpose.assert_not_called()
    op_node._manage_then.assert_not_called()  # nav2 startup gated on it too
    load_map_calls[0](True)
    op_node._publish_initialpose.assert_called_once_with([1.0, 2.0, 0.0])
    op_node._manage_then.assert_called_once()


def test_navigation_map_load_failure_faults(op_node):
    op_node._load_map = Mock(side_effect=lambda cb: cb(False))
    op_node._publish_initialpose = Mock()
    op_node._events.clear()

    op_node._after_loc_startup(True)

    op_node._publish_initialpose.assert_not_called()
    from core_task_controller.function import Event
    assert list(op_node._events) == [Event.ERROR]


# ---- Operation_controller: capture at CLOSE_MAPPING, publish at goal collection

def test_close_mapping_captures_live_pose(op_node):
    op_node.tf_buffer.lookup_transform = Mock(
        return_value=_fake_transform(1.5, -2.0, 0.3))

    from core_task_controller.function import Phase
    op_node._on_enter(Phase.CLOSE_MAPPING)

    x, y, yaw = op_node._last_mapped_pose
    assert (round(x, 3), round(y, 3), round(yaw, 3)) == (1.5, -2.0, 0.3)


def test_goalpoints_publish_uses_mapped_pose_then_clears(op_node):
    op_node._last_mapped_pose = [1.5, -2.0, 0.3]
    op_node._publish_initialpose = Mock()

    op_node._after_map_loaded_for_goalpoints(True)

    op_node._publish_initialpose.assert_called_once_with([1.5, -2.0, 0.3])
    assert op_node._last_mapped_pose is None  # doesn't leak into a later run


def test_goalpoints_reuses_existing_perimeter_dock_for_already_mapped_map(
        op_node, tmp_path):
    """Re-collecting an existing map (no mapping ran this session) should use
    that map's own recorded dock, not the default map's hardcoded pose."""
    from core_task_controller.function import save_perimeter
    op_node.map_dir = str(tmp_path)
    op_node.mission = {'map_name': 'foo'}
    save_perimeter(tmp_path / 'foo_perimeter.yaml', 'foo', [4.0, 5.0, 1.0], [])
    op_node._last_mapped_pose = None
    op_node._publish_initialpose = Mock()

    op_node._after_map_loaded_for_goalpoints(True)

    op_node._publish_initialpose.assert_called_once_with([4.0, 5.0, 1.0])


def test_goalpoints_first_ever_collection_falls_back_to_amcl_default(
        op_node, tmp_path):
    """Brand new map, never collected before, not mapped this session either
    - nothing on disk or in memory to seed from; let AMCL's own (cold-boot)
    initial_pose param stand."""
    op_node.map_dir = str(tmp_path)
    op_node.mission = {'map_name': 'never_seen'}
    op_node._last_mapped_pose = None
    op_node._publish_initialpose = Mock()

    op_node._after_map_loaded_for_goalpoints(True)

    op_node._publish_initialpose.assert_not_called()


def test_goalpoints_waits_for_map_swap_before_publishing_pose(op_node):
    """map_server keeps serving the previous mission's map the instant it
    reactivates, and only swaps once LoadMap's response lands - publishing
    initialpose before that risks AMCL evaluating it against the wrong map."""
    op_node._last_mapped_pose = [1.5, -2.0, 0.3]
    op_node._publish_initialpose = Mock()
    load_map_calls = []
    op_node._load_map = lambda cb: load_map_calls.append(cb)

    op_node._after_loc_for_goalpoints(True)

    op_node._publish_initialpose.assert_not_called()  # not yet - map isn't loaded
    load_map_calls[0](True)                            # LoadMap now completes
    op_node._publish_initialpose.assert_called_once_with([1.5, -2.0, 0.3])


def test_goalpoints_map_load_failure_faults_without_publishing(op_node):
    op_node._last_mapped_pose = [1.5, -2.0, 0.3]
    op_node._publish_initialpose = Mock()
    op_node._load_map = Mock(side_effect=lambda cb: cb(False))
    op_node._events.clear()

    op_node._after_loc_for_goalpoints(True)

    op_node._publish_initialpose.assert_not_called()
    from core_task_controller.function import Event
    assert list(op_node._events) == [Event.ERROR]


def test_goalpoints_failure_still_faults(op_node):
    from core_task_controller.function import Event
    op_node._events.clear()
    op_node._after_loc_for_goalpoints(False)
    assert list(op_node._events) == [Event.ERROR]


# ---- goal_collector: no bogus dock write before AMCL confirms a real pose

def test_entering_collection_does_not_save_stale_or_zero_dock(collector):
    collector.amcl_pose = Mock()  # leftover from a previous activation
    with patch('core_task_controller.goal_collector.save_perimeter') as save:
        collector.map_name = 'warehouse22'
        collector._on_feedback(SimpleNamespace(data='goalpoint_collection'))

    save.assert_not_called()
    assert collector.dock is None
    assert collector.amcl_pose is None  # stale reading cleared, not reused


def test_amcl_pose_after_entering_pins_dock_and_saves(collector):
    collector.map_name = 'warehouse22'
    collector._on_feedback(SimpleNamespace(data='goalpoint_collection'))

    pose_msg = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
    with patch('core_task_controller.goal_collector.save_perimeter') as save:
        collector._on_amcl_pose(pose_msg)

    assert collector.dock == [1.0, 2.0, 0.0]
    save.assert_called_once()
    assert save.call_args[0][2] == [1.0, 2.0, 0.0]  # dock arg


def _pose_msg(x, y):
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))


def test_dock_keeps_refining_until_first_waypoint_captured(collector):
    """Reproduces the warehouse33 log: AMCL's activation broadcasts
    nav2_params.yaml's default pose first, then the corrected one for this
    map moments later. Locking onto the first message would pin the wrong
    (default-map) dock into the perimeter file."""
    collector.map_name = 'warehouse33'
    collector._on_feedback(SimpleNamespace(data='goalpoint_collection'))

    with patch('core_task_controller.goal_collector.save_perimeter'):
        collector._on_amcl_pose(_pose_msg(0.264, 3.527))   # transient default
        collector._on_amcl_pose(_pose_msg(6.460, 1.912))   # corrected, moments later

    assert collector.dock == [6.460, 1.912, 0.0]


def test_dock_freezes_once_first_waypoint_is_captured(collector):
    collector.map_name = 'warehouse33'
    collector._on_feedback(SimpleNamespace(data='goalpoint_collection'))

    with patch('core_task_controller.goal_collector.save_perimeter'):
        collector._on_amcl_pose(_pose_msg(6.460, 1.912))
        collector._add([1.0, 1.0, 0.0], 'test')
        collector._on_amcl_pose(_pose_msg(9.0, 9.0))  # robot has since moved on

    assert collector.dock == [6.460, 1.912, 0.0]
