"""Abort-path tests for OperationController, ROS graph not required.

The node is built with its sibling-package lookup and Nav2 clients patched out,
so it constructs anywhere. These cover the safety-critical bits the pure FSM
table cannot: phase gating, goal cancellation, and suppressing the cancelled
waypoint result so an abort does not land in FAULT.
"""
from unittest.mock import Mock, patch

import pytest
import rclpy
from std_srvs.srv import Trigger

from core_task_controller.function import Event, Phase


@pytest.fixture(scope='module')
def ros():
    """Bring rclpy up once for the module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros, tmp_path):
    """Build an OperationController with package lookup + Nav2 clients stubbed."""
    with patch('core_task_controller.Operation_controller.'
               'get_package_share_directory', return_value=str(tmp_path)), \
            patch('core_task_controller.Operation_controller.ActionClient'), \
            patch('core_task_controller.Operation_controller.TransformListener'):
        from core_task_controller.Operation_controller import OperationController
        n = OperationController()
    yield n
    n.destroy_node()


def abort(node):
    """Call the abort service handler and return its response."""
    return node._on_abort(Trigger.Request(), Trigger.Response())


def test_abort_rejected_outside_perimeter(node):
    """Mapping, idle, etc. have no patrol to interrupt."""
    for phase in (Phase.MAPPING, Phase.IDLE, Phase.START_MAPPING):
        node.phase = phase
        resp = abort(node)
        assert resp.success is False
        assert phase.name in resp.message


def test_abort_while_returning_is_a_noop_success(node):
    """Already heading home: acknowledge, don't re-trigger."""
    node.phase = Phase.RETURN_TO_DOCK
    resp = abort(node)
    assert resp.success is True
    assert 'already' in resp.message.lower()


def test_abort_in_perimeter_cancels_then_redirects(node):
    """Abort cancels the live goal and only enqueues ABORT once cancel completes."""
    node.phase = Phase.PERIMETER
    node._events.clear()
    handle = Mock()
    node._nav_goal_handle = handle

    resp = abort(node)

    assert resp.success is True
    assert node._aborting is True
    assert handle.cancel_goal_async.called
    # Serialised: nothing redirects until the cancel callback fires.
    assert list(node._events) == []

    cancel_cb = handle.cancel_goal_async.return_value \
        .add_done_callback.call_args[0][0]
    cancel_cb(Mock())
    assert list(node._events) == [Event.ABORT]


def test_abort_with_no_live_goal_redirects_immediately(node):
    """Between waypoints there may be no handle; abort still proceeds."""
    node.phase = Phase.PERIMETER
    node._events.clear()
    node._nav_goal_handle = None

    abort(node)

    assert list(node._events) == [Event.ABORT]


def test_cancelled_waypoint_result_does_not_fault(node):
    """The cancelled goal reports non-success; that must not enqueue ERROR."""
    node._aborting = True
    node._events.clear()
    result = Mock()
    result.result.return_value.status = 6      # 6 == STATUS_ABORTED

    node._on_wp_result(result)

    assert list(node._events) == []            # no ERROR -> no FAULT
    assert node._nav_goal_handle is None


def test_normal_waypoint_failure_still_faults(node):
    """Without an abort in progress, a failed goal must still fault."""
    node._aborting = False
    node._events.clear()
    result = Mock()
    result.result.return_value.status = 6

    node._on_wp_result(result)

    assert list(node._events) == [Event.ERROR]
