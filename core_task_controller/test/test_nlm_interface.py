"""End-to-end tests for the nlm_interface node, with Claude mocked.

Covers the whole LLM -> validator -> /submit_mission path, including the node's
own routing. No AWS credentials and no ROS graph needed: rclpy.init() is enough
to build the node, and boto3.client is patched out.

The property worth protecting is the negative one — a mission the validator
rejects must never reach /submit_mission, because Operation_controller trusts
whatever lands there.
"""
import json
from unittest.mock import Mock, patch

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String


@pytest.fixture(scope='module')
def ros():
    """Bring rclpy up once for the module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(ros):
    """nlm_interface with a mocked Bedrock client and mocked publishers.

    provider is forced to 'bedrock' via the ROS param rather than left to
    auto-detect: LlmClient._detect_provider() reads real env vars
    (AWS_BEARER_TOKEN_BEDROCK etc.), so without this override the test's
    outcome would depend on whatever happens to be set in the environment
    running it, not just the boto3.client patch below.
    """
    with patch('boto3.client'):
        from core_task_controller.nlm_interface import (
            NaturalLanguageMissionInterface,
        )
        n = NaturalLanguageMissionInterface(
            parameter_overrides=[Parameter('provider', value='bedrock')])
    n.submit_pub = Mock()
    n.feedback_pub = Mock()
    yield n
    n.destroy_node()


def claude_replies(node, text):
    """Make the mocked Bedrock client return `text` as its response body."""
    node.llm.client.converse.return_value = {
        'output': {'message': {'content': [{'text': text}]}}}


def claude_proposes(node, mission):
    """Make the mocked Claude propose `mission` in a json code block."""
    claude_replies(node, '```json\n%s\n```' % json.dumps(mission))


def published_mission(node):
    """Return the mission dict actually published to /submit_mission."""
    return json.loads(node.submit_pub.publish.call_args[0][0].data)


def feedback(node):
    """Return the last string published to /nlm_feedback."""
    return node.feedback_pub.publish.call_args[0][0].data


def test_valid_navigation_mission_is_published(node):
    """A valid mission reaches /submit_mission unchanged."""
    mission = {'mode': 'navigation', 'map_name': 'warehouse', 'loops': 2}
    claude_proposes(node, mission)

    node._on_nl_command(String(data='Navigate the perimeter twice'))

    node.submit_pub.publish.assert_called_once()
    assert published_mission(node) == mission
    assert 'accepted' in feedback(node).lower()


def test_valid_mapping_mission_is_published(node):
    """Mapping missions carry no loops and still pass."""
    mission = {'mode': 'mapping', 'map_name': 'warehouse'}
    claude_proposes(node, mission)

    node._on_nl_command(String(data='Start building a map'))

    node.submit_pub.publish.assert_called_once()
    assert published_mission(node) == mission


def test_mission_rejected_by_validator_is_not_published(node):
    """A schema-valid-looking mission the validator rejects is dropped.

    loops=0 clears LlmClient's required-field check (mode and map_name are both
    present) and is only caught by validate_mission. This is the seam the node
    exists to police.
    """
    claude_proposes(
        node, {'mode': 'navigation', 'map_name': 'warehouse', 'loops': 0})

    node._on_nl_command(String(data='Patrol zero times'))

    node.submit_pub.publish.assert_not_called()
    assert 'loops' in feedback(node).lower()


def test_unknown_mode_is_not_published(node):
    """A mode outside VALID_MODES never reaches the executor."""
    claude_proposes(node, {'mode': 'inspect', 'map_name': 'warehouse'})

    node._on_nl_command(String(data='Inspect the shelves'))

    node.submit_pub.publish.assert_not_called()
    assert 'validation failed' in feedback(node).lower()


def test_llm_failure_is_not_published(node):
    """An API error is reported, not published, and does not raise."""
    node.llm.client.converse.side_effect = Exception('API timeout')

    node._on_nl_command(String(data='Patrol twice'))

    node.submit_pub.publish.assert_not_called()
    assert 'llm failed' in feedback(node).lower()


def test_prose_response_is_not_published(node):
    """A model that answers in prose instead of JSON is handled."""
    claude_replies(node, 'I am not sure what you would like me to do.')

    node._on_nl_command(String(data='Foobarize the robot'))

    node.submit_pub.publish.assert_not_called()
    assert 'llm failed' in feedback(node).lower()
