#!/usr/bin/env python3
"""Natural Language Mission Interface - ROS 2 node.

Listens for natural language commands on /natural_language_mission, calls an LLM
(via AWS Bedrock, Claude API, or OpenAI) to generate structured mission JSON,
validates the result, and publishes valid missions to /submit_mission
(Operation_controller reads it).

LLM proposes; this node validates; the executor (Operation_controller) decides.
The LLM is never in the control loop.
"""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from core_task_controller.llm_client import LlmClient
from core_task_controller.mission_validator import validate_and_explain


class NaturalLanguageMissionInterface(Node):
    """ROS 2 node bridging natural language to validated mission JSON."""

    def __init__(self):
        """Wire up the LLM client, publishers, and the command subscription."""
        super().__init__('nlm_interface')

        model = self.declare_parameter('model', None).value
        temperature = self.declare_parameter('temperature', 0.3).value
        provider = self.declare_parameter('provider', None).value

        # Auto-detect provider from environment: Bedrock, Claude API, or OpenAI
        self.llm = LlmClient(
            model=model,
            temperature=temperature,
            provider=provider,
        )

        self.submit_pub = self.create_publisher(String, 'submit_mission', 10)
        self.squad_pub = self.create_publisher(
            String, 'submit_squad_mission', 10)
        self.feedback_pub = self.create_publisher(String, 'nlm_feedback', 10)
        self.create_subscription(
            String, 'natural_language_mission', self._on_nl_command, 10)

        self.get_logger().info(
            'nlm_interface ready (model=%s, temperature=%s)' % (model, temperature))

    def _on_nl_command(self, msg: String):
        """Handle incoming natural language command."""
        # ponytail: the API call blocks the executor for a few seconds. This node
        # does nothing else while waiting, so commands just queue. Move to a
        # MultiThreadedExecutor if it ever needs to serve anything concurrently.
        nl_command = msg.data
        self.get_logger().info('NL command received: "%s"' % nl_command)

        llm_result = self.llm.generate_mission(nl_command)
        if not llm_result['success']:
            self._publish_feedback(
                'LLM failed to generate mission: %s' % llm_result['reasoning'])
            self.get_logger().warn('LLM error: %s' % llm_result['reasoning'])
            return

        mission = llm_result['mission']
        self.get_logger().info('LLM proposed: %s' % json.dumps(mission))

        validation = validate_and_explain(mission)
        if not validation['valid']:
            self._publish_feedback(
                'Mission validation failed: %s' % validation['reason'])
            self.get_logger().warn('Validation error: %s' % validation['reason'])
            return

        if mission['mode'] == 'squad_navigation':
            self.squad_pub.publish(String(data=json.dumps(mission)))
            target = '/submit_squad_mission'
        else:
            self.submit_pub.publish(String(data=json.dumps(mission)))
            target = '/submit_mission'
        self._publish_feedback(
            'Mission accepted: mode=%s, map=%s'
            % (mission['mode'], mission['map_name']))
        self.get_logger().info('Mission published to %s' % target)

    def _publish_feedback(self, message: str):
        """Publish feedback to /nlm_feedback topic."""
        self.feedback_pub.publish(String(data=message))


def main(args=None):
    """Spin the node until interrupted."""
    rclpy.init(args=args)
    node = NaturalLanguageMissionInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
