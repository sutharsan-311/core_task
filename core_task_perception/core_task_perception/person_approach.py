#!/usr/bin/env python3
"""Vision-servo approach: drives toward the detected target and reports arrival.

Idle until /approach_enable turns it on (Operation_controller does this once
the search phase finds the person), then on every /target_detection report
publishes a steering Twist on /cmd_vel via approach.steering_command(). Once
close enough it stops, publishes /approach_done once, and goes idle again.

    ros2 run core_task_perception person_approach

No detection math lives here - target_detector.py/detection.py already turn
the camera into the found/cx/cy/area report this node just drives on.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from core_task_perception.approach import steering_command


class PersonApproach(Node):
    """Vision-servo drive toward the current /target_detection report."""

    def __init__(self):
        """Wire up enable/detection in and cmd_vel/done out; start idle."""
        super().__init__('person_approach')

        self.arrived_area = self.declare_parameter('arrived_area', 0.35).value
        self.k_angular = self.declare_parameter('k_angular', 1.5).value
        self.linear_speed = self.declare_parameter('linear_speed', 0.15).value
        detection_topic = self.declare_parameter(
            'detection_topic', 'target_detection').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', 'cmd_vel').value

        self.enabled = False
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.done_pub = self.create_publisher(Bool, 'approach_done', 10)
        self.create_subscription(Bool, 'approach_enable', self._on_enable, 10)
        self.create_subscription(String, detection_topic, self._on_detection, 10)

        self.get_logger().info('person_approach ready (idle)')

    def _on_enable(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.cmd_pub.publish(Twist())      # stop immediately when disabled
        self.get_logger().info(
            'approach %s' % ('enabled' if self.enabled else 'disabled'))

    def _on_detection(self, msg):
        if not self.enabled:
            return
        report = json.loads(msg.data)
        linear, angular, arrived = steering_command(
            report, self.arrived_area, self.k_angular, self.linear_speed)

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

        if arrived:
            self.enabled = False
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info('arrived at target (area=%.3f)' % report['area'])


def main(args=None):
    """Spin the approach node until interrupted."""
    rclpy.init(args=args)
    node = PersonApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
