#!/usr/bin/env python3
"""Squad coordinator: expand one squad_navigation mission into per-robot
navigation missions.

Subscribes to /submit_squad_mission, validates it is a squad_navigation
mission, and fans out one navigation mission per robot to that robot's
submit_mission topic. It holds no map data and no control logic - the
contiguous half-split happens inside each controller (split_waypoints). The
LLM is never in the control loop; this node is deterministic.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from core_task_controller.function import expand_squad, validate_mission

# index -> the submit_mission topic of each robot's controller.
ROBOT_TOPICS = ['/submit_mission', '/robot2/submit_mission']


class SquadCoordinator(Node):
    def __init__(self):
        super().__init__('squad_coordinator')
        self.n_robots = len(ROBOT_TOPICS)
        self.pubs = [self.create_publisher(String, t, 10) for t in ROBOT_TOPICS]
        self.create_subscription(
            String, 'submit_squad_mission', self._on_squad, 10)
        self.get_logger().info(
            'squad_coordinator ready (%d robots)' % self.n_robots)

    def _on_squad(self, msg):
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error('squad mission not JSON: %s' % exc)
            return
        if mission.get('mode') != 'squad_navigation':
            self.get_logger().error(
                'not a squad_navigation mission: %s' % mission.get('mode'))
            return
        ok, reason = validate_mission(mission)
        if not ok:
            self.get_logger().error('invalid squad mission: %s' % reason)
            return
        for pub, sub_mission in zip(self.pubs,
                                    expand_squad(mission, self.n_robots)):
            pub.publish(String(data=json.dumps(sub_mission)))
            self.get_logger().info('dispatched: %s' % json.dumps(sub_mission))


def main(args=None):
    rclpy.init(args=args)
    node = SquadCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
