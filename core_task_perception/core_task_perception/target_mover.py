#!/usr/bin/env python3
"""Walk the target model around the patrol loop, for the follow demo.

Moves the person model kinematically via Gazebo's /set_entity_state service:
the model is posed directly, not pushed by physics, so it never falls and never
gets stuck the way a social-force actor does in cluttered space. It walks the
robot's own perimeter waypoints (navigable floor by definition), ping-ponged so
it never cuts the interior diagonal. Slower than the robot's 0.22 m/s top speed
so the robot can actually follow it.

    ros2 run core_task_perception target_mover
    ros2 run core_task_perception target_mover --ros-args -p speed:=0.1

Needs the gazebo_ros_state world plugin (adds /gazebo/set_entity_state); the
warehouse world loads it.
"""
import math

import rclpy
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node

from core_task_perception.path import ping_pong, pose_at

# Robot's patrol perimeter (x, y), the same loop Nav2 drives - so it is known
# navigable and the target and robot share ground. Flattened for the ROS param.
_DEFAULT_LOOP = [
    -4.46, -2.81, -4.75, -6.00, 0.59, -6.41,
    0.68, -1.47, 2.72, 0.91, 2.44, 2.58,
]


class TargetMover(Node):
    """Kinematically walk the target model around a closed loop."""

    def __init__(self):
        """Load the loop and start the walk timer."""
        super().__init__('target_mover')

        self.model = self.declare_parameter('model_name', 'target_person').value
        self.speed = self.declare_parameter('speed', 0.13).value
        self.z = self.declare_parameter('z_height', 0.0).value
        rate = self.declare_parameter('rate_hz', 20.0).value
        flat = self.declare_parameter('waypoints', _DEFAULT_LOOP).value
        service = self.declare_parameter(
            'service', '/gazebo/set_entity_state').value

        pts = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.loop = ping_pong(pts)
        self.dt = 1.0 / max(1.0, rate)
        self.dist = 0.0
        self._busy = False

        self.cli = self.create_client(SetEntityState, service)
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '%s not available - is the gazebo_ros_state world plugin '
                'loaded? Target will not move.' % service)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            'target_mover walking %r around %d waypoints at %.2f m/s'
            % (self.model, len(self.loop), self.speed))

    def _tick(self):
        if self._busy:
            return                       # last set_entity_state not back yet
        self.dist += self.speed * self.dt
        x, y, yaw = pose_at(self.loop, self.dist)

        req = SetEntityState.Request()
        req.state.name = self.model
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(self.z)
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        req.state.reference_frame = 'world'

        self._busy = True
        self.cli.call_async(req).add_done_callback(self._done)

    def _done(self, _future):
        self._busy = False


def main(args=None):
    """Spin the mover until interrupted."""
    rclpy.init(args=args)
    node = TargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
