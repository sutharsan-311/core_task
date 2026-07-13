#!/usr/bin/env python3
"""Clamp cmd_vel to safe limits before it reaches the diff-drive controller.

Inline filter between the commander and the robot:

    teleop / Nav2 --(cmd_vel_in)--> cmd_vel_limiter --(cmd_vel)--> diff drive

The diff-drive plugin listens on /cmd_vel, so this node owns /cmd_vel and takes
its input from cmd_vel_in. Remap the commander onto cmd_vel_in, e.g.:

    ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r cmd_vel:=cmd_vel_in

Limits default to the TurtleBot3 Waffle Pi spec and are ROS params (see
config/cmd_vel_limiter.yaml). Only linear.x and angular.z are passed through;
a diff-drive robot can't move sideways, so linear.y is dropped.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, limit):
    """Symmetric clamp to [-limit, +limit]. Per-axis: preserves each component's
    sign but not the turning radius (fine for a hard velocity cap)."""
    return max(-limit, min(limit, value))


class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_limiter')
        self.max_linear = self.declare_parameter('max_linear', 0.26).value
        self.max_angular = self.declare_parameter('max_angular', 1.82).value
        in_topic = self.declare_parameter('input_topic', 'cmd_vel_in').value
        out_topic = self.declare_parameter('output_topic', 'cmd_vel').value

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._on_cmd, 10)
        self.get_logger().info(
            'limiting %s -> %s at |v|<=%.2f m/s, |w|<=%.2f rad/s'
            % (in_topic, out_topic, self.max_linear, self.max_angular))

    def _on_cmd(self, msg):
        out = Twist()
        out.linear.x = clamp(msg.linear.x, self.max_linear)
        out.angular.z = clamp(msg.angular.z, self.max_angular)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
