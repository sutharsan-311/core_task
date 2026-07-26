#!/usr/bin/env python3
"""Mirror robot2's namespaced TF onto the global /tf so ONE RViz shows both robots.

robot2's stack publishes to /robot2/tf(_static) to stay isolated from robot1;
RViz only reads /tf(_static), so robot2 is otherwise invisible there. Every
robot2 frame is robot2/-prefixed, so merging onto the global tree is
collision-free. Strictly one-way (/robot2/tf -> /tf) - no loop.

Only needed with squad:=true; bringup starts it under that condition.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class TfBridge(Node):
    """Republish /robot2/tf -> /tf and /robot2/tf_static -> /tf_static."""

    def __init__(self):
        super().__init__('robot2_tf_bridge')
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(TFMessage, '/robot2/tf', self._dyn, 100)
        # tf_static is latched; match TRANSIENT_LOCAL so an RViz (or this bridge)
        # that connects late still receives robot2's one-shot static transforms.
        static_qos = QoSProfile(depth=1)
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_static = self.create_publisher(
            TFMessage, '/tf_static', static_qos)
        self.create_subscription(
            TFMessage, '/robot2/tf_static', self._static, static_qos)
        self.get_logger().info(
            'robot2 tf bridge up: /robot2/tf(_static) -> /tf(_static)')

    def _dyn(self, msg):
        self.pub.publish(msg)

    def _static(self, msg):
        self.pub_static.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
