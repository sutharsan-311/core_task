#!/usr/bin/env python3
"""YOLO target detector for the robot camera.

Subscribes to the robot's camera, runs a YOLO model on the latest frame at a
fixed rate, and publishes a normalised report of the configured target class:

    /target_detection        std_msgs/String   JSON: found, cx, cy, area, conf
    /target_detection/image   sensor_msgs/Image annotated frame (for the operator)

The detection maths live in detection.py (pure, unit-tested). This node is just
camera in -> model -> publish. The follow controller consumes /target_detection
and turns it into /cmd_vel; keeping detection and control separate means either
can be tested or swapped alone.

    ros2 run core_task_perception target_detector
    ros2 run core_task_perception target_detector --ros-args -p target_class:=chair
"""
import json

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from core_task_perception.detection import summarize


class TargetDetector(Node):
    """Detect a configurable class in the camera stream and report it."""

    def __init__(self):
        """Load the model, wire the camera in and the detection out."""
        super().__init__('target_detector')

        self.target = self.declare_parameter('target_class', 'person').value
        self.conf = self.declare_parameter('conf', 0.35).value
        model_path = self.declare_parameter('model', 'yolo26n.pt').value
        image_topic = self.declare_parameter(
            'image_topic', '/camera/image_raw').value
        rate = self.declare_parameter('rate_hz', 10.0).value
        self.publish_annotated = self.declare_parameter(
            'annotate', True).value

        # Import here, not at module top: it pulls in torch (seconds to load)
        # and it keeps detection.py's unit tests free of the heavy dependency.
        from ultralytics import YOLO
        self.get_logger().info('loading model %s ...' % model_path)
        self.model = YOLO(model_path)
        self.names = self.model.names           # class id -> label
        if self.target not in self.names.values():
            self.get_logger().warn(
                'target_class %r is not in the model classes; nothing will '
                'ever match. Known example classes: person, chair, backpack.'
                % self.target)

        self.bridge = CvBridge()
        self._frame = None                       # latest cv image, BGR
        self._seen = False                       # logged the first hit yet?

        self.det_pub = self.create_publisher(String, 'target_detection', 10)
        self.img_pub = self.create_publisher(
            Image, 'target_detection/image', 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)
        # Run inference on a timer over the latest frame rather than per
        # callback, so a fast camera cannot pile up more work than the model
        # can clear.
        self.create_timer(1.0 / max(1.0, rate), self._infer)

        self.get_logger().info(
            'target_detector ready (target=%s, model=%s, topic=%s)'
            % (self.target, model_path, image_topic))

    def _on_image(self, msg):
        self._frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _infer(self):
        frame = self._frame
        if frame is None:
            return
        h, w = frame.shape[:2]
        results = self.model.predict(
            frame, conf=self.conf, verbose=False)[0]

        boxes = []
        for box in results.boxes:
            label = self.names[int(box.cls[0])]
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            boxes.append((label, float(box.conf[0]), x1, y1, x2, y2))

        report = summarize(boxes, self.target, w, h)
        self.det_pub.publish(String(data=json.dumps(report)))

        if report['found'] and not self._seen:
            self._seen = True
            self.get_logger().info(
                'target %r acquired (conf=%.2f, cx=%.2f, area=%.3f)'
                % (self.target, report['conf'], report['cx'], report['area']))
        elif not report['found']:
            self._seen = False               # re-announce next acquisition

        if self.publish_annotated:
            annotated = results.plot()       # BGR frame with boxes drawn
            self.img_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))


def main(args=None):
    """Spin the detector until interrupted."""
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
