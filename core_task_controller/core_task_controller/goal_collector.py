#!/usr/bin/env python3
"""Perimeter goal collector - standalone helper, decoupled from the FSM.

It never talks to Operation_controller directly; it is driven entirely by the
controller's public topics:

  /submit_mission     -> learns map_name + loops (which perimeter file to write)
  /operation_feedback -> only collects while the phase is 'goalpoint_collection'

Capture inputs (both live at once):

  /goal_pose     RViz "2D Goal Pose"   -> x, y + the heading you drag
  /clicked_point RViz "Publish Point"  -> x, y + the robot's current AMCL heading

Every captured point is written to <map_name>_perimeter.yaml immediately, so the
file on disk always matches what has been clicked so far - no batching, nothing
lost if the run is interrupted.

The dock is pinned to the robot's AMCL pose at the moment collection starts.
"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from core_task_controller.function import quaternion_to_yaw, save_perimeter
from core_task_controller.markers import waypoint_markers

COLLECT_PHASE = 'goalpoint_collection'


class GoalCollector(Node):
    def __init__(self):
        super().__init__('goal_collector')
        # Same source-first map dir as Operation_controller, so both agree.
        nav_share = get_package_share_directory('core_task_navigation')
        src_pkg = nav_share.replace('/install/', '/src/core_task/').split('/share/')[0]
        src_map = os.path.join(src_pkg, 'map')
        default_map_dir = src_map if os.path.isdir(src_map) \
            else os.path.join(nav_share, 'map')
        self.map_dir = self.declare_parameter('map_dir', default_map_dir).value

        self.phase = None
        self.map_name = self._get_last_map_name()
        self.loops = 1
        self.dock = None
        self.points = []
        self.amcl_pose = None

        self.create_subscription(String, 'submit_mission', self._on_mission, 10)
        # /operation_feedback is published TRANSIENT_LOCAL (latched). Match it,
        # otherwise a collector started mid-mission never receives the retained
        # phase and silently ignores every click until the next transition.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Latched so an RViz that connects mid-collection still gets the markers.
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints', latched)
        self.create_subscription(String, 'operation_feedback',
                                 self._on_feedback, latched)
        self.create_subscription(PointStamped, 'clicked_point',
                                 self._on_clicked_point, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self._on_goal_pose, 10)
        # AMCL publishes amcl_pose TRANSIENT_LOCAL and only on motion updates -
        # a stationary robot never sends a new one. Match the durability so we
        # get the retained last pose immediately and can pin the dock.
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_amcl_pose, latched)
        self.get_logger().info('goal_collector ready (map_dir=%s)' % self.map_dir)

    def _get_last_map_name(self):
        """Read the last saved map name from .last_map file, else default to warehouse."""
        last_map_file = os.path.join(self.map_dir, '.last_map')
        try:
            if os.path.exists(last_map_file):
                with open(last_map_file, 'r') as f:
                    name = f.read().strip()
                    if name:
                        return name
        except Exception:
            pass
        return 'warehouse'  # fallback default

    # ---- control inputs -------------------------------------------------
    def _on_mission(self, msg):
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if mission.get('map_name'):
            self.map_name = mission['map_name']
            self.loops = mission.get('loops', 1)

    def _on_feedback(self, msg):
        previous, self.phase = self.phase, msg.data
        if self.phase == COLLECT_PHASE and previous != COLLECT_PHASE:
            # Entering collection: check if a newly mapped map was saved
            # (from operation_controller's mapping phase), and use it.
            last_map = self._get_last_map_name()
            if last_map and last_map != self.map_name:
                self.map_name = last_map
                self.get_logger().info('using newly mapped map: "%s"' % self.map_name)

            # Start clean. The dock may be None here -
            # AMCL is activated by this same transition and usually hasn't
            # published a pose yet, so _on_amcl_pose pins it a moment later.
            self.points = []
            self.dock = self._current_pose()
            self.get_logger().info('collecting for map "%s" (dock=%s)'
                                   % (self.map_name, self.dock or 'pending amcl'))
            self._save()
            self._publish_markers()      # clears any markers from a prior run
        elif self.phase != COLLECT_PHASE and previous == COLLECT_PHASE:
            # Exiting collection: clear the .last_map file so the next collection
            # (if manual/individual) doesn't auto-use the previous mapping's name.
            # This ensures individual point collections use the explicitly set map_name.
            last_map_file = os.path.join(self.map_dir, '.last_map')
            try:
                if os.path.exists(last_map_file):
                    os.remove(last_map_file)
            except Exception as e:
                self.get_logger().warning('Could not clear last_map: %s' % e)
            self.get_logger().info('collection finished; cleared .last_map')

    def _on_amcl_pose(self, msg):
        self.amcl_pose = msg.pose.pose
        # Pin the dock to the first AMCL pose seen after collection starts.
        if self.phase == COLLECT_PHASE and self.dock is None:
            self.dock = self._current_pose()
            self.get_logger().info('dock pinned from amcl: [%.2f, %.2f, %.2f]'
                                   % tuple(self.dock))
            self._save()

    # ---- capture inputs -------------------------------------------------
    def _on_clicked_point(self, msg):
        if self.phase != COLLECT_PHASE:
            return
        self._add([msg.point.x, msg.point.y, self._current_yaw()], 'Publish Point')

    def _on_goal_pose(self, msg):
        if self.phase != COLLECT_PHASE:
            return
        q = msg.pose.orientation
        self._add([msg.pose.position.x, msg.pose.position.y,
                   quaternion_to_yaw(q.x, q.y, q.z, q.w)], '2D Goal Pose')

    # ---- helpers --------------------------------------------------------
    def _current_pose(self):
        if self.amcl_pose is None:
            return None
        p, q = self.amcl_pose.position, self.amcl_pose.orientation
        return [p.x, p.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)]

    def _current_yaw(self):
        pose = self._current_pose()
        return pose[2] if pose else 0.0

    def _add(self, point, source):
        self.points.append(point)
        self._save()
        self._publish_markers()
        self.get_logger().info(
            'point %d captured from %s -> [%.2f, %.2f, %.2f]'
            % (len(self.points), source, point[0], point[1], point[2]))

    def _publish_markers(self):
        self.marker_pub.publish(waypoint_markers(self.points))

    def _path(self):
        return os.path.join(self.map_dir, '%s_perimeter.yaml' % self.map_name)

    def _save(self):
        if not self.map_name:
            self.get_logger().warn('no map_name seen yet - cannot save')
            return
        save_perimeter(self._path(), self.map_name, self.dock, self.points,
                       self.loops)


def main(args=None):
    rclpy.init(args=args)
    node = GoalCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
