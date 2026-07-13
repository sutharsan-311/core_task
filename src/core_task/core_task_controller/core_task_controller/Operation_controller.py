#!/usr/bin/env python3
"""Operation_controller: deterministic hierarchical state machine.

Runs the robot through a mapping run (slam_toolbox subprocess + map save +
RViz perimeter capture) and a navigation run (Nav2 lifecycle + per-waypoint
NavigateToPose loop + return to dock). Decision logic is the pure table in
function.py; this node only turns runtime conditions into events and performs
side effects. See docs/superpowers/specs/2026-07-13-operation-controller-design.md.
"""
import json
import os
import signal
import subprocess
from collections import deque

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from core_task_controller.function import (
    Event, Phase, load_perimeter, next_phase, quaternion_to_yaw,
    save_perimeter, validate_mission, yaw_to_quaternion)


class OperationController(Node):
    def __init__(self):
        super().__init__('operation_controller')
        # Where maps + perimeters live (co-located with the navigation package).
        # Prefer the SOURCE map dir so saved maps/perimeters persist across
        # colcon builds and land in git; fall back to the install share dir if
        # the source tree isn't present (install-only deployment).
        nav_share = get_package_share_directory('core_task_navigation')
        src_pkg = nav_share.replace('/install/', '/src/core_task/').split('/share/')[0]
        src_map = os.path.join(src_pkg, 'map')
        default_map_dir = src_map if os.path.isdir(src_map) \
            else os.path.join(nav_share, 'map')
        self.map_dir = self.declare_parameter('map_dir', default_map_dir).value
        self.get_logger().info('map_dir = %s' % self.map_dir)

        self.phase = Phase.IDLE
        self.mission = None
        self.dock = None                 # {'x','y','yaw'} captured at mapping start
        self.points = []                 # collected clicked perimeter points
        self.waypoints = []              # loaded perimeter for navigation
        self.loops_total = 1
        self.loop_index = 0
        self.wp_index = 0
        self.slam_proc = None
        self.amcl_pose = None            # latest /amcl_pose, for clicked-point yaw
        self._events = deque()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.feedback_pub = self.create_publisher(
            String, 'operation_feedback', latched)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        self.create_subscription(String, 'submit_mission', self._on_mission, 10)
        self.create_subscription(PointStamped, 'clicked_point',
                                 self._on_clicked_point, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose',
                                 self._on_amcl_pose, 10)
        self.create_service(Trigger, 'operator_done', self._on_operator_done)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.loc_mgr = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_localization/manage_nodes')
        self.nav_mgr = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.load_map_cli = self.create_client(LoadMap, '/map_server/load_map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_timer(0.1, self._tick)   # 10 Hz
        self._publish_feedback()
        self.get_logger().info('operation_controller ready (phase=IDLE)')

    # ---- event plumbing -------------------------------------------------
    def enqueue(self, event):
        self._events.append(event)

    def _tick(self):
        while self._events:
            self._apply(self._events.popleft())
        self._poll()
        while self._events:
            self._apply(self._events.popleft())

    def _apply(self, event):
        new = next_phase(self.phase, event)
        if new != self.phase:
            self.get_logger().info('%s --%s--> %s'
                                   % (self.phase.name, event.name, new.name))
            self.phase = new
            self._publish_feedback()
            self._on_enter(new)

    def _publish_feedback(self):
        self.feedback_pub.publish(String(data=self.phase.name.lower()))

    # ---- inputs ---------------------------------------------------------
    def _on_mission(self, msg):
        if self.phase not in (Phase.IDLE, Phase.FAULT):
            self.get_logger().warn('mission ignored: busy in %s' % self.phase.name)
            return
        try:
            mission = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error('mission not JSON: %s' % exc)
            self.enqueue(Event.SUBMIT_INVALID)
            return
        ok, reason = validate_mission(mission)
        if not ok:
            self.get_logger().error('invalid mission: %s' % reason)
            self.feedback_pub.publish(String(data='fault:%s' % reason))
            self.enqueue(Event.SUBMIT_INVALID)
            return
        self.mission = mission
        self.enqueue({'mapping': Event.SUBMIT_MAPPING,
                      'navigation': Event.SUBMIT_NAV,
                      'collect_goals': Event.SUBMIT_GOALS}[mission['mode']])

    def _on_operator_done(self, request, response):
        if self.phase == Phase.MAPPING:
            self.enqueue(Event.OPERATOR_DONE)
            response.success, response.message = True, 'mapping -> saving'
        elif self.phase == Phase.GOALPOINT_COLLECTION:
            self._save_perimeter()
            self.enqueue(Event.OPERATOR_DONE)
            response.success, response.message = True, 'perimeter saved'
        else:
            response.success = False
            response.message = 'no operator action in phase %s' % self.phase.name
        return response

    def _on_amcl_pose(self, msg):
        self.amcl_pose = msg.pose.pose

    def _current_yaw(self):
        """Robot heading from the latest AMCL pose (0.0 until AMCL localizes)."""
        if self.amcl_pose is None:
            return 0.0
        q = self.amcl_pose.orientation
        return quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _current_pose_dict(self):
        """Robot pose from the latest AMCL estimate, or None if not localized."""
        if self.amcl_pose is None:
            return None
        p, q = self.amcl_pose.position, self.amcl_pose.orientation
        return {'x': p.x, 'y': p.y, 'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w)}

    def _on_clicked_point(self, msg):
        if self.phase == Phase.GOALPOINT_COLLECTION:
            yaw = self._current_yaw()
            self.points.append(
                {'x': float(msg.point.x), 'y': float(msg.point.y), 'yaw': yaw})
            self.get_logger().info(
                'perimeter point %d captured (yaw=%.2f from amcl)'
                % (len(self.points), yaw))

    # ---- entry side effects --------------------------------------------
    def _on_enter(self, phase):
        if phase == Phase.INITIALIZATION:
            self.enqueue({'mapping': Event.INIT_MAPPING,
                          'navigation': Event.INIT_NAV,
                          'collect_goals': Event.INIT_GOALS}[self.mission['mode']])
        elif phase == Phase.START_MAPPING:
            self._start_slam()
        elif phase == Phase.SAVING:
            self._save_map()
        elif phase == Phase.CLOSE_MAPPING:
            self._stop_slam()
        elif phase == Phase.GOALPOINT_COLLECTION:
            self.points = []
            self._manage_then(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP,
                              self._after_loc_for_goalpoints)
        elif phase == Phase.GOAL_POINTS_SAVED:
            self._manage(self.loc_mgr, ManageLifecycleNodes.Request.PAUSE)
        elif phase == Phase.START_NAVIGATION:
            self._start_navigation()
        elif phase == Phase.PERIMETER:
            self.loop_index = 0
            self.wp_index = 0
            self._send_current_waypoint()
        elif phase == Phase.RETURN_TO_DOCK:
            self._send_goal(self.dock, self._on_dock_result)
        elif phase == Phase.CLOSE_NAVIGATION:
            self._manage(self.nav_mgr, ManageLifecycleNodes.Request.PAUSE)
            self._manage(self.loc_mgr, ManageLifecycleNodes.Request.PAUSE)
            self.enqueue(Event.NAV_CLOSED)
        elif phase == Phase.FAULT:
            self.get_logger().error('FAULT - holding')

    # ---- per-phase polling ---------------------------------------------
    def _poll(self):
        p = self.phase
        if p == Phase.START_MAPPING:
            if self._capture_dock():
                self.enqueue(Event.SLAM_READY)
        elif p == Phase.CLOSE_MAPPING:
            if self.slam_proc is None or self.slam_proc.poll() is not None:
                self.slam_proc = None
                self.enqueue(Event.SLAM_CLOSED)
        elif p in (Phase.MAP_SAVED_SUCCESSFULLY, Phase.GOAL_POINTS_SAVED,
                   Phase.PERIMETER_COMPLETED, Phase.DOCKED):
            self.enqueue(Event.ADVANCE)

    # ---- slam / map -----------------------------------------------------
    def _start_slam(self):
        self.slam_proc = subprocess.Popen(
            ['ros2', 'launch', 'core_task_mapping', 'mapping.launch.py'],
            preexec_fn=os.setsid)

    def _stop_slam(self):
        if self.slam_proc and self.slam_proc.poll() is None:
            os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGINT)

    def _capture_dock(self):
        """Look up map->base_footprint once; store as dock. True when captured."""
        if self.dock is not None:
            return True
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        self.dock = {'x': t.x, 'y': t.y,
                     'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w)}
        self.get_logger().info('dock captured: %s' % self.dock)
        return True

    def _map_path(self, ext=''):
        return os.path.join(self.map_dir, self.mission['map_name'] + ext)

    def _save_map(self):
        rc = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', self._map_path()],
            capture_output=True).returncode
        self.enqueue(Event.SAVE_OK if rc == 0 else Event.ERROR)

    def _save_perimeter(self):
        # In collect_goals mode there was no mapping phase to capture the dock,
        # so fall back to the robot's current AMCL pose (then origin).
        dock = self.dock or self._current_pose_dict() \
            or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        save_perimeter(self._map_path('_perimeter.yaml'),
                       self.mission['map_name'], dock, self.points,
                       loops=self.mission.get('loops', 1))
        self.get_logger().info('perimeter saved (%d points, dock=%s)'
                               % (len(self.points), dock))

    # ---- navigation -----------------------------------------------------
    def _start_navigation(self):
        # Load the perimeter first so a missing/invalid file fails fast, before
        # we bother activating Nav2.
        try:
            data = load_perimeter(self._map_path('_perimeter.yaml'))
        except (FileNotFoundError, OSError) as exc:
            self.get_logger().error('cannot load perimeter: %s' % exc)
            self.enqueue(Event.ERROR)
            return
        self.dock = data['dock']
        self.waypoints = data['waypoints']
        self.loops_total = self.mission.get('loops', data.get('loops', 1))
        # Activate localization, then navigation. Each step only proceeds once
        # its lifecycle_manager reports the nodes active (the STARTUP response).
        self._manage_then(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP,
                          self._after_loc_startup)

    def _after_loc_startup(self, ok):
        if not ok:
            self.enqueue(Event.ERROR)
            return
        self._load_map()
        self._publish_initialpose(self.dock)
        self._manage_then(self.nav_mgr, ManageLifecycleNodes.Request.STARTUP,
                          self._after_nav_startup)

    def _after_nav_startup(self, ok):
        # Nav2 is active only now, so it is safe to send the first goal.
        self.enqueue(Event.NAV_READY if ok else Event.ERROR)

    def _after_loc_for_goalpoints(self, ok):
        if ok:
            self._load_map()
        else:
            self.enqueue(Event.ERROR)

    def _manage(self, client, command):
        """Fire-and-forget lifecycle command (used for PAUSE / teardown)."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('lifecycle manager unavailable')
            self.enqueue(Event.ERROR)
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        client.call_async(req)

    def _manage_then(self, client, command, cb):
        """Lifecycle command that calls cb(success) once the transition completes.
        The manage_nodes response only returns after every managed node has
        reached the target state, so this is the readiness signal for STARTUP."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('lifecycle manager unavailable')
            cb(False)
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        client.call_async(req).add_done_callback(
            lambda f: cb(self._manage_ok(f)))

    def _manage_ok(self, future):
        try:
            return future.result().success
        except Exception as exc:
            self.get_logger().error('lifecycle call failed: %s' % exc)
            return False

    def _load_map(self):
        if not self.load_map_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('load_map unavailable')
            self.enqueue(Event.ERROR)
            return
        req = LoadMap.Request()
        req.map_url = self._map_path('.yaml')
        self.load_map_cli.call_async(req)

    def _publish_initialpose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose['x'])
        msg.pose.pose.position.y = float(pose['y'])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose['yaw']))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initialpose_pub.publish(msg)

    def _send_current_waypoint(self):
        self._send_goal(self.waypoints[self.wp_index], self._on_wp_result)

    def _send_goal(self, pose, result_cb):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose server unavailable')
            self.enqueue(Event.ERROR)
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(pose['x'])
        ps.pose.position.y = float(pose['y'])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose['yaw']))
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps
        self.nav_client.send_goal_async(goal).add_done_callback(
            lambda f: self._on_goal_response(f, result_cb))

    def _on_goal_response(self, future, result_cb):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            self.enqueue(Event.ERROR)
            return
        handle.get_result_async().add_done_callback(result_cb)

    def _on_wp_result(self, future):
        if future.result().status != 4:   # 4 == STATUS_SUCCEEDED
            self.enqueue(Event.ERROR)
            return
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.wp_index = 0
            self.loop_index += 1
        if self.loop_index >= self.loops_total:
            self.enqueue(Event.LOOPS_DONE)
        else:
            self._send_current_waypoint()

    def _on_dock_result(self, future):
        self.enqueue(Event.DOCK_REACHED if future.result().status == 4
                     else Event.ERROR)


def main(args=None):
    rclpy.init(args=args)
    node = OperationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_slam()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
