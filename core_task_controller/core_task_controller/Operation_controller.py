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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap, ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from core_task_controller.function import (
    Event, Phase, load_perimeter, next_phase, quaternion_to_yaw,
    split_waypoints, validate_mission, yaw_to_quaternion)
from core_task_controller.markers import waypoint_markers


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

        # Optional dock override: robot2's instance sets this to its spawn pose
        # ([x, y, yaw]) so it goes home to its own start, not the perimeter
        # file's dock. Default is a 1-element sentinel (length != 3) meaning
        # "unset" -> use the dock from the perimeter file (robot1). rclpy cannot
        # infer the array type from an empty-list default, hence [0.0].
        self.dock_override = list(
            self.declare_parameter('dock', [0.0]).value or [])

        self.phase = Phase.IDLE
        self.mission = None
        self.dock = None                 # [x, y, yaw] captured at mapping start
        self._last_mapped_pose = None    # [x, y, yaw] captured at mapping end
        self.waypoints = []              # loaded perimeter for navigation
        self.loops_total = 1
        self.loop_index = 0
        self.wp_index = 0
        self.slam_proc = None
        self._events = deque()
        self._nav_goal_handle = None     # live NavigateToPose goal, for cancel
        self._aborting = False           # suppress the cancelled waypoint result
        self._save_name = None           # operator's map name for this save only

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.feedback_pub = self.create_publisher(
            String, 'operation_feedback', latched)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        # Latched so RViz shows this robot's perimeter for the whole run.
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints', latched)

        # Perimeter capture lives in the standalone goal_collector node, driven
        # by /submit_mission + /operation_feedback. This node owns the FSM only.
        self.create_subscription(String, 'submit_mission', self._on_mission, 10)
        # Name-carrying "save <name>" for a mapping run; sets the map name, then
        # finishes mapping. (Bare "save"/"done" via operator_done keeps the map's
        # default name and is what goal collection uses.)
        self.create_subscription(String, 'save_map', self._on_save_map, 10)
        self.create_service(Trigger, 'operator_done', self._on_operator_done)
        self.create_service(Trigger, 'abort_mission', self._on_abort)
        self.create_service(Trigger, 'reset', self._on_reset)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.loc_mgr = self.create_client(
            ManageLifecycleNodes, 'lifecycle_manager_localization/manage_nodes')
        self.nav_mgr = self.create_client(
            ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes')
        self.load_map_cli = self.create_client(LoadMap, 'map_server/load_map')

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

    def _on_save_map(self, msg):
        """'save <name>' during mapping: set the map name, then finish + save."""
        if self.phase != Phase.MAPPING:
            return
        name = msg.data.strip()
        if name:
            self._save_name = name
        self.enqueue(Event.OPERATOR_DONE)

    def _on_operator_done(self, request, response):
        if self.phase == Phase.MAPPING:
            self.enqueue(Event.OPERATOR_DONE)
            response.success, response.message = True, 'mapping -> saving'
        elif self.phase == Phase.GOALPOINT_COLLECTION:
            # goal_collector has already written each point in realtime.
            self.enqueue(Event.OPERATOR_DONE)
            response.success, response.message = True, 'collection finished'
        else:
            response.success = False
            response.message = 'no operator action in phase %s' % self.phase.name
        return response

    def _on_abort(self, request, response):
        # Only a live patrol can be aborted. Returning to dock, docked, mapping,
        # or idle: nothing to interrupt, so say so instead of faking an abort.
        if self.phase == Phase.RETURN_TO_DOCK:
            response.success, response.message = True, 'already returning to dock'
            return response
        if self.phase != Phase.PERIMETER:
            response.success = False
            response.message = 'nothing to abort in phase %s' % self.phase.name
            return response

        self.get_logger().warn('ABORT requested - returning to dock')
        self._aborting = True
        # Cancel the in-flight waypoint goal, THEN redirect. Enqueuing ABORT from
        # the cancel callback serialises stop-then-go-home, so the dock goal is
        # not sent while the waypoint goal is still active.
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async().add_done_callback(
                lambda _f: self.enqueue(Event.ABORT))
        else:
            self.enqueue(Event.ABORT)
        response.success, response.message = True, 'aborting -> return to dock'
        return response

    def _on_reset(self, request, response):
        # 'reset' clears a held fault back to idle. Only FAULT has a RESET
        # transition, so in any other phase this is a deliberate no-op rather
        # than a way to yank a live mapping/patrol run back to idle.
        if self.phase != Phase.FAULT:
            response.success = False
            response.message = 'reset only clears a fault (phase %s)' \
                % self.phase.name
            return response
        self.get_logger().warn('RESET requested - tearing down lifecycle, '
                               'clearing fault -> idle')
        self._reset_cleanup()
        self.enqueue(Event.RESET)
        response.success, response.message = True, 'fault cleared -> idle'
        return response

    def _reset_cleanup(self):
        """Best-effort teardown so IDLE is a genuine clean slate: drop any live
        nav goal, stop SLAM, and RESET the nav + localization lifecycles if up.

        Lifecycle RESET (not PAUSE) deactivates AND cleans up, leaving the nodes
        unconfigured - the same state a fresh launch gives them - so the next
        mission's STARTUP is a normal cold start. Unlike _manage(), a missing
        manager here is ignored, not faulted: we are recovering FROM a fault, so
        re-faulting would defeat the reset.
        """
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._aborting = False
        self._stop_slam()
        self.slam_proc = None
        for mgr in (self.nav_mgr, self.loc_mgr):
            if mgr.service_is_ready():
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request.RESET
                mgr.call_async(req)

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
            # slam_toolbox's map->base_footprint is the only accurate fix we'll
            # have on the just-saved map: nav2_params.yaml's initial_pose is a
            # cold-boot default for the original map only, wrong for any map
            # built since. Grab it before the SLAM process (and its TF) dies.
            self._last_mapped_pose = self._lookup_pose()
            self._stop_slam()
        elif phase == Phase.GOALPOINT_COLLECTION:
            self._manage_then(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP,
                              self._after_loc_for_goalpoints)
        elif phase == Phase.GOAL_POINTS_SAVED:
            self._manage(self.loc_mgr, ManageLifecycleNodes.Request.PAUSE)
        elif phase == Phase.START_NAVIGATION:
            self._start_navigation()
        elif phase == Phase.PERIMETER:
            self.loop_index = 0
            self.wp_index = 0
            self._aborting = False       # fresh patrol; clear any prior abort
            if not self.waypoints:       # empty share (more robots than points)
                self.enqueue(Event.LOOPS_DONE)
            else:
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

    def _lookup_pose(self):
        """map->base_footprint right now, or None if unavailable."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return [t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)]

    def _capture_dock(self):
        """Look up map->base_footprint once; store as dock. True when captured."""
        if self.dock is not None:
            return True
        pose = self._lookup_pose()
        if pose is None:
            return False
        self.dock = pose
        self.get_logger().info('dock captured: %s' % self.dock)
        return True

    def _map_path(self, ext=''):
        return os.path.join(self.map_dir, self.mission['map_name'] + ext)

    def _save_map(self):
        # An operator "save <name>" overrides the mission's map name for this
        # save only; reset after so it never leaks into a later nav/collect run.
        name = self._save_name or self.mission['map_name']
        self._save_name = None
        path = os.path.join(self.map_dir, name)
        rc = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path,
             '--occ', '0.65', '--free', '0.196'],
            capture_output=True).returncode
        self.get_logger().info('map saved as "%s" (rc=%s)' % (name, rc))

        # Record the last used map name so goal collection can find it
        last_map_file = os.path.join(self.map_dir, '.last_map')
        try:
            with open(last_map_file, 'w') as f:
                f.write(name)
        except Exception as e:
            self.get_logger().warning('Could not save last map name: %s' % e)

        self.enqueue(Event.SAVE_OK if rc == 0 else Event.ERROR)

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
        if not data.get('waypoints'):
            self.get_logger().error(
                'perimeter has no waypoints - run goal collection first')
            self.enqueue(Event.ERROR)
            return
        self.dock = (self.dock_override
                     if len(self.dock_override) == 3 else data['dock'])
        robots = self.mission.get('robots', 1)
        index = self.mission.get('robot_index', 0)
        self.waypoints = split_waypoints(data['waypoints'], robots, index)
        self.marker_pub.publish(waypoint_markers(self.waypoints))
        self.loops_total = self.mission.get('loops', data.get('loops', 1))
        # Activate localization, then navigation. Each step only proceeds once
        # its lifecycle_manager reports the nodes active (the STARTUP response).
        self._manage_then(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP,
                          self._after_loc_startup)

    def _after_loc_startup(self, ok):
        if not ok:
            self.enqueue(Event.ERROR)
            return
        # map_server keeps serving whatever it last had loaded the instant it
        # (re)activates - it only swaps to this mission's map once this
        # LoadMap call actually completes. Publishing initialpose any sooner
        # risks AMCL evaluating it against the wrong (previous) map.
        self._load_map(self._after_map_loaded_for_nav)

    def _after_map_loaded_for_nav(self, ok):
        if not ok:
            self.enqueue(Event.ERROR)
            return
        self._publish_initialpose(self.dock)
        self._manage_then(self.nav_mgr, ManageLifecycleNodes.Request.STARTUP,
                          self._after_nav_startup)

    def _after_nav_startup(self, ok):
        # Nav2 is active only now, so it is safe to send the first goal.
        self.enqueue(Event.NAV_READY if ok else Event.ERROR)

    def _after_loc_for_goalpoints(self, ok):
        if not ok:
            self.enqueue(Event.ERROR)
            return
        self._load_map(self._after_map_loaded_for_goalpoints)

    def _after_map_loaded_for_goalpoints(self, ok):
        if not ok:
            self.enqueue(Event.ERROR)
            return
        pose, self._last_mapped_pose = self._last_mapped_pose, None
        if pose is None:
            # Not freshly mapped this session - the map's own perimeter file
            # (if this map was ever collected before) has a real dock for it,
            # unlike nav2_params.yaml's initial_pose which only fits the
            # original default map. First-ever collection has neither; AMCL's
            # own cold-boot default is the best guess left at that point.
            pose = self._existing_dock()
        if pose is not None:
            self._publish_initialpose(pose)

    def _existing_dock(self):
        try:
            return load_perimeter(self._map_path('_perimeter.yaml')).get('dock')
        except (FileNotFoundError, OSError):
            return None

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

    def _load_map(self, cb):
        """cb(success) fires once map_server has actually swapped to this
        mission's map - not merely once the request was sent."""
        if not self.load_map_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('load_map unavailable')
            cb(False)
            return
        req = LoadMap.Request()
        req.map_url = self._map_path('.yaml')
        self.load_map_cli.call_async(req).add_done_callback(
            lambda f: cb(self._load_map_ok(f)))

    def _load_map_ok(self, future):
        try:
            return future.result().result == LoadMap.Response.RESULT_SUCCESS
        except Exception as exc:
            self.get_logger().error('load_map call failed: %s' % exc)
            return False

    def _publish_initialpose(self, pose):
        """pose is [x, y, yaw]."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.initialpose_pub.publish(msg)

    def _send_current_waypoint(self):
        self._send_goal(self.waypoints[self.wp_index], self._on_wp_result)

    def _send_goal(self, pose, result_cb):
        """pose is [x, y, yaw], straight from the perimeter file."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose server unavailable')
            self.enqueue(Event.ERROR)
            return
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(pose[0])
        ps.pose.position.y = float(pose[1])
        qx, qy, qz, qw = yaw_to_quaternion(float(pose[2]))
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
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(result_cb)

    def _on_wp_result(self, future):
        self._nav_goal_handle = None
        if self._aborting:
            # The goal we just cancelled reports non-success; that is the abort
            # working, not a fault. _on_abort already redirected us to the dock.
            return
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
        self._nav_goal_handle = None
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
