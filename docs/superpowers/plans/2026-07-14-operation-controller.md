# Operation Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `Operation_controller`, a deterministic hierarchical state machine that runs the robot through a mapping run and a perimeter-navigation run, orchestrating slam_toolbox and Nav2.

**Architecture:** One rclpy node. All decision logic is a pure, ROS-free transition table in `function.py` (unit-tested); the node maps runtime conditions to events, applies the table, and performs the side effects (launch slam, activate Nav2 lifecycle, send NavigateToPose goals). Two mutually exclusive modes: `mapping` and `navigation`.

**Tech Stack:** ROS 2 Humble, `ament_python`, rclpy, nav2_msgs (action + lifecycle + load_map), tf2_ros, PyYAML, std_srvs/Trigger.

## Global Constraints

- ROS 2 **Humble**, package build type **ament_python**.
- **No new interface (msg/srv) package.** Mission arrives as JSON on a `std_msgs/String` topic; operator "done" is `std_srvs/Trigger`. (Custom srv would force an ament_cmake interface package — out of scope.)
- Robot frames: fixed frame **`map`**, base frame **`base_footprint`**.
- Feedback is a **latched** (`TRANSIENT_LOCAL`) `std_msgs/String` topic `/operation_feedback`.
- Map + perimeter live together in `core_task_navigation/map/`, named `<map_name>.yaml/.pgm` and `<map_name>_perimeter.yaml`.
- Git: repo hook forbids a `Co-Authored-By` trailer — do not add one. Work on branch `feature/operation-controller`.
- Package dir: `src/core_task/core_task_controller/`. The node file is `core_task_controller/Operation_controller.py` (already exists, empty). Pure helpers go in `core_task_controller/function.py` (already exists, empty).

---

### Task 1: Package dependencies and entry point

**Files:**
- Modify: `src/core_task/core_task_controller/package.xml`
- Modify: `src/core_task/core_task_controller/setup.py`

**Interfaces:**
- Consumes: nothing.
- Produces: `ros2 run core_task_controller operation_controller` executable target.

- [ ] **Step 1: Add exec dependencies to package.xml**

In `package.xml`, replace the existing exec_depend block with:

```xml
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>nav2_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>core_task_navigation</exec_depend>
```

- [ ] **Step 2: Register the entry point in setup.py**

In `setup.py`, `entry_points['console_scripts']` list, add alongside the existing `cmd_vel_limiter` line:

```python
            'operation_controller = core_task_controller.Operation_controller:main',
```

- [ ] **Step 3: Build to verify packaging**

Run: `cd ~/omokai_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select core_task_controller`
Expected: `Finished <<< core_task_controller` (the entry point won't run yet — module is empty — but the package builds).

- [ ] **Step 4: Commit**

```bash
git add src/core_task/core_task_controller/package.xml src/core_task/core_task_controller/setup.py
git commit -m "operation_controller: package deps and entry point"
```

---

### Task 2: `function.py` — mission validation

**Files:**
- Create/modify: `src/core_task/core_task_controller/core_task_controller/function.py`
- Test: `src/core_task/core_task_controller/test/test_function_mission.py`

**Interfaces:**
- Produces: `validate_mission(mission: dict) -> tuple[bool, str]` — returns `(True, "ok")` or `(False, reason)`.

- [ ] **Step 1: Write the failing test**

Create `test/test_function_mission.py`:

```python
from core_task_controller.function import validate_mission


def test_valid_mapping():
    ok, _ = validate_mission({'mode': 'mapping', 'map_name': 'warehouse'})
    assert ok is True


def test_valid_navigation_with_loops():
    ok, _ = validate_mission(
        {'mode': 'navigation', 'map_name': 'warehouse', 'loops': 2})
    assert ok is True


def test_bad_mode():
    ok, reason = validate_mission({'mode': 'fly', 'map_name': 'w'})
    assert ok is False and 'mode' in reason


def test_missing_map_name():
    ok, reason = validate_mission({'mode': 'mapping'})
    assert ok is False and 'map_name' in reason


def test_bad_loops():
    ok, reason = validate_mission(
        {'mode': 'navigation', 'map_name': 'w', 'loops': 0})
    assert ok is False and 'loops' in reason


def test_not_a_dict():
    ok, _ = validate_mission("nope")
    assert ok is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd ~/omokai_ws && source install/setup.bash && python3 -m pytest src/core_task/core_task_controller/test/test_function_mission.py -q -p no:cacheprovider`
Expected: FAIL with `ImportError` / `cannot import name 'validate_mission'`.

- [ ] **Step 3: Write minimal implementation**

In `function.py`:

```python
"""Pure, ROS-free logic for Operation_controller: mission validation, the FSM
transition table, perimeter file I/O, and pose math. No rclpy imports here so
it stays unit-testable without a ROS graph."""

VALID_MODES = ('mapping', 'navigation')


def validate_mission(mission):
    """Return (ok, reason). A mission is {mode, map_name, loops?}."""
    if not isinstance(mission, dict):
        return False, 'mission must be a JSON object'
    mode = mission.get('mode')
    if mode not in VALID_MODES:
        return False, "mode must be 'mapping' or 'navigation'"
    if not mission.get('map_name'):
        return False, 'map_name is required'
    if mode == 'navigation':
        loops = mission.get('loops', 1)
        if not isinstance(loops, int) or loops < 1:
            return False, 'loops must be a positive integer'
    return True, 'ok'
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_mission.py -q -p no:cacheprovider`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add src/core_task/core_task_controller/core_task_controller/function.py src/core_task/core_task_controller/test/test_function_mission.py
git commit -m "operation_controller: mission validation"
```

---

### Task 3: `function.py` — yaw / quaternion conversion

**Files:**
- Modify: `src/core_task/core_task_controller/core_task_controller/function.py`
- Test: `src/core_task/core_task_controller/test/test_function_pose.py`

**Interfaces:**
- Produces: `yaw_to_quaternion(yaw: float) -> tuple[float,float,float,float]` (x,y,z,w); `quaternion_to_yaw(x,y,z,w) -> float`.

- [ ] **Step 1: Write the failing test**

Create `test/test_function_pose.py`:

```python
import math
from core_task_controller.function import yaw_to_quaternion, quaternion_to_yaw


def test_zero_yaw():
    x, y, z, w = yaw_to_quaternion(0.0)
    assert (x, y, z) == (0.0, 0.0, 0.0) and abs(w - 1.0) < 1e-9


def test_roundtrip():
    for yaw in (-2.0, -0.5, 0.0, 1.2, 3.0):
        x, y, z, w = yaw_to_quaternion(yaw)
        assert abs(quaternion_to_yaw(x, y, z, w) - yaw) < 1e-6


def test_half_pi():
    _, _, z, w = yaw_to_quaternion(math.pi / 2)
    assert abs(z - math.sqrt(0.5)) < 1e-9 and abs(w - math.sqrt(0.5)) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_pose.py -q -p no:cacheprovider`
Expected: FAIL with import error.

- [ ] **Step 3: Write minimal implementation**

Append to `function.py`:

```python
import math


def yaw_to_quaternion(yaw):
    """Yaw (rad) -> (x, y, z, w) for a flat-ground robot."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x, y, z, w):
    """Full-quaternion -> yaw (rad)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_pose.py -q -p no:cacheprovider`
Expected: PASS (3 passed).

- [ ] **Step 5: Commit**

```bash
git add src/core_task/core_task_controller/core_task_controller/function.py src/core_task/core_task_controller/test/test_function_pose.py
git commit -m "operation_controller: yaw/quaternion helpers"
```

---

### Task 4: `function.py` — perimeter file I/O

**Files:**
- Modify: `src/core_task/core_task_controller/core_task_controller/function.py`
- Test: `src/core_task/core_task_controller/test/test_function_perimeter.py`

**Interfaces:**
- Produces:
  - `save_perimeter(path: str, map_name: str, dock: dict, waypoints: list[dict], loops: int = 1) -> None` — writes YAML. `dock` and each waypoint are `{'x':float,'y':float,'yaw':float}`.
  - `load_perimeter(path: str) -> dict` — returns `{'map','frame_id','loops','dock','waypoints'}`.

- [ ] **Step 1: Write the failing test**

Create `test/test_function_perimeter.py`:

```python
import os
from core_task_controller.function import save_perimeter, load_perimeter


def test_save_then_load(tmp_path):
    path = os.path.join(tmp_path, 'warehouse_perimeter.yaml')
    dock = {'x': 6.56, 'y': 2.18, 'yaw': 3.14}
    wps = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0}, {'x': 3.0, 'y': 2.0, 'yaw': 1.57}]
    save_perimeter(path, 'warehouse', dock, wps, loops=2)

    data = load_perimeter(path)
    assert data['map'] == 'warehouse'
    assert data['frame_id'] == 'map'
    assert data['loops'] == 2
    assert data['dock'] == dock
    assert data['waypoints'] == wps
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_perimeter.py -q -p no:cacheprovider`
Expected: FAIL with import error.

- [ ] **Step 3: Write minimal implementation**

Append to `function.py`:

```python
import yaml


def save_perimeter(path, map_name, dock, waypoints, loops=1):
    data = {
        'map': map_name,
        'frame_id': 'map',
        'loops': loops,
        'dock': dock,
        'waypoints': waypoints,
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)


def load_perimeter(path):
    with open(path) as f:
        return yaml.safe_load(f)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_perimeter.py -q -p no:cacheprovider`
Expected: PASS (1 passed).

- [ ] **Step 5: Commit**

```bash
git add src/core_task/core_task_controller/core_task_controller/function.py src/core_task/core_task_controller/test/test_function_perimeter.py
git commit -m "operation_controller: perimeter YAML I/O"
```

---

### Task 5: `function.py` — FSM phases, events, transition table

**Files:**
- Modify: `src/core_task/core_task_controller/core_task_controller/function.py`
- Test: `src/core_task/core_task_controller/test/test_function_fsm.py`

**Interfaces:**
- Produces:
  - `Phase` (Enum): `IDLE, INITIALIZATION, START_MAPPING, MAPPING, SAVING, MAP_SAVED_SUCCESSFULLY, CLOSE_MAPPING, GOALPOINT_COLLECTION, GOAL_POINTS_SAVED, START_NAVIGATION, PERIMETER, PERIMETER_COMPLETED, RETURN_TO_DOCK, DOCKED, CLOSE_NAVIGATION, FAULT`.
  - `Event` (Enum): `SUBMIT_MAPPING, SUBMIT_NAV, INIT_MAPPING, INIT_NAV, SUBMIT_INVALID, SLAM_READY, OPERATOR_DONE, SAVE_OK, SLAM_CLOSED, PERIMETER_SAVED, NAV_READY, LOOPS_DONE, DOCK_REACHED, NAV_CLOSED, ADVANCE, ERROR`.
  - `next_phase(phase: Phase, event: Event) -> Phase` — pure. `ERROR` always yields `FAULT`; an unmapped `(phase, event)` yields `phase` unchanged.

- [ ] **Step 1: Write the failing test**

Create `test/test_function_fsm.py`:

```python
from core_task_controller.function import Phase, Event, next_phase


def run(seq, start=Phase.IDLE):
    p = start
    for ev in seq:
        p = next_phase(p, ev)
    return p


def test_mapping_happy_path():
    seq = [Event.SUBMIT_MAPPING, Event.INIT_MAPPING, Event.SLAM_READY,
           Event.OPERATOR_DONE, Event.SAVE_OK, Event.ADVANCE,
           Event.SLAM_CLOSED, Event.OPERATOR_DONE, Event.ADVANCE]
    assert run(seq) == Phase.IDLE


def test_navigation_happy_path():
    seq = [Event.SUBMIT_NAV, Event.INIT_NAV, Event.NAV_READY,
           Event.LOOPS_DONE, Event.ADVANCE, Event.DOCK_REACHED,
           Event.ADVANCE, Event.NAV_CLOSED]
    assert run(seq) == Phase.IDLE


def test_operator_done_branches_by_phase():
    # OPERATOR_DONE means SAVING from MAPPING...
    assert next_phase(Phase.MAPPING, Event.OPERATOR_DONE) == Phase.SAVING
    # ...but GOAL_POINTS_SAVED from GOALPOINT_COLLECTION
    assert (next_phase(Phase.GOALPOINT_COLLECTION, Event.OPERATOR_DONE)
            == Phase.GOAL_POINTS_SAVED)


def test_error_always_faults():
    for p in (Phase.MAPPING, Phase.PERIMETER, Phase.SAVING):
        assert next_phase(p, Event.ERROR) == Phase.FAULT


def test_invalid_mission_faults():
    assert next_phase(Phase.INITIALIZATION, Event.SUBMIT_INVALID) == Phase.FAULT


def test_new_mission_clears_fault():
    assert next_phase(Phase.FAULT, Event.SUBMIT_NAV) == Phase.INITIALIZATION


def test_unmapped_event_is_noop():
    assert next_phase(Phase.MAPPING, Event.SLAM_READY) == Phase.MAPPING
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_fsm.py -q -p no:cacheprovider`
Expected: FAIL with import error.

- [ ] **Step 3: Write minimal implementation**

Append to `function.py`:

```python
from enum import Enum, auto


class Phase(Enum):
    IDLE = auto()
    INITIALIZATION = auto()
    START_MAPPING = auto()
    MAPPING = auto()
    SAVING = auto()
    MAP_SAVED_SUCCESSFULLY = auto()
    CLOSE_MAPPING = auto()
    GOALPOINT_COLLECTION = auto()
    GOAL_POINTS_SAVED = auto()
    START_NAVIGATION = auto()
    PERIMETER = auto()
    PERIMETER_COMPLETED = auto()
    RETURN_TO_DOCK = auto()
    DOCKED = auto()
    CLOSE_NAVIGATION = auto()
    FAULT = auto()


class Event(Enum):
    SUBMIT_MAPPING = auto()
    SUBMIT_NAV = auto()
    INIT_MAPPING = auto()
    INIT_NAV = auto()
    SUBMIT_INVALID = auto()
    SLAM_READY = auto()
    OPERATOR_DONE = auto()
    SAVE_OK = auto()
    SLAM_CLOSED = auto()
    PERIMETER_SAVED = auto()
    NAV_READY = auto()
    LOOPS_DONE = auto()
    DOCK_REACHED = auto()
    NAV_CLOSED = auto()
    ADVANCE = auto()
    ERROR = auto()


_TRANSITIONS = {
    (Phase.IDLE, Event.SUBMIT_MAPPING): Phase.INITIALIZATION,
    (Phase.IDLE, Event.SUBMIT_NAV): Phase.INITIALIZATION,
    (Phase.FAULT, Event.SUBMIT_MAPPING): Phase.INITIALIZATION,
    (Phase.FAULT, Event.SUBMIT_NAV): Phase.INITIALIZATION,
    (Phase.INITIALIZATION, Event.INIT_MAPPING): Phase.START_MAPPING,
    (Phase.INITIALIZATION, Event.INIT_NAV): Phase.START_NAVIGATION,
    (Phase.INITIALIZATION, Event.SUBMIT_INVALID): Phase.FAULT,
    (Phase.START_MAPPING, Event.SLAM_READY): Phase.MAPPING,
    (Phase.MAPPING, Event.OPERATOR_DONE): Phase.SAVING,
    (Phase.SAVING, Event.SAVE_OK): Phase.MAP_SAVED_SUCCESSFULLY,
    (Phase.MAP_SAVED_SUCCESSFULLY, Event.ADVANCE): Phase.CLOSE_MAPPING,
    (Phase.CLOSE_MAPPING, Event.SLAM_CLOSED): Phase.GOALPOINT_COLLECTION,
    (Phase.GOALPOINT_COLLECTION, Event.OPERATOR_DONE): Phase.GOAL_POINTS_SAVED,
    (Phase.GOAL_POINTS_SAVED, Event.ADVANCE): Phase.IDLE,
    (Phase.START_NAVIGATION, Event.NAV_READY): Phase.PERIMETER,
    (Phase.PERIMETER, Event.LOOPS_DONE): Phase.PERIMETER_COMPLETED,
    (Phase.PERIMETER_COMPLETED, Event.ADVANCE): Phase.RETURN_TO_DOCK,
    (Phase.RETURN_TO_DOCK, Event.DOCK_REACHED): Phase.DOCKED,
    (Phase.DOCKED, Event.ADVANCE): Phase.CLOSE_NAVIGATION,
    (Phase.CLOSE_NAVIGATION, Event.NAV_CLOSED): Phase.IDLE,
}


def next_phase(phase, event):
    """Pure transition. ERROR -> FAULT from anywhere; unmapped pair is a no-op."""
    if event == Event.ERROR:
        return Phase.FAULT
    return _TRANSITIONS.get((phase, event), phase)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest src/core_task/core_task_controller/test/test_function_fsm.py -q -p no:cacheprovider`
Expected: PASS (7 passed).

- [ ] **Step 5: Run the whole function.py suite and commit**

Run: `python3 -m pytest src/core_task/core_task_controller/test/ -q -p no:cacheprovider -k function`
Expected: PASS (all function tests).

```bash
git add src/core_task/core_task_controller/core_task_controller/function.py src/core_task/core_task_controller/test/test_function_fsm.py
git commit -m "operation_controller: FSM phases, events, transition table"
```

---

### Task 6: `Operation_controller.py` — the node

**Files:**
- Create/modify: `src/core_task/core_task_controller/core_task_controller/Operation_controller.py`

**Interfaces:**
- Consumes: everything from `function.py` (Task 2–5).
- Produces: `main(args=None)` entry point; node `operation_controller`.
- Topics/services used at runtime:
  - Sub `/submit_mission` (`std_msgs/String`, JSON), `/clicked_point` (`geometry_msgs/PointStamped`).
  - Srv server `/operator_done` (`std_srvs/Trigger`).
  - Pub `/operation_feedback` (`std_msgs/String`, latched), `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`).
  - Action client `NavigateToPose` on `navigate_to_pose`.
  - Srv clients `/lifecycle_manager_localization/manage_nodes`, `/lifecycle_manager_navigation/manage_nodes` (`nav2_msgs/ManageLifecycleNodes`), `/map_server/load_map` (`nav2_msgs/LoadMap`).
  - Subprocess: `ros2 launch core_task_mapping mapping.launch.py`; `ros2 run nav2_map_server map_saver_cli`.

- [ ] **Step 1: Write the node file**

Create `core_task_controller/Operation_controller.py` with the full contents:

```python
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
        default_map_dir = os.path.join(
            get_package_share_directory('core_task_navigation'), 'map')
        self.map_dir = self.declare_parameter('map_dir', default_map_dir).value

        self.phase = Phase.IDLE
        self.mission = None
        self.dock = None                 # {'x','y','yaw'} captured at mapping start
        self.points = []                 # collected clicked perimeter points
        self.waypoints = []              # loaded perimeter for navigation
        self.loops_total = 1
        self.loop_index = 0
        self.wp_index = 0
        self.slam_proc = None
        self._events = deque()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.feedback_pub = self.create_publisher(
            String, 'operation_feedback', latched)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)

        self.create_subscription(String, 'submit_mission', self._on_mission, 10)
        self.create_subscription(PointStamped, 'clicked_point',
                                 self._on_clicked_point, 10)
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
        self.enqueue(Event.SUBMIT_MAPPING if mission['mode'] == 'mapping'
                     else Event.SUBMIT_NAV)

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

    def _on_clicked_point(self, msg):
        if self.phase == Phase.GOALPOINT_COLLECTION:
            self.points.append(
                {'x': float(msg.point.x), 'y': float(msg.point.y), 'yaw': 0.0})
            self.get_logger().info('perimeter point %d captured' % len(self.points))

    # ---- entry side effects --------------------------------------------
    def _on_enter(self, phase):
        if phase == Phase.INITIALIZATION:
            self.enqueue(Event.INIT_MAPPING if self.mission['mode'] == 'mapping'
                         else Event.INIT_NAV)
        elif phase == Phase.START_MAPPING:
            self._start_slam()
        elif phase == Phase.SAVING:
            self._save_map()
        elif phase == Phase.CLOSE_MAPPING:
            self._stop_slam()
        elif phase == Phase.GOALPOINT_COLLECTION:
            self.points = []
            self._manage(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP)
            self._load_map()
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
            self.get_logger().error('FAULT — holding')

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
        save_perimeter(self._map_path('_perimeter.yaml'),
                       self.mission['map_name'], self.dock, self.points,
                       loops=self.mission.get('loops', 1))
        self.get_logger().info('perimeter saved (%d points)' % len(self.points))

    # ---- navigation -----------------------------------------------------
    def _start_navigation(self):
        self._manage(self.loc_mgr, ManageLifecycleNodes.Request.STARTUP)
        self._manage(self.nav_mgr, ManageLifecycleNodes.Request.STARTUP)
        self._load_map()
        data = load_perimeter(self._map_path('_perimeter.yaml'))
        self.dock = data['dock']
        self.waypoints = data['waypoints']
        self.loops_total = self.mission.get('loops', data.get('loops', 1))
        self._publish_initialpose(self.dock)
        self.enqueue(Event.NAV_READY)

    def _manage(self, client, command):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('lifecycle manager unavailable')
            self.enqueue(Event.ERROR)
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        client.call_async(req)

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
```

- [ ] **Step 2: Build**

Run: `cd ~/omokai_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select core_task_controller`
Expected: `Finished <<< core_task_controller`.

- [ ] **Step 3: Import smoke test (no ROS graph needed to import)**

Run: `source install/setup.bash && python3 -c "import core_task_controller.Operation_controller as m; assert hasattr(m, 'main'); print('import OK')"`
Expected: `import OK`.

- [ ] **Step 4: Node starts and reaches IDLE**

Run:
```bash
source install/setup.bash
ros2 run core_task_controller operation_controller &
sleep 4
ros2 topic echo /operation_feedback --once
kill %1
```
Expected: `data: idle`.

- [ ] **Step 5: Commit**

```bash
git add src/core_task/core_task_controller/core_task_controller/Operation_controller.py
git commit -m "operation_controller: FSM node (slam + Nav2 orchestration)"
```

---

### Task 7: Nav2 launch — expose `autostart` so the conductor can keep it inactive

**Files:**
- Modify: `src/core_task/core_task_navigation/launch/navigation.launch.py`

**Interfaces:**
- Produces: `autostart` launch arg (default `true`) forwarded to nav2 bringup. The conductor launch passes `autostart:=false`.

- [ ] **Step 1: Add the autostart launch argument and forward it**

In `navigation.launch.py`, add an `autostart` LaunchConfiguration and declaration next to the existing `use_sim_time` ones:

```python
    autostart = LaunchConfiguration('autostart', default='true')
```
add to the `declare` list:
```python
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto-activate Nav2 lifecycle on launch'),
```
and add to the `nav2` include's `launch_arguments`:
```python
            'autostart': autostart,
```

- [ ] **Step 2: Build and verify the arg exists**

Run:
```bash
cd ~/omokai_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select core_task_navigation && source install/setup.bash
ros2 launch core_task_navigation navigation.launch.py --show-args | grep -A1 autostart
```
Expected: the `autostart` argument is listed with default `true`.

- [ ] **Step 3: Commit**

```bash
git add src/core_task/core_task_navigation/launch/navigation.launch.py
git commit -m "core_task_navigation: expose autostart launch arg"
```

---

### Task 8: `Operation_controller.launch.py` — conductor + resident (inactive) Nav2 + RViz

**Files:**
- Create/modify: `src/core_task/core_task_controller/launch/Operation_controller.launch.py`

**Interfaces:**
- Consumes: `operation_controller` executable (Task 6); `core_task_navigation` `navigation.launch.py` with `autostart` (Task 7).
- Produces: `ros2 launch core_task_controller Operation_controller.launch.py`.

- [ ] **Step 1: Write the launch file**

Create `launch/Operation_controller.launch.py`:

```python
#!/usr/bin/env python3
"""Bring up the deterministic executor plus a resident (inactive) Nav2 stack.

The Gazebo sim is launched separately (core_task_gazebo warehouse.launch.py).
Nav2 comes up with autostart:=false so it sits inactive until the conductor
activates localization / navigation via lifecycle. slam_toolbox is NOT launched
here — the conductor starts it as a subprocess during mapping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('core_task_navigation')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'autostart': 'false'}.items())

    controller = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        output='screen')

    return LaunchDescription([nav2, controller])
```

- [ ] **Step 2: Build**

Run: `cd ~/omokai_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select core_task_controller && source install/setup.bash`
Expected: `Finished <<< core_task_controller`.

- [ ] **Step 3: Launch parses and starts (sim not required to parse)**

Run: `ros2 launch core_task_controller Operation_controller.launch.py &` then after 5s `ros2 node list | grep operation_controller`; then `kill %1`.
Expected: `/operation_controller` present (Nav2 nodes will also appear, inactive).

- [ ] **Step 4: Commit**

```bash
git add src/core_task/core_task_controller/launch/Operation_controller.launch.py
git commit -m "operation_controller: conductor launch (resident inactive Nav2)"
```

---

### Task 9: Run documentation

**Files:**
- Create: `src/core_task/core_task_controller/README.md`

**Interfaces:**
- Consumes: all prior tasks.
- Produces: operator-facing run instructions.

- [ ] **Step 1: Write the README**

Create `README.md`:

````markdown
# core_task_controller

Deterministic executor (`Operation_controller`) plus a `/cmd_vel` limiter.

## Run: mapping

```bash
# 1. sim
ros2 launch core_task_gazebo warehouse.launch.py
# 2. conductor + resident Nav2 (inactive)
ros2 launch core_task_controller Operation_controller.launch.py
# 3. teleop (through the limiter, optional)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_in

# submit a mapping mission
ros2 topic pub --once /submit_mission std_msgs/String \
  '{data: "{\"mode\": \"mapping\", \"map_name\": \"warehouse\"}"}'
# drive to build the map, then:
ros2 service call /operator_done std_srvs/srv/Trigger   # save map, close slam
# click perimeter corners in RViz (Publish Point), then:
ros2 service call /operator_done std_srvs/srv/Trigger   # save <map>_perimeter.yaml

# watch progress
ros2 topic echo /operation_feedback
```

## Run: navigation

```bash
ros2 launch core_task_gazebo warehouse.launch.py
ros2 launch core_task_controller Operation_controller.launch.py
ros2 topic pub --once /submit_mission std_msgs/String \
  '{data: "{\"mode\": \"navigation\", \"map_name\": \"warehouse\", \"loops\": 2}"}'
```

The robot seeds AMCL from the saved dock, drives the perimeter `loops` times,
returns to the dock, then pauses Nav2. `/operation_feedback` reports each phase.
````

- [ ] **Step 2: Commit**

```bash
git add src/core_task/core_task_controller/README.md
git commit -m "operation_controller: run documentation"
```

---

## Notes / deliberate simplifications

- **Clicked perimeter points have `yaw: 0.0`** — `/clicked_point` carries no orientation. If waypoint heading matters, switch capture to `/goal_pose` (RViz "2D Nav Goal", a `PoseStamped`) and read its yaw. `ponytail: yaw=0 for clicked points; upgrade to /goal_pose if heading matters.`
- **Lifecycle calls are fire-and-forget** (`call_async`, not awaited). Good enough for the demo; if activation races the first goal, add a state check on the `manage_nodes` response.
- **`_save_map` blocks** the executor for the ~1–2 s the map_saver runs. Acceptable at 10 Hz for a single save; move to async if it ever stalls the tick noticeably.
```
