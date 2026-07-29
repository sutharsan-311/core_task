# core_task_controller

Deterministic executor (`Operation_controller`) plus a `/cmd_vel` limiter for the
TurtleBot3 Waffle Pi in the AWS warehouse. `Operation_controller` is a
hierarchical state machine — `mapping`, `navigation`, `collect_goals`,
`squad_navigation`, `find_person` — that orchestrates slam_toolbox and Nav2. It
publishes its current phase on the latched topic `/operation_feedback`.

See the design and plan:
- `docs/superpowers/specs/2026-07-13-operation-controller-design.md`
- `docs/superpowers/plans/2026-07-14-operation-controller.md`

## Run: mapping

```bash
# 1. sim
ros2 launch core_task_gazebo warehouse.launch.py
# 2. conductor + resident Nav2 (inactive until the conductor activates it)
ros2 launch core_task_controller Operation_controller.launch.py
# 3. teleop, through the /cmd_vel limiter (optional but recommended)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_in
ros2 run core_task_controller cmd_vel_limiter \
  --ros-args --params-file $(ros2 pkg prefix core_task_controller)/share/core_task_controller/config/cmd_vel_limiter.yaml

# submit a mapping mission
ros2 topic pub --once /submit_mission std_msgs/msg/String \
  '{data: "{\"mode\": \"mapping\", \"map_name\": \"warehouse\"}"}'

# drive around to build the map, then save + close slam:
ros2 service call /operator_done std_srvs/srv/Trigger
# click the perimeter corners in RViz (Publish Point), then save the perimeter:
ros2 service call /operator_done std_srvs/srv/Trigger

# watch progress at any time
ros2 topic echo /operation_feedback
```

Outputs (co-located, keyed by map name), under
`core_task_navigation/.../map/`: `warehouse.yaml`, `warehouse.pgm`,
`warehouse_perimeter.yaml` (dock pose + clicked waypoints).

## Run: navigation

```bash
ros2 launch core_task_gazebo warehouse.launch.py
ros2 launch core_task_controller Operation_controller.launch.py
ros2 topic pub --once /submit_mission std_msgs/msg/String \
  '{data: "{\"mode\": \"navigation\", \"map_name\": \"warehouse\", \"loops\": 2}"}'
```

The robot seeds AMCL from the saved dock pose, drives the perimeter `loops`
times, returns to the dock, then pauses Nav2. `/operation_feedback` reports each
phase (`start_navigation`, `perimeter`, `perimeter_completed`, `return_to_dock`,
`docked`, `close_navigation`, `idle`).

## Run: find_person

```bash
ros2 launch core_task_gazebo warehouse.launch.py
ros2 launch core_task_controller Operation_controller.launch.py
ros2 run core_task_perception target_detector
ros2 run core_task_perception person_approach
ros2 topic pub --once /submit_mission std_msgs/msg/String \
  '{data: "{\"mode\": \"find_person\", \"map_name\": \"warehouse\"}"}'
```

Same startup as `navigation` (localize, load the map), then: spin in place
scanning for the person; if not found, walk the perimeter once as a search
route; the moment `target_detector` reports the person, cancel the search goal
and hand `/cmd_vel` to `person_approach`, which steers in and stops once close
(`arrived_area` param — see `core_task_perception/README.md`, needs a one-time
field calibration). The robot stays put near the person; it does not return to
the dock. A full search with nobody found, or losing the target for more than
`lost_target_timeout` seconds mid-approach, faults the mission the same way a
Nav2 failure does anywhere else.

## Run: natural language (optional)

`nlm_interface` accepts a sentence, asks Claude for mission JSON, validates it,
and publishes the result to `/submit_mission`. `bringup.launch.py` starts it by
default:

```bash
pip install anthropic && export ANTHROPIC_API_KEY=sk-...

# terminal 1: sim + Nav2 + executor + language interface
ros2 launch core_task_controller bringup.launch.py
# ...or skip the language interface if you have no API key:
ros2 launch core_task_controller bringup.launch.py nlm:=false

# terminal 2: talk to it
ros2 run core_task_controller nlm_cli
```

`nlm_cli` is a full-screen client: type a mission, watch the transcript and the
robot's phase, `/quit` to exit. To skip it and publish by hand instead:

```bash
ros2 topic pub --once /natural_language_mission std_msgs/msg/String \
  '{data: "Patrol the perimeter twice"}'
ros2 topic echo /nlm_feedback
```

Nothing else depends on it — `/submit_mission` still takes hand-written JSON, and
an invalid or hallucinated mission is rejected before it reaches the executor.
See [README_NLM.md](README_NLM.md).

## Phases

```
mapping:     initialization -> start_mapping -> mapping -> saving ->
             map_saved_successfully -> close_mapping ->
             goalpoint_collection -> goal_points_saved -> idle
navigation:  start_navigation -> perimeter -> perimeter_completed ->
             return_to_dock -> docked -> close_navigation -> idle
find_person: start_navigation -> search_spin -> [search_patrol] ->
             approaching -> arrived -> close_navigation -> idle
             (search_spin/search_patrol skip straight to approaching once
             the person is seen; no dock return - the robot stays put)
perimeter/search_spin/search_patrol/approaching --(abort)--> return_to_dock
any phase --(nav timeout / slam crash / save fail / search exhausted /
             target lost mid-approach)--> fault
```

## Tests

Pure logic (validation, pose math, perimeter I/O, FSM table, and the mocked LLM
client) is unit-tested and needs no ROS graph:

```bash
cd src/core_task/core_task_controller
python3 -m pytest test/ -q
```

(The repo-root `pytest.ini` disables ROS's `launch_testing`/`launch_ros`
plugins, which are incompatible with the installed pytest and would otherwise
abort collection.)
