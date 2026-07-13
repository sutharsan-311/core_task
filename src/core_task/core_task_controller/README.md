# core_task_controller

Deterministic executor (`Operation_controller`) plus a `/cmd_vel` limiter for the
TurtleBot3 Waffle Pi in the AWS warehouse. `Operation_controller` is a
hierarchical state machine with two modes — `mapping` and `navigation` — that
orchestrates slam_toolbox and Nav2. It publishes its current phase on the
latched topic `/operation_feedback`.

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

## Phases

```
mapping:     initialization -> start_mapping -> mapping -> saving ->
             map_saved_successfully -> close_mapping ->
             goalpoint_collection -> goal_points_saved -> idle
navigation:  start_navigation -> perimeter -> perimeter_completed ->
             return_to_dock -> docked -> close_navigation -> idle
any phase --(nav abort / timeout / slam crash / save fail)--> fault
```

## Tests

Pure logic (validation, pose math, perimeter I/O, FSM table) is unit-tested and
needs no ROS graph:

```bash
python3 -m pytest src/core_task/core_task_controller/test/ -q -p no:anyio -k function
```
