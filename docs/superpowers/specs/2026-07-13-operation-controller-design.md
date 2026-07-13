# Operation Controller — Design

**Date:** 2026-07-13
**Component:** `core_task_controller/Operation_controller.py`
**Role in the task:** the *deterministic executor* in the pipeline
`prompt → LLM → validated mission JSON → deterministic executor → simulator`.

## Purpose

A single ROS 2 node that acts as the conductor for the whole robot. It is a
hierarchical finite-state machine: it takes a validated mission, brings up the
right stack, walks a fixed sequence of phases, and reports progress. It holds no
intelligence and never calls the LLM — the same mission always produces the same
sequence of transitions, which is what makes it auditable.

The LLM and the JSON validator live in a separate node (not built here). They
hand a validated mission to this node over a service. If the executor ever gets
a malformed mission it rejects it and stays put; it does not improvise.

## Architecture

```
[llm_node] --validated JSON--> /submit_mission (srv) --> [Operation_controller / FSM]
                                                              |  drives:
                                    NavigateToPose action ────┤──> Nav2 (per-waypoint)
                              lifecycle_manager (STARTUP/PAUSE)┤──> Nav2 localization + navigation
                                    subprocess launch/kill ────┤──> slam_toolbox (mapping)
                                    map_saver / load_map ──────┤──> saved map
                                    /clicked_point (sub) ──────┘    RViz perimeter capture
```

Design choices (with the reasoning, for defence):

- **Plain Python enum FSM**, driven by a 10 Hz timer `tick()`. No FSM library —
  zero extra dependencies, fully transparent, trivial to step through live. The
  transition logic is pure and unit-testable without ROS.
- **Nav2 `NavigateToPose`, one waypoint at a time.** Reuses the existing
  `core_task_navigation` stack (planning, obstacle avoidance, recovery). Sending
  waypoints individually gives a clean branch point between goals for future
  work (vision-follow, pause) without reshaping the machine.
- **Hybrid node control.** Nav2 is lifecycle-managed, so the conductor drives it
  through Nav2's own `lifecycle_manager` (STARTUP/PAUSE). slam_toolbox on Humble
  is *not* a lifecycle node (verified: it doesn't link the lifecycle library and
  exposes no `change_state` services), so it is launched and killed as a
  subprocess. The two modes are mutually exclusive, so nothing runs twice.
- **Feedback as a latched topic**, not a parameter — `ros2 topic echo`-able and
  recordable for the demo video.

## State model

Hierarchical: `mode` is the superstate, the fine-grained `phase` is the
substate. Every phase transition publishes the phase name on
`/operation_feedback`.

```
MAPPING mode
  INITIALIZATION -> START_MAPPING -> MAPPING -> SAVING ->
  MAP_SAVED_SUCCESSFULLY -> CLOSE_MAPPING ->
  GOALPOINT_COLLECTION -> GOAL_POINTS_SAVED -> IDLE

NAVIGATION mode
  START_NAVIGATION -> PERIMETER -> PERIMETER_COMPLETED ->
  RETURN_TO_DOCK -> DOCKED -> CLOSE_NAVIGATION -> IDLE

any phase --(nav abort / action timeout / subprocess crash / save fail)--> FAULT
```

### Phase behaviour

| Phase | Action | Advances when |
|---|---|---|
| INITIALIZATION | validate mission fields, reset state | always |
| START_MAPPING | `Popen(ros2 launch core_task_mapping mapping.launch.py)`; once slam publishes, record the robot's start pose in the map frame (TF `map`->`base_footprint`) as the **dock pose** | `/map` seen / slam node up + dock captured |
| MAPPING | operator teleops to build the map | operator signals done |
| SAVING | call `nav2_map_server` map_saver, write `<map_name>.yaml/.pgm` | save returns ok |
| MAP_SAVED_SUCCESSFULLY | log saved path | immediately |
| CLOSE_MAPPING | SIGINT the slam subprocess, wait for exit | process exited |
| GOALPOINT_COLLECTION | activate Nav2 **localization** lifecycle (map_server+AMCL); load saved map so RViz shows it; collect `/clicked_point` | operator signals done |
| GOAL_POINTS_SAVED | write `<map_name>_perimeter.yaml` (dock pose + waypoints) | file written |
| START_NAVIGATION | activate Nav2 **navigation** lifecycle; `load_map` `<map_name>.yaml`; load `<map_name>_perimeter.yaml`; publish the saved **dock pose** to `/initialpose` to seed AMCL | Nav2 active + AMCL seeded |
| PERIMETER | send waypoints one-by-one via NavigateToPose, repeat x `loops` | last waypoint of last loop succeeded |
| PERIMETER_COMPLETED | log | immediately |
| RETURN_TO_DOCK | send NavigateToPose to the saved dock pose | dock goal succeeded |
| DOCKED | log return to dock | immediately |
| CLOSE_NAVIGATION | `lifecycle_manager` PAUSE | Nav2 inactive |
| FAULT | halt robot, hold phase, publish `fault:<reason>` | new mission clears it |

"Operator signals done" for MAPPING and GOALPOINT_COLLECTION is an explicit
trigger (a small std_srvs/Trigger service or a `/operation_cmd` topic), not a
timer — the operator decides when the map and the perimeter are complete.

## Interfaces

**Inputs**
- `/submit_mission` (service) — validated mission JSON: `{mode, map_name, loops}`.
- `/clicked_point` (`geometry_msgs/PointStamped`, from RViz) — perimeter capture.
- `/operation_cmd` or a Trigger service — operator "phase done" signal.
- `mode` parameter — manual override for testing without the LLM node.

**Outputs / clients**
- `/operation_feedback` (`std_msgs/String`, latched) — current phase.
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) — seeds AMCL with the dock pose at START_NAVIGATION.
- `NavigateToPose` action client — Nav2 (perimeter waypoints and the dock return).
- Nav2 `lifecycle_manager` `manage_nodes` service clients (localization, navigation).
- `map_server` `map_saver` and `/map_server/load_map` services.
- slam_toolbox subprocess handle.

**Persistence** — everything for a map lives together and is named after it:

```
core_task_navigation/map/
  <map_name>.yaml            # map metadata (map_saver output)
  <map_name>.pgm             # occupancy image
  <map_name>_perimeter.yaml  # clicked perimeter for THIS map
```

`<map_name>_perimeter.yaml`:

```yaml
map: warehouse       # must match the map it was captured on
frame_id: map
loops: 1             # default; mission can override
dock: {x: 6.56, y: 2.18, yaw: 3.14}   # robot start/home; AMCL initial pose + return target
waypoints:
  - {x: 6.56, y: 2.18, yaw: 3.14}
  - {x: 2.10, y: 2.18, yaw: 3.14}
  - {x: 2.10, y: -3.40, yaw: 0.0}
```

Keying the perimeter file to the map name enforces in the filesystem that a
perimeter is only ever driven on the map it was drawn on — no runtime mismatch
check needed, and it scales to many maps. The dock pose (robot start/home) is
captured at the start of mapping and stored in the same file, so navigation can
seed AMCL from it and return to it when the perimeter finishes.

## Error handling

Any Nav2 goal abort, action timeout, slam subprocess crash, or failed map/
perimeter save routes to **FAULT**: the robot is halted, the phase is held, and
the reason is published on `/operation_feedback`. FAULT is cleared only by a new
mission. There is no silent recovery — a failure is visible and stops the run.

## Testing

- **Unit (the real safety net):** the FSM transition function is pure —
  `(phase, event) -> next_phase`. Test the whole table with synthetic events and
  no ROS running. Deterministic, fast, and the thing to defend live.
- **Integration:** a launch-based smoke run (sim + conductor), exercised
  manually for the demo video.

## Files

- `core_task_controller/Operation_controller.py` — the node and the FSM.
- `core_task_controller/function.py` — helpers: mission validation, perimeter
  YAML read/write, pose/quaternion conversion.
- `launch/Operation_controller.launch.py` — conductor + Nav2 (resident,
  `autostart:=false`) + RViz. The Gazebo sim is launched separately.
- `config/` — runtime perimeter files live under `core_task_navigation/map/`;
  any static schema for the mission lives here.

## Out of scope (clean seams left for later)

Operator pause/resume, the vision detect+follow state (task challenge 3), and
multi-robot formations (challenge 1). Each is a new phase or mode against the
same machine; none is built now. FAULT is the only cross-cutting addition, and
it is genuine safety rather than speculation.
