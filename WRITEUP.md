# Omokai — Submission Write-Up

## Executive Summary

**Omokai** is a prompt-driven ground robot control system that executes natural-language missions in simulation. A TurtleBot3 Waffle Pi navigates a warehouse world under ROS 2, guided by an LLM that translates English instructions into structured, validated mission JSON. The same JSON always produces the same behavior; the LLM proposes, a validator gates, and a deterministic state machine executes. All three senior challenges are wired in: multi-agent squad patrol, SLAM mapping, and vision-based target detection and follow.

---

## 1. Architecture

### Pipeline Overview

```
Prompt  →  LLM  →  Validated JSON  →  Executor FSM  →  Nav2/Gazebo
(nlm_cli) (Bedrock) (schema check)  (state machine)    (simulator)
```

### Core Components

**1. LLM Front-End (`core_task_controller/nlm_interface`)**
- Accepts natural-language commands from `nlm_cli` (the mission prompt)
- Sends structured prompts to an LLM via one of three providers:
  - **AWS Bedrock** (default, if `AWS_BEARER_TOKEN_BEDROCK` or AWS credentials set)
  - **Claude API** (Anthropic, if `ANTHROPIC_API_KEY` set)
  - **OpenAI** (if `OPENAI_API_KEY` set)
- The LLM returns JSON; **it never touches the control loop**
- Fallback: if no LLM credentials, drive the executor directly via ROS topics (`nlm:=false`)

**2. Mission Validator (`core_task_controller/mission_validator.py`)**
- Checks returned JSON against a schema: mission type, robot(s), waypoints, speeds
- Validates safety: waypoints must be in known maps, speeds within limits (0–0.5 m/s), valid command sequences
- Drops malformed or unsafe missions before any wheel turns
- Example: a prompt to "go 100 m/s" is rejected; one to "patrol the perimeter twice" passes

**3. Deterministic Executor (`core_task_controller/operation_controller.py`)**
- A finite-state machine (FSM) that reads validated JSON and issues concrete commands
- States: idle → localize (if needed) → navigate → return to dock → idle
- Publishes waypoint markers to RViz for real-time feedback
- Audit trail: every mission and its execution can be traced and replayed
- **Reproducibility**: same JSON + same ROS 2 state = same behavior always

**4. Navigation Stack**
- **Nav2** (from ros-humble-navigation2) handles path planning and obstacle avoidance
- **AMCL** localizes the robot on a known map using its LiDAR and odometry
- **RViz2** visualizes the map, costmaps, and robot state in real time

**5. Simulation**
- **Gazebo Classic 11** with a warehouse world (AWS RoboMaker Small Warehouse)
- **TurtleBot3 Waffle Pi**: differential drive robot, LiDAR (12 m range by default), RGB camera
- Two robots supported: Robot 1 (global namespace), Robot 2 (isolated under `/robot2`)

---

## 2. Challenges Implemented

### Challenge 1: Multi-Agent Squad Patrol ✅

**What it does:**
- User issues `"both robots patrol the perimeter, split it between them"`
- Two robots spawn in Gazebo (robot 1 at the dock, robot 2 at origin)
- A squad coordinator divides the perimeter waypoints between them
- Both navigate their half of the loop autonomously in parallel

**How it works:**
- The LLM understands multi-robot intent and returns a squad mission JSON with role assignments (`"leader"`, `"follower"`)
- `OperationController` spawns one FSM per robot; each reads its waypoint segment
- Nav2's per-robot namespaced stacks keep TF and topics (`/robot2/tf`, `/robot2/cmd_vel`, etc.) isolated
- Each robot's RSP publishes its own TF tree; collision-free navigation via separate costmaps

**Key design decision:**
- No dynamic formation control: robots follow their assigned waypoints independently, not as a tight formation
- This is simpler and more robust in a cluttered warehouse (no collision prediction needed between robots)
- Scaling: a true formation layer (leader-follower geometry) would add coordinated feedback; we chose autonomy + deconfliction via waypoint splitting

---

### Challenge 2: SLAM (Simultaneous Localization and Mapping) ✅

**What it does:**
- User types `"start building a map called floor2"` → robot enters SLAM mode
- Gazebo + Nav2 + SLAM Toolbox bring up online mapping
- Robot drives around the warehouse (manually guided via arrow keys or direct teleoperation)
- RViz shows the map filling in real time
- User says `"done"` → map is saved as `floor2_perimeter.yaml`

**How it works:**
- `core_task_mapping/launch/mapping.launch.py` brings up SLAM Toolbox with a TurtleBot3 config
- SLAM Toolbox subscribes to `/scan` (LiDAR) and `/odom` (odometry) and builds an `occupancy_grid`
- Map Server loads the grid and provides a `/map` service
- Nav2 can now localize on the map for subsequent patrol missions
- `LocalizationMode` FSM state in the executor calls the SLAM nodes and monitors map output

**Why it matters:**
- Without SLAM, the robot can only follow pre-recorded waypoints (Challenge 3-ready scenario)
- With SLAM, the robot can explore unknown spaces and return to known regions (real-world critical)

**Key trade-offs:**
- We use SLAM Toolbox (CPU-light, 2D) over RTAB-Map (3D, heavier)
- Good for warehouse aisles; not sufficient for multi-floor or large-scale spaces
- Scaling: Larger environments need hierarchical SLAM or cloud-based mapping

---

### Challenge 3: Vision AI — Detect and Follow ✅

**What it does:**
- User types `"find and follow people"` (or any configured target class)
- YOLO (YOLOv8 nano) runs on the robot's camera feed
- When a person is detected, the robot:
  1. Publishes an annotated image for the operator (bounding box, confidence)
  2. Automatically drives toward the target
  3. Maintains a safe distance (1–2 m) while tracking

**How it works:**
- `core_task_perception/target_detector.py`:
  - Subscribes to `/camera/image_raw` from Gazebo's simulated camera
  - Runs YOLOv8 inference (class-filtered: e.g., only "person" detections)
  - Publishes annotated image and target pose to RViz
  
- `core_task_perception/target_mover.py`:
  - Spawns a moving "person" in Gazebo (a simple DAE mesh)
  - Drives it in a pattern (loop, random walk) for testing
  
- `OperationController` FSM state `FollowTarget`:
  - Receives target bounding box from the detector
  - Calculates desired robot heading and forward speed
  - Issues `/cmd_vel` commands to move toward the target

**Key design decision:**
- **Detector is a sensor, not a planner**: YOLO feeds perception to the executor, which decides if/how to follow
- If confidence is low (<50%), follow is suppressed; operator always has veto
- The robot respects Nav2 costmaps (doesn't drive into obstacles toward a target)

**Scaling story:**
- Real-world: edge-deploy YOLOv8 on Nvidia Jetson, not in ROS; publish class-agnostic detections
- Multi-target: track multiple people simultaneously (maintain closest, switch on input)
- Failure handling: loss-of-target triggers return-to-dock (not endless search)

---

## 3. Design Decisions & Trade-Offs

### Why LLM is Out of the Control Loop

**Problem:** Naive integration runs raw LLM output as robot commands. This is unsafe (hallucinated waypoints), slow (high latency), and not auditable.

**Solution:** LLM → JSON → validation → execution
- LLM's job: translate intent (propose a mission)
- Validator's job: check safety and correctness
- Executor's job: drive the robot deterministically

**Benefit:** Reproducibility. The same validated JSON always produces the same robot behavior, independent of LLM output variability.

### Single Perimeter Loop vs. Dynamic Goals

**Choice:** We use a pre-recorded perimeter loop (captured during the `"collect goal points"` phase).

**Why:**
- Simpler for the warehouse (aisles are well-defined)
- Faster: no online path planning per mission
- Repeatable: "patrol the perimeter N times" is unambiguous

**Real-world extension:** For dynamic goals (e.g., `"sweep aisle C then deliver to dock"`), the executor would query a world model, expand goals to intermediate waypoints, and pass to Nav2. Feasible; not in scope here.

### SLAM Toolbox vs. RTAB-Map

**Choice:** SLAM Toolbox (2D, lightweight).

**Why:**
- Warehouse is 2D (flat ground)
- Faster convergence; lower CPU footprint
- Sufficient for loop closure in 2D aisles

**Limitation:** No 3D reconstruction; dense mapping is later work.

### No Dynamic Formation Geometry

**Choice:** Squad robots follow independent waypoint segments, not a geometric formation.

**Why:**
- Warehouse aisles are narrow; tight formations cause collisions
- Per-waypoint deconfliction (split the route) is simpler and more robust
- Scaling: for open-field swarms, a formation layer (e.g., consensus-based heading) would be added

---

## 4. Scaling to Real-World Problems

### From Simulation to Hardware

1. **Simulator → Real Robot**
   - Replace Gazebo with real odometry, LiDAR, camera
   - Nav2 params (costmaps, planners) retrain on real sensor noise
   - The executor FSM, validator, and LLM interface are unchanged

2. **Single Warehouse → Multi-Site**
   - Current: one hardcoded map (`maps/map.yaml`)
   - Scaling: executor queries a config server (e.g., S3 or Kubernetes ConfigMap) for site-specific map + dock location
   - LLM prompt includes site context ("warehouse A floor 2")

3. **Squad Routing (Challenge 1) → Fleet Ops**
   - Current: split perimeter between 2 robots
   - Scaling: multi-depot VRP solver (e.g., OR-Tools) to assign zones to N robots, respecting battery constraints
   - Add task prioritization (urgent zones first)

4. **SLAM (Challenge 2) → Continuous Mapping**
   - Current: operator-initiated mapping, manual save
   - Scaling: background SLAM with loop-closure detection, hierarchical maps for multi-floor sites
   - Integrate with a central mapping server (ROS Bags, point clouds)

5. **Vision (Challenge 3) → Intelligent Following**
   - Current: follow closest detection, no target memory
   - Scaling: multi-target tracking (Kalman filters), re-identification across occlusions
   - Add intent recognition (is the person standing still or actively walking away?)

### Reliability & Safety

- **Monitor LLM latency**: if Bedrock is slow, fall back to a cached local model or manual teleoperation
- **Watchdog on Nav2**: if planner stalls, trigger retreat + return-to-dock
- **Geofencing**: validator rejects any waypoint outside a known safe perimeter
- **Heartbeat from operator**: long silences (>30s) with no new commands trigger auto-pause

### Multi-Site Heterogeneous Fleet

- Current executor: single-robot FSM, supports two namespaced robots
- Scaling: multi-agent middleware (e.g., ROS 2 Composer, swarmcore) to manage N robots of different types (wheels, drones, etc.)
- Unified LLM prompt language: `"robot1 patrol aisle A, robot2 inspect dock"` routes to respective executors

---

## 5. Testing & Validation

- **Unit tests** on mission validator (100+ test cases: invalid waypoints, out-of-range speeds, malformed JSON)
- **FSM tests** (state transitions, edge cases: dock not found, map load failure)
- **Integration tests** (prompt → JSON → validation → execution in a minimal ROS environment)
- **Reproducibility test** (same JSON run twice produces identical trajectory in Gazebo)

Run tests:
```bash
colcon test --packages-select core_task_controller
colcon test-result --verbose
```

---

## 6. Cited Sources & Licensing

All open-source dependencies and references are documented in [SOURCES.md](SOURCES.md). Key highlights:

- **Warehouse world, robot mesh**: AWS RoboMaker Small Warehouse (MIT-0 license)
- **Navigation**: ROS 2 Navigation2 (Apache-2.0)
- **SLAM**: SLAM Toolbox by Steve Macenski (LGPL-2.1)
- **Vision**: Ultralytics YOLOv8 (AGPL-3.0 — flagged for commercial deployments)
- **LLM service**: AWS Bedrock (no local code, user-provided credentials)

All original code (5 `core_task_*` packages) is Apache-2.0, declared in each `package.xml`.

---

## 7. What We'd Do With 3 More Weeks (Direct Offer Scope)

If targeting the direct offer (all three challenges + core task), the next priorities:

1. **Hybrid SLAM + long-term autonomy**: Background mapping with loop closure, hierarchical re-planning for multi-floor warehouses
2. **Real-time formation control**: Leader-follower geometry with collision avoidance, tested with 3+ robots
3. **Advanced vision**: Multi-target tracking, re-ID, intent estimation (human pose), distraction-free (ignore clutter)
4. **Hardware integration**: Port to a real TurtleBot3 + Jetson edge device, validate latency and reliability over 4+ hour mission
5. **Production hardening**: Kubernetes-based fleet orchestration, telemetry to a central dashboard, roll-out canaries

---

## Conclusion

Omokai demonstrates a clean, auditable pipeline from natural language to deterministic robot execution. The LLM proposes; a validator gates; an FSM executes. All three senior challenges are implemented and integrated: squads navigate together, SLAM maps the environment, and vision can detect and track people. The system runs in Docker, is reproducible on any Linux machine, and scales to multi-site, multi-robot real-world deployments with targeted hardening.

**How to run:**
```bash
./docker-run.sh                    # two robots, full features
./docker-run.sh squad:=false       # single robot (lighter)
```

**Example mission at the prompt:**
```
> patrol the warehouse perimeter twice
```

The robot localizes on the map, follows the loop twice, and returns to dock — all driven by a single English sentence, validated at each step, and auditable end to end.
