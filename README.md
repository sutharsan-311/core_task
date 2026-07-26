# Omokai — Prompt-driven ground robot (ROS 2 + Gazebo + Nav2)

Type a mission in plain English, watch a robot carry it out in simulation. An LLM
turns your sentence into a structured mission, a validator checks it, and a
deterministic state machine drives Nav2. The LLM proposes; it never touches the
control loop.

```
prompt  →  LLM  →  validated mission JSON  →  deterministic executor  →  Gazebo/Nav2
(nlm_cli) (Any LLM)  (schema + sanity)        (FSM, Operation_controller)
          (Bedrock / Claude / OpenAI)
```

The same JSON always produces the same behaviour, so a run is reproducible and
auditable. If the model returns something malformed or unsafe, the validator
drops it before a wheel turns.

## What's in here

| Package | Role |
|---|---|
| `core_task_controller` | LLM front-end (`nlm_interface`), the `nlm_cli` prompt, the executor FSM (`Operation_controller`), goal collection, squad coordinator |
| `core_task_navigation` | Nav2 params, maps, RViz config, robot2's namespaced stack |
| `core_task_gazebo` | Warehouse world + robot spawning |
| `core_task_mapping` | SLAM (slam_toolbox) bring-up |
| `core_task_perception` | YOLO target detector + a moving target |

Three of the sheet's challenges are wired in: multi-agent squad patrol, SLAM
mapping, and vision detect-and-follow.

Every open-source repo, asset, and reference this builds on is cited in
[SOURCES.md](SOURCES.md) (repo URL, license, and what was used).

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble ([install](https://docs.ros.org/en/humble/Installation.html))
- Gazebo Classic 11 (`sudo apt install gazebo`)
- **One of:** AWS Bedrock account, Claude API key (Anthropic), or OpenAI API key
  (see [LLM setup](#llm-setup) — all three are supported, pick one)

## Quick start with Docker (recommended)

The image pins every version, so it runs the same on the examiner's machine as
on mine. Needs Docker + an X server on the host for the Gazebo/RViz GUI.

**Defaults:** Two robots (squad mode), keyboard teleop, and LLM front-end enabled.

```bash
docker build -t omokai:latest .            # ~ROS desktop-full + Nav2 + deps + build

# Set ONE of: AWS Bedrock, Claude API, or OpenAI credentials (see LLM Setup above)
export AWS_REGION=us-east-1
export AWS_BEARER_TOKEN_BEDROCK=...        # or AWS_ACCESS_KEY_ID + AWS_SECRET_ACCESS_KEY
# or:
export ANTHROPIC_API_KEY=...
# or:
export OPENAI_API_KEY=...

./docker-run.sh                            # DEFAULT: two-robot squad + teleop + LLM
./docker-run.sh squad:=false               # single robot only
./docker-run.sh squad:=false teleop:=false # single robot, no keyboard control
CMD=bash ./docker-run.sh                   # just a shell in the container
```

`docker-run.sh` wires up X11, passes your LLM credentials (or mounts `~/.aws`), and
adds `--gpus all` when you set `GPU=1`. Everything below (commands, what to
expect) is identical inside the container. For a bare-metal setup instead,
follow [Install](#install).

### Exact dependency / version list

| Layer | Pinned to |
|---|---|
| OS + ROS + sim | `osrf/ros:humble-desktop-full` (Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, RViz2) |
| Navigation | `ros-humble-navigation2`, `ros-humble-nav2-bringup` (Humble apt) |
| SLAM | `ros-humble-slam-toolbox` (Humble apt) |
| Robot | `ros-humble-turtlebot3-gazebo` (Humble apt), `TURTLEBOT3_MODEL=waffle_pi` |
| Vision bridge | `ros-humble-cv-bridge` (Humble apt) |
| Python | `boto3`, `numpy<2`, `ultralytics>=8.0` (see `core_task_perception/requirements.txt`) |

The `numpy<2` pin is load-bearing: Humble's `cv_bridge` is built against numpy
1.x and fails to import under numpy 2.

## Install

```bash
# 1. clone into a workspace and enter it
cd ~/omokai_ws

# 2. ROS dependencies
sudo apt update
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. Python dependencies (LLM client + vision)
pip install boto3 ultralytics

# 4. build and source
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`turtlebot3_gazebo`, `nav2_bringup`, and `slam_toolbox` come from apt via rosdep.
The launch files set `TURTLEBOT3_MODEL` and `GAZEBO_MODEL_PATH` themselves, so
you don't have to.

## LLM Setup (Pick One)

The system auto-detects which LLM provider to use based on environment variables.

### Option 1: AWS Bedrock (Default)
```bash
export AWS_REGION=us-east-1
export AWS_BEARER_TOKEN_BEDROCK=your_token     # or use AWS credentials
# or:
export AWS_ACCESS_KEY_ID=your_key
export AWS_SECRET_ACCESS_KEY=your_secret
```
Default model: `us.amazon.nova-micro-v1:0`. Override with `model:=us.amazon.nova-lite-v1:0`

### Option 2: Claude API (Anthropic)
```bash
export ANTHROPIC_API_KEY=sk-ant-...
```
Default model: `claude-3-5-sonnet-20241022`. Override with `model:=claude-3-opus-20250219`

### Option 3: OpenAI
```bash
export OPENAI_API_KEY=sk-proj-...
```
Default model: `gpt-4o-mini`. Override with `model:=gpt-4`

### No LLM? Run Executor Only
```bash
./run.sh nlm:=false  # drive the executor directly via ROS topics (no credentials needed)
```

## Run

One terminal does everything. `run.sh` brings up Gazebo + Nav2 + the executor +
the LLM front-end in the background, then hands this terminal to the mission
prompt.

```bash
./run.sh                 # two robots + squad coordinator (default)
./run.sh squad:=false    # single robot — lighter, faster to start
./run.sh nlm:=false      # no LLM; drive with `ros2 topic pub /submit_mission ...`
```

Give it a few seconds for Gazebo and Nav2 to settle. Stack logs go to
`src/core_task/logs/` (the path is printed at startup).

> The two-robot default is heavy (two full Nav2 stacks + RViz). On a modest
> laptop, prefer `squad:=false` while iterating.

## The NLM Prompt Interface

When you run `./run.sh`, the nlm_cli opens an interactive prompt:

```
╔════════════════════════════════════════════════════════════════╗
║                  MISSIONS · CONTROLS · STATUS                  ║
╠════════════════════════════════════════════════════════════════╣
║ MISSIONS:                                                      ║
║   patrol the perimeter twice in warehouse                      ║
║   start building a map called mapname                          ║
║   collect waypoints                                            ║
║   patrol by squad                                              ║
║                                                                ║
║ CONTROLS:  ↑ ↓ ← → to drive (arrow keys)                      ║
║            Ctrl+C to stop & reset                              ║
║                                                                ║
║ STATUS: idle, ready for mission                               ║
╚════════════════════════════════════════════════════════════════╝

Type a mission in plain English:
```

### At the Prompt

**Type your mission**, e.g.:
```
patrol the perimeter twice in warehouse
```

**What happens:**
1. Your text is sent to the LLM (Bedrock/Claude/OpenAI)
2. The LLM responds with JSON (or a rejection message like "Please specify a map name")
3. The JSON is validated
4. **Status updates** — one-line feedback showing the result:
   - ✓ Success: mission accepted, executor starting
   - ✗ Error: invalid mission, try again
5. **Gazebo/RViz** show the robot executing the mission

**Use arrow keys anytime** to manually drive during SLAM mapping (see Teleop below).

## Manual Robot Control (Teleop)

Keyboard teleop is **integrated into nlm_cli** and enabled by default:

```bash
./run.sh                    # teleop on by default (arrow keys work at prompt)
./run.sh teleop:=false      # disable if you don't want keyboard control
```

### Driving with Arrow Keys

While at the NLM prompt (anytime, even mid-mission):

- `↑` → forward
- `↓` → backward
- `←` → turn left
- `→` → turn right

The robot continues moving as long as you hold a key. Release the key to stop (200ms timeout).

### When to use Teleop

- **During SLAM mapping** — manually explore while the map builds
- **Testing** — drive without LLM involvement
- **Examiner demo** — manual control for interactive exploration
- **Map collection** — position the robot before clicking waypoints

## Commands to issue

At the prompt, type a mission in plain English. The LLM converts your command to structured JSON, a validator checks it, and then the executor runs it deterministically.

### Mission Commands (sent to LLM)

| Command Example | What it does | Notes |
|---|---|---|
| `patrol the perimeter twice in warehouse` | Navigate the saved perimeter loop 2 times and return to dock | Requires a map and perimeter file; loops can be 1, 2, 3, etc. |
| `patrol the perimeter 3 times` | Navigate the loop 3 times | Works for any number of loops |
| `start building a map called floor2` | Begin SLAM mapping under the name "floor2" | **Must specify a map name**; use arrow keys to manually drive the robot around |
| `start building a map` | **Rejected** — LLM requires explicit map name | Reply with: "Please specify a map name (e.g., 'start building a map called floor2')" |
| `collect waypoints` | Collect perimeter waypoints for the default "warehouse" map | Click points in RViz (`Publish Point` or `2D Goal Pose`); each point appears as a green marker |
| `capture the perimeter waypoints` | Same as above | Alternative phrasing |
| `both robots patrol the perimeter twice in warehouse` | Multi-agent mode: split the perimeter between two robots; each completes 2 loops on their half | Requires `squad:=true` (default); each robot gets a contiguous half |

**Key rule:** Commands for mapping and navigation **must include an explicit map name**. The LLM will reject vague commands like `"start mapping"` or `"patrol"` without a name.

### Local Commands (no LLM required)

These are intercepted by `nlm_cli` and sent directly to the executor:

| Command | Effect | When to use |
|---|---|---|
| `abort`, `stop`, `return to start`, `go home` | Stop the current patrol, drive back to dock immediately | Interrupt a running navigation mission |
| `done`, `save` | Finish mapping or waypoint collection; save the result | After you've manually driven the map or clicked enough waypoints |
| `kill` | Tear down the entire stack (Gazebo, Nav2, ROS, runs `kill.sh`) | Hard shutdown before a new run |
| `/quit` | Leave the prompt and shut down the stack | Graceful exit |

## How it works: command to execution

```
Your command (NLM terminal)
         ↓
    LLM provider (Bedrock / Claude / OpenAI)
    converts to JSON: {"mode": "navigation", "map_name": "warehouse", "loops": 2}
         ↓
    Validator checks schema + sanity
         ↓
    Operation_controller FSM executes the mission deterministically
         ↓
    Nav2 drives the robot(s); Gazebo/RViz show the result
```

**Key point:** The LLM is a one-shot converter, not a closed-loop controller. Once the JSON is validated, the executor takes over completely. The same JSON always produces the same behaviour.

## What to expect

### Mapping (`start building a map called mapname`)
- Gazebo opens on the warehouse scene
- The robot is placed at its spawn point
- You manually drive it with **arrow keys** to explore and build the map
- RViz displays the map filling in real-time via SLAM
- Say `done` to close SLAM and save the map file as `mapname_perimeter.yaml`
- The perimeter file starts empty (no waypoints yet)

### Collect Goals (`collect waypoints` or `capture the perimeter waypoints`)
- The robot localizes on an existing map
- RViz is ready to receive waypoint clicks
- Click in RViz using:
  - **2D Goal Pose** (top toolbar, drag to set position + heading)
  - **Publish Point** (top toolbar, click once for position; uses current robot heading)
- Each click drops a **green marker** and is written to `mapname_perimeter.yaml` immediately
- Say `done` to finish collection
- The perimeter file now contains the dock (starting pose) and all waypoint poses

### Patrol (`patrol the perimeter N times`)
- The robot localizes on the saved map
- It navigates to the first waypoint, then follows the entire perimeter loop N times
- After the last loop, it drives back to the dock and docks
- The waypoints display as green markers throughout the run
- Console shows progress: "Waypoint 1/5", "Loop 1/2 complete", etc.

### Squad Patrol (`both robots patrol the perimeter N times`)
- **robot1** and **robot2** spawn in Gazebo at their dock positions
- The perimeter is split: robot1 takes the front half, robot2 takes the back half
- Both robots run their half of the loop N times in parallel
- The squad_coordinator synchronizes them so they complete loops together
- All robot frames, models, lasers, and waypoints appear in one RViz window
- Both robots return to their docks after the final loop

## Vision detect-and-follow (Challenge 3)

```bash
ros2 run core_task_perception target_detector   # YOLO on the robot camera
ros2 run core_task_perception target_mover       # move a target through the scene
```

The detector publishes an annotated image and, on a configured target class,
drives the robot toward it.

## Tear down

Type `kill` at the prompt, or from any terminal:

```bash
./kill.sh
```

It stops Gazebo and every ROS 2 process and resets the ROS 2 daemon, so the next
run starts from a clean slate.

## Tests

```bash
colcon test --packages-select core_task_controller
colcon test-result --verbose
```

The executor's FSM, mission validator, LLM client, waypoint markers, and the
nlm_interface routing all have unit tests that run without a ROS graph.
