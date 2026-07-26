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

## Manual Robot Control (Teleop)

Keyboard teleop is **enabled by default**. Use arrow keys anytime to manually drive the robot:

```bash
./run.sh                    # teleop on by default
./run.sh teleop:=false      # disable if you don't want it
```

Once running, use **arrow keys** to drive:
- `↑` → forward
- `↓` → backward
- `←` → turn left
- `→` → turn right
- `Ctrl+C` → stop

Useful for:
- Manually exploring during `"start building a map"` (SLAM mode)
- Quick testing without LLM
- Examiner-driven manual control

## Commands to issue

At the prompt, type a mission in plain English. Examples:

| You type | Mission |
|---|---|
| `patrol the warehouse perimeter twice` | navigate the loop 2× |
| `start building a map` | SLAM mapping |
| `collect goal points for warehouse` | capture perimeter waypoints |
| `both robots patrol, split the perimeter between them` | squad patrol |

A handful of words are handled locally and go straight to the executor, never
through the LLM:

| Word | Effect |
|---|---|
| `abort`, `stop`, `return to start`, `go home` | stop the patrol, drive back to dock |
| `done`, `save` | finish mapping or goal collection |
| `kill` | tear the whole stack down (runs `kill.sh`) |
| `/quit` | leave the prompt (also tears the stack down) |

## What to expect

- **Mapping** — Gazebo opens on the warehouse, the robot drives under SLAM as
  you send it around, and RViz shows the map filling in. Say `done` to save it.
- **Collect goals** — click points in RViz (`Publish Point` or `2D Goal Pose`);
  each one drops a green marker and is written to the perimeter file immediately.
  Say `done` to finish.
- **Patrol** — the robot localises, then follows the saved perimeter loop for the
  requested number of laps and returns to its dock. The waypoints show as green
  markers in RViz for the whole run.
- **Squad** — a second robot spawns, the perimeter splits front/back between the
  two, and both patrol their half. Each robot's frames, model, laser, and
  waypoints appear in the one RViz window.

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
