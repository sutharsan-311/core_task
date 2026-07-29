# nlm_interface

Turns a sentence into a validated mission JSON and publishes it on
`/submit_mission`. It is an optional front-end to `Operation_controller` — the
same topic takes hand-written JSON, so nothing here is in the control path.

```
/natural_language_mission  ->  [nlm_interface]  ->  /submit_mission  ->  [Operation_controller]
    "patrol twice"              Claude proposes       validated JSON        deterministic FSM
                                validator gates
                                     |
                                /nlm_feedback  (what happened, incl. rejections)
```

The LLM only ever proposes. `validate_and_explain()` (the same
`function.validate_mission()` the executor trusts) decides what gets published,
so a hallucinated or malformed mission is dropped here and reported on
`/nlm_feedback` rather than reaching the robot.

## Setup

```bash
pip install anthropic
export ANTHROPIC_API_KEY=sk-...   # node exits at startup without it
```

There is no rosdep key for `anthropic`, so it is not in `package.xml` — install
it yourself. Every other node in this package runs without it.

## Run

`bringup.launch.py` starts it by default — sim, Nav2, executor, and this node:

```bash
export ANTHROPIC_API_KEY=sk-...
ros2 launch core_task_controller bringup.launch.py
ros2 launch core_task_controller bringup.launch.py nlm:=false   # without a key
```

Or standalone, alongside an already-running `Operation_controller.launch.py`:

```bash
ros2 launch core_task_controller nlm_interface.launch.py
```

## Giving it orders

### The terminal client (easiest)

```bash
ros2 run core_task_controller nlm_cli
```

Full-screen: type a mission, watch the transcript and the phase, `/quit` to
exit. It publishes to `/natural_language_mission` and renders `/nlm_feedback`
and `/operation_feedback`, so one window replaces three. It never calls Claude
itself and needs no API key — `nlm_interface` owns that.

```
Type a mission in plain English. /quit to exit.
> patrol the warehouse perimeter twice
  ok       Mission accepted: mode=navigation, map=warehouse
  phase    start_navigation
> what is the capital of france
  rejected LLM failed to generate mission: Claude response did not
           include ```json code block
 phase: start_navigation | nlm ok | enter to send | /quit to exit
> _
```

The status bar shows `nlm NOT RUNNING` when nothing is listening, and the
client refuses to send rather than dropping the command silently.

### Aborting a patrol

Say **abort**, **return to start**, **go home**, **stop**, or `/abort`. The
robot cancels the loop it is on and drives back to the dock, then shuts nav
down the same way a finished mission does.

Abort does **not** go through Claude. It is matched in the client and calls the
executor's `/abort_mission` service directly, so it still works with a dead API
key or no network - a safety command should never wait on an LLM round trip.
It only interrupts an active patrol; in any other phase the executor replies
`nothing to abort in phase X` and does nothing.

By hand, without the CLI:

```bash
ros2 service call /abort_mission std_srvs/srv/Trigger
```

### By hand

From a **second terminal**, once `nlm_interface ready` appears in the log:

```bash
ros2 topic pub --once /natural_language_mission std_msgs/msg/String \
  '{data: "Patrol the warehouse perimeter twice"}'
```

`--once` matters: without it `topic pub` republishes at 1 Hz and you will
re-submit the mission every second.

Watch what happened in a third terminal:

```bash
ros2 topic echo /nlm_feedback        # what the LLM proposed, accepted or not
ros2 topic echo /operation_feedback  # what the robot is actually doing
```

Neither `/natural_language_mission` nor `/submit_mission` is latched, so a
command sent before the node is up goes nowhere. If nothing happens, check
`/nlm_feedback` first — or use `nlm_cli`, which warns you instead.

Commands map onto the modes in `function.VALID_MODES`:

| Say | Mission |
|---|---|
| "Patrol the perimeter 3 times" | `{"mode": "navigation", "map_name": "warehouse", "loops": 3}` |
| "Start building a map" | `{"mode": "mapping", "map_name": "warehouse"}` |
| "Capture the perimeter waypoints" | `{"mode": "collect_goals", "map_name": "warehouse"}` |
| "Find the person" | `{"mode": "find_person", "map_name": "warehouse"}` |

## Config

`config/nlm_interface.yaml`:

```yaml
nlm_interface:
  ros__parameters:
    model: "claude-haiku-4-5"
    temperature: 0.3
```

`temperature` (0.0-1.0) trades determinism for variety. Turning a sentence into
three fields wants stability, so keep it low.

There is no retry setting: the Anthropic SDK already retries 429/5xx/connection
errors twice on its own.

**The two knobs are model-dependent and mutually exclusive.** Haiku 4.5 takes
`temperature` and rejects `output_config.effort` with a 400 (`This model does not
support the effort parameter`). Newer Opus/Sonnet models are the reverse — they
reject `temperature` with a 400 and expect `effort`. Changing `model` here to an
Opus or Sonnet ID therefore also means changing `LlmClient` to send `effort`;
the model ID alone is not enough.

## Troubleshooting

Read `/nlm_feedback` first — every failure is reported there.

| Feedback says | Cause |
|---|---|
| `Mission validation failed: ...` | The LLM produced well-formed JSON that broke the schema (bad mode, `loops` < 1, missing `map_name`). The reason is the validator's own. |
| `LLM failed to generate mission: API error: 401` | Bad or missing API key. |
| `LLM failed to generate mission: API error: 404` | `model` in the yaml is retired or misspelled. |
| `... did not include \`\`\`json code block` | The model answered in prose. Usually a sign the command was too vague to be a mission. |

`RuntimeError: Missing ANTHROPIC_API_KEY` at startup means the env var did not
survive into the launch environment — `export` it in the shell you launch from.

An `export` in `~/.bashrc` only covers interactive terminals: Ubuntu's stock
`.bashrc` returns early for non-interactive shells, so anything above the export
never runs under systemd, cron, or CI. Put the key somewhere the service manager
reads if you ever run this headless.

The node calls the API from inside its subscription callback, so it is
unresponsive for a second or two per command. Commands sent during that window
queue rather than drop.

## Tests

No API key or ROS graph needed; the Claude client is mocked.

```bash
cd src/core_task/core_task_controller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_llm_client.py \
  test/test_mission_validator.py test/test_nlm_interface.py -q
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` is needed because ROS ships a `launch_testing`
pytest plugin that is incompatible with the installed pytest and breaks
collection before any test runs.
