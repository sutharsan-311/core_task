# core_task_perception

Vision for the detect-and-follow challenge: run YOLO on the robot camera, find a
user-chosen target class, and report where it is in the frame. Detection
(`target_detector.py`) and the vision-servo approach that turns a detection into
`/cmd_vel` (`person_approach.py`) are separate nodes; `core_task_controller`'s
`find_person` mission drives the search and hands off between them.

## Why a separate package

Perception is independent of the control stack. Keeping it out of
`core_task_controller` means the detector can be run, tested, or swapped (a
different model, a different camera) without touching the executor or the LLM
pipeline.

## Install

```bash
pip install -r requirements.txt
```

**`numpy` must stay < 2.** ROS Humble's `cv_bridge` is built against numpy 1.x
and fails to import under numpy 2 (`_ARRAY_API not found`). Installing
`ultralytics` can drag numpy 2 in, so the pin in `requirements.txt` matters —
if the detector dies on import, check `python3 -c "import numpy; print(numpy.__version__)"`
first. The YOLO weights (`yolo26n.pt`, ~5 MB) download automatically on first run.

## Run

```bash
# 1. sim (person_target stands wherever it's spawned unless target_mover is
#    also run - see core_task_gazebo/models/person_target/model.sdf)
ros2 launch core_task_gazebo warehouse.launch.py

# 2. the detector
ros2 run core_task_perception target_detector
#    a different target (any COCO class):
ros2 run core_task_perception target_detector --ros-args -p target_class:=chair

# 3. the approach node (only drives while enabled - see below)
ros2 run core_task_perception person_approach
```

The node logs the first acquisition:

```
target 'person' acquired (conf=0.73, cx=-0.09, area=0.102)
```

`person_approach` stays idle until something publishes `true` on
`/approach_enable` — normally `core_task_controller`'s `find_person` mission,
which spins/patrols to locate the person first, then hands off. To exercise it
standalone: `ros2 topic pub /approach_enable std_msgs/msg/Bool "{data: true}"`.

## Topics

| Topic | Type | Meaning |
|---|---|---|
| `/camera/image_raw` (in) | `sensor_msgs/Image` | Robot camera, from Gazebo |
| `/target_detection` (out) | `std_msgs/String` | JSON per frame (see below) |
| `/target_detection/image` (out) | `sensor_msgs/Image` | Annotated frame — the "picture to the operator" |
| `/approach_enable` (in, `person_approach`) | `std_msgs/Bool` | `true` to start driving toward the target, `false` to stop |
| `/target_detection` (in, `person_approach`) | `std_msgs/String` | Same detection stream, consumed for steering |
| `/cmd_vel` (out, `person_approach`) | `geometry_msgs/Twist` | Drive command while approaching |
| `/approach_done` (out, `person_approach`) | `std_msgs/Bool` | `true`, published once, when close enough to stop |

`/target_detection` payload:

```json
{"found": true, "label": "person", "conf": 0.73,
 "cx": -0.09, "cy": 0.02, "area": 0.102}
```

- `cx`, `cy` — target offset from image centre, `-1..+1`. `cx>0` is right of
  centre. This is the steering signal for the follow controller.
- `area` — box area / image area, `0..1`. Grows as the target nears; the
  distance signal.
- All of `cx`/`cy`/`area` are `0.0` when `found` is false.

## Parameters

`target_detector`:

| Param | Default | Notes |
|---|---|---|
| `target_class` | `person` | Any COCO class the model knows (person, chair, backpack, …) |
| `model` | `yolo26n.pt` | nano, NMS-free; fastest CPU inference of the current Ultralytics lineup |
| `conf` | `0.35` | Detection confidence threshold |
| `image_topic` | `/camera/image_raw` | Camera input |
| `rate_hz` | `10.0` | Inference rate; the latest frame is used, not a queue |
| `annotate` | `true` | Publish the annotated image |

`person_approach`:

| Param | Default | Notes |
|---|---|---|
| `arrived_area` | `0.35` | Detection box area (fraction of frame) that counts as "close enough" to stop. No depth sensor, so this is a proxy for distance — **needs a one-time field calibration**: walk the robot up to the target in Gazebo, read the reported `area`, tune this to match roughly the standoff you want. |
| `k_angular` | `1.5` | Proportional gain turning toward an off-center target |
| `linear_speed` | `0.15` | Constant forward crawl speed while approaching |
| `detection_topic` | `target_detection` | Where to read detection reports from |
| `cmd_vel_topic` | `cmd_vel` | Drive output; matches where Nav2 already publishes directly (the `cmd_vel_limiter` node exists but isn't wired into any launch file yet) |

## Design

`detection.py` and `approach.py` are pure (no rclpy, no torch): they turn raw
YOLO boxes into the normalised report above, and that report into a steering
command, and are unit-tested without a ROS graph or a model. `target_detector.py`
and `person_approach.py` are the nodes — camera/topics in, publish out. The
split keeps the geometry and control law testable and the heavy imports out of
the tests.

## Tests

```bash
cd src/core_task/core_task_perception
python3 -m pytest test/ -q
```

Covers the detection maths (offsets, area, class filtering, confidence
selection) and the approach control law (steering direction, arrival
threshold). End-to-end detection and approach against the sim is a manual
check — launch the world and watch for the acquisition/arrival logs.

## Sources

- **Ultralytics YOLO** (`ultralytics`, AGPL-3.0) — detection model and inference.
  https://github.com/ultralytics/ultralytics
- Approach follows the task's suggested reference `PX4-ROS2-Gazebo-YOLOv8`
  (camera → YOLO → target), adapted to a ground robot and the Nav2 stack.
- Walking target uses Gazebo Classic's stock `walk.dae` actor animation.
