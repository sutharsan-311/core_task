# core_task_perception

Vision for the detect-and-follow challenge: run YOLO on the robot camera, find a
user-chosen target class, and report where it is in the frame. Detection only —
the follow controller (turning a detection into `/cmd_vel`) is a separate node.

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
first. The YOLO weights (`yolov8n.pt`, ~6 MB) download automatically on first run.

## Run

```bash
# 1. sim with the walking-person target (added to the warehouse world)
ros2 launch core_task_gazebo warehouse.launch.py

# 2. the detector
ros2 run core_task_perception target_detector
#    a different target (any COCO class):
ros2 run core_task_perception target_detector --ros-args -p target_class:=chair
```

The robot faces a walking `person` actor on spawn, so detections start without
driving. The node logs the first acquisition:

```
target 'person' acquired (conf=0.73, cx=-0.09, area=0.102)
```

## Topics

| Topic | Type | Meaning |
|---|---|---|
| `/camera/image_raw` (in) | `sensor_msgs/Image` | Robot camera, from Gazebo |
| `/target_detection` (out) | `std_msgs/String` | JSON per frame (see below) |
| `/target_detection/image` (out) | `sensor_msgs/Image` | Annotated frame — the "picture to the operator" |

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

| Param | Default | Notes |
|---|---|---|
| `target_class` | `person` | Any COCO class the model knows (person, chair, backpack, …) |
| `model` | `yolov8n.pt` | nano; fast on CPU, faster on GPU |
| `conf` | `0.35` | Detection confidence threshold |
| `image_topic` | `/camera/image_raw` | Camera input |
| `rate_hz` | `10.0` | Inference rate; the latest frame is used, not a queue |
| `annotate` | `true` | Publish the annotated image |

## Design

`detection.py` is pure (no rclpy, no torch): it turns raw YOLO boxes into the
normalised report above and is unit-tested without a ROS graph or a model.
`target_detector.py` is the node — camera in, model, publish. The split keeps
the geometry testable and the heavy imports out of the tests.

## Tests

```bash
cd src/core_task/core_task_perception
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

Covers the detection maths (offsets, area, class filtering, confidence
selection). End-to-end detection against the sim is a manual check — launch the
world and watch for the acquisition log.

## Sources

- **Ultralytics YOLO** (`ultralytics`, AGPL-3.0) — detection model and inference.
  https://github.com/ultralytics/ultralytics
- Approach follows the task's suggested reference `PX4-ROS2-Gazebo-YOLOv8`
  (camera → YOLO → target), adapted to a ground robot and the Nav2 stack.
- Walking target uses Gazebo Classic's stock `walk.dae` actor animation.
