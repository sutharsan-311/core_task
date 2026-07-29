# Sources & Citations

Everything this project builds on, with repo URL, license, and what was taken.
Per-package notes also live in each package's `README.md`; this is the
consolidated list.

## Our own code

All five `core_task_*` packages are original work written for this task
(Apache-2.0, declared in each `package.xml`): the LLM front-end and prompt CLI,
the mission validator, the deterministic FSM executor, the squad coordinator,
the TF bridge, the perception nodes, and all launch files. Where an idea came
from a reference, it's cited below.

## Simulation assets (vendored into the repo)

| Asset | Source | License | What we took |
|---|---|---|---|
| Warehouse world, shelves, clutter, ground, lamps (`core_task_gazebo/worlds`, `models/aws_robomaker_warehouse_*`, `maps`) | [aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world) | MIT-0 (MIT No Attribution; see `core_task_gazebo/LICENSE`) | The world SDF and all warehouse model meshes/configs, repackaged from catkin (ROS 1) into an ament package; content unchanged. |
| `walk.dae` mesh in `models/person_target` | Gazebo Classic built-in media (OSRF) | Apache-2.0 (Gazebo Classic) | The static human mesh, vendored so the repo is self-contained. (The person target has since been removed from the active world; the model dir remains.) |

## ROS 2 dependencies (installed via rosdep/apt, not vendored)

| Package | Repo | License | Used for |
|---|---|---|---|
| ROS 2 Humble core — `rclpy`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`, `tf2_ros`, `std_srvs` | https://github.com/ros2 | Apache-2.0 | Node framework, messages, TF, markers |
| Navigation2 — `nav2_bringup`, `nav2_amcl`, `nav2_map_server`, `nav2_msgs` | https://github.com/ros-navigation/navigation2 | Apache-2.0 (some BSD-3-Clause components) | Autonomous navigation, localization, the per-robot nav stacks |
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox | LGPL-2.1 | Online mapping (Challenge 2) |
| TurtleBot3 — `turtlebot3_gazebo` | https://github.com/ROBOTIS-GIT/turtlebot3_simulations | Apache-2.0 (ROBOTIS) | Robot model (Waffle Pi), `robot_state_publisher` / spawn launch includes |
| Gazebo Classic + `gazebo_ros_pkgs` (`gazebo_ros`, `gazebo_msgs`) | https://github.com/ros-simulation/gazebo_ros_pkgs | Apache-2.0 | Physics simulator, `set_entity_state`, spawn services |
| `cv_bridge` (vision_opencv) | https://github.com/ros-perception/vision_opencv | Apache-2.0 | ROS image ↔ OpenCV for the vision node |
| RViz2 | https://github.com/ros2/rviz | BSD-3-Clause | Visualization |

## Python dependencies (pip)

| Package | Repo | License | Used for |
|---|---|---|---|
| boto3 / botocore (AWS SDK) | https://github.com/boto/boto3 | Apache-2.0 | Calling AWS Bedrock (the LLM stage) |
| Ultralytics YOLO — `yolo26n.pt` + inference | https://github.com/ultralytics/ultralytics | **AGPL-3.0** | Target detection (Challenge 3) |
| NumPy (pinned `<2` for Humble `cv_bridge`) | https://github.com/numpy/numpy | BSD-3-Clause | Array math in the vision pipeline |

> **Note on Ultralytics / AGPL-3.0.** Ultralytics YOLO is copyleft (AGPL-3.0), not
> permissive like the rest of the stack. It's fine for this demo, but a
> production or closed-source deployment would need an Ultralytics commercial
> license or a permissively-licensed detector. Flagged here so it's a conscious
> choice, not a surprise.

## LLM service

The pipeline's LLM stage calls **AWS Bedrock** (default model
`us.amazon.nova-micro-v1:0`) via boto3 — a hosted service, not
open source. It authenticates with the operator's own AWS credentials. The
project originally targeted the Anthropic Claude API directly and was migrated
to Bedrock; either can drive the same validator/executor unchanged.

## Conceptual references (ideas, no code taken)

These are from the task sheet's own recommended list. We read them for approach;
no source was copied.

- **ChatDrones / ROSGPT-style** — https://github.com/Gaurang-1402/ChatDrones —
  the prompt → structured-JSON → ROS command pattern. Our LLM front-end follows
  this shape (LLM proposes JSON, a validator gates it, a deterministic executor
  acts); the implementation is our own.
- **PX4-ROS2-Gazebo-YOLOv8** — https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8
  — the camera → YOLO → follow-a-moving-target loop, adapted from a drone to a
  ground robot on the Nav2 stack.

## Verifying licenses

Licenses above reflect each project's repository at time of writing. For
redistribution, check the exact `LICENSE` in each upstream repo and the vendored
`core_task_gazebo/LICENSE`.
