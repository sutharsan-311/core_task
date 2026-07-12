# core_task_gazebo

Gazebo simulation assets for the Omokai core task — the facility the robot patrols.

- **World:** AWS RoboMaker small warehouse (`no_roof_small_warehouse.world`)
- **Robot:** TurtleBot3 Waffle Pi (camera + 360° LIDAR)

## Run

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch core_task_gazebo warehouse.launch.py
```

## Sources / attribution

- `worlds/`, `models/`, `maps/`, `rviz/` derived from
  **aws-robomaker-small-warehouse-world**
  (https://github.com/aws-robotics/aws-robomaker-small-warehouse-world),
  Apache 2.0 — see [LICENSE](LICENSE). Repackaged from catkin (ROS 1) into an
  ament_cmake (ROS 2) package; content unchanged.
- Robot model and `robot_state_publisher` / `spawn_turtlebot3` launch includes
  from `turtlebot3_gazebo` (ROBOTIS, Apache 2.0).
