#!/usr/bin/env python3
"""Full bringup: Gazebo warehouse sim + resident (inactive) Nav2 + the
Operation_controller state machine + the natural-language interface, plus the
optional second robot and squad coordinator, in one launch.

Nav2 comes up with autostart:=false so the conductor owns activation via
lifecycle. slam_toolbox is started by the conductor as a subprocess during
mapping, so it is not launched here.

    ros2 launch core_task_controller bringup.launch.py               # two robots (squad)
    ros2 launch core_task_controller bringup.launch.py squad:=false  # single robot
    ros2 launch core_task_controller bringup.launch.py nlm:=false    # no LLM front-end

nlm_interface needs ANTHROPIC_API_KEY and exits without it, so it is behind
`nlm:=` to keep bringup usable for anyone who only wants to drive the robot.

`squad:=` (default true) is the single switch for the whole multi-agent layer:
it spawns robot2 in the sim, brings up robot2's /robot2 Nav2 stack (inactive,
sharing robot1's map) and its controller, and starts the squad_coordinator that
fans a squad_navigation mission out to both robots. Pass squad:=false for a clean
single-robot sim with no robot2.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('core_task_gazebo')
    nav_share = get_package_share_directory('core_task_navigation')
    ctrl_share = get_package_share_directory('core_task_controller')

    nlm_arg = DeclareLaunchArgument(
        'nlm', default_value='true',
        description='Start nlm_interface (needs ANTHROPIC_API_KEY).')

    squad_arg = DeclareLaunchArgument(
        'squad', default_value='true',
        description='Bring up robot2 (nav + controller) and the squad '
                    'coordinator for multi-agent squad_navigation. On by '
                    'default: two robots unless squad:=false.')
    squad = LaunchConfiguration('squad')

    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='true',
        description='Enable keyboard teleop (teleop_twist_keyboard) for '
                    'manual robot control. On by default; disable with teleop:=false.')

    # robot2 spawns only when squad is on, so squad:=false gives a clean
    # single-robot sim with no orphaned second robot.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'warehouse.launch.py')),
        launch_arguments={'robot2': squad}.items())

    # Gazebo's gzserver.launch.py declares a `params_file` arg defaulting to '',
    # which collides with Nav2's `params_file` and leaks the empty value into
    # Nav2 (RewrittenYaml then opens ''). Pass Nav2's params/map explicitly so
    # the correct files win regardless of the collision.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'autostart': 'false',
            'use_sim_time': 'true',
            'params_file': os.path.join(nav_share, 'param', 'nav2_params.yaml'),
            'map': os.path.join(nav_share, 'map', 'warehouse.yaml'),
        }.items())

    controller = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Standalone perimeter capture; idle unless /operation_feedback says
    # goalpoint_collection.
    collector = Node(
        package='core_task_controller',
        executable='goal_collector',
        name='goal_collector',
        output='screen',
        parameters=[{'use_sim_time': True}])

    # Language front-end. /submit_mission is not latched, so it has to be up
    # before you send a command - it comes up with the controller, not earlier.
    nlm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ctrl_share, 'launch', 'nlm_interface.launch.py')),
        condition=IfCondition(LaunchConfiguration('nlm')))

    # ---- multi-agent layer (squad:=true) --------------------------------
    # robot2's Nav2 stack under /robot2, inactive like robot1's so its own
    # controller owns lifecycle. Shares robot1's warehouse map.
    nav2_r2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'robot2_nav.launch.py')),
        launch_arguments={'autostart': 'false', 'use_sim_time': 'true'}.items(),
        condition=IfCondition(squad))

    # robot2's controller: same executable under /robot2 (its relative client
    # names resolve to the /robot2 stack), docking at robot2's spawn pose.
    # dock is robot2's true map-frame pose (Gazebo (0,0,0) through robot1's
    # world->map calibration); the controller publishes it as /robot2/initialpose,
    # so it MUST match nav2_params_robot2.yaml's amcl initial_pose or it relocates
    # robot2 to the origin and every plan fails on phantom obstacles.
    controller_r2 = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True, 'dock': [2.427, -3.04, 1.568]}],
        condition=IfCondition(squad))

    # Expands one squad_navigation mission into a navigation mission per robot.
    coordinator = Node(
        package='core_task_controller',
        executable='squad_coordinator',
        name='squad_coordinator',
        output='screen',
        condition=IfCondition(squad))

    # robot2's TF lives on /robot2/tf(_static); mirror it onto the global /tf so
    # the single RViz can show robot2's model, laser and frames.
    tf_bridge = Node(
        package='core_task_controller',
        executable='tf_bridge',
        name='robot2_tf_bridge',
        output='screen',
        condition=IfCondition(squad))

    # Keyboard teleop is now integrated into nlm_cli (arrow keys work directly
    # in the mission prompt). The teleop:= argument is kept for backward compatibility
    # but teleop_twist_keyboard node is no longer launched separately.

    # Give Gazebo a head start so Nav2 / the controllers don't spam TF-lookup
    # failures before the robots have spawned.
    delayed = TimerAction(
        period=5.0,
        actions=[nav2, controller, collector, nlm,
                 nav2_r2, controller_r2, coordinator, tf_bridge])

    return LaunchDescription([nlm_arg, squad_arg, teleop_arg, gazebo, delayed])
