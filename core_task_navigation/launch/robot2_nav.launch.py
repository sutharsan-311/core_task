#!/usr/bin/env python3
"""Namespaced Nav2 stack for the second robot (multi-agent challenge).

Brings up a full Nav2 (map_server + AMCL + planner/controller/BT) under the
/robot2 namespace, sharing the same warehouse map as robot 1. All frames are
prefixed robot2/ via nav2_params_robot2.yaml; nav2_bringup's use_namespace puts
the nodes and their relative topics (scan, cmd_vel, odom) under /robot2.

Robot 1 keeps its own un-namespaced stack (core_task_navigation
navigation.launch.py) - the two do not overlap. Sim + robot 2 must already be
up: ros2 launch core_task_gazebo warehouse.launch.py robot2:=true.

    ros2 launch core_task_navigation robot2_nav.launch.py
    ros2 topic pub --once /robot2/goal_pose geometry_msgs/msg/PoseStamped ...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('core_task_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml = os.path.join(pkg_share, 'map', 'warehouse.yaml')
    params_file = os.path.join(
        pkg_share, 'param', 'nav2_params_robot2.yaml')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Auto-activate the robot2 Nav2 lifecycle on launch'),
    ]

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': 'robot2',
            'use_namespace': 'true',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items())

    ld = LaunchDescription()
    for d in declare:
        ld.add_action(d)
    ld.add_action(nav2)
    return ld
