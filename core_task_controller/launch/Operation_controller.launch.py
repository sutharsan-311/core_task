#!/usr/bin/env python3
"""Bring up the deterministic executor plus a resident (inactive) Nav2 stack.

The Gazebo sim is launched separately (core_task_gazebo warehouse.launch.py).
Nav2 comes up with autostart:=false so it sits inactive until the conductor
activates localization / navigation via lifecycle. slam_toolbox is NOT launched
here - the conductor starts it as a subprocess during mapping.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('core_task_navigation')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'autostart': 'false', 'use_sim_time': 'true'}.items())

    controller = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        output='screen',
        parameters=[{'use_sim_time': True}])

    collector = Node(
        package='core_task_controller',
        executable='goal_collector',
        name='goal_collector',
        output='screen',
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([nav2, controller, collector])
