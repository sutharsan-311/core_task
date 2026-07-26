#!/usr/bin/env python3
"""Squad launch: two operation controllers + two Nav2 stacks + the squad
coordinator, for the multi-agent contiguous half-split patrol.

Gazebo must already be up with both robots:
    ros2 launch core_task_gazebo warehouse.launch.py robot2:=true

Then:
    ros2 launch core_task_controller squad.launch.py

robot1 runs un-namespaced (its relative client names resolve globally); robot2
runs under /robot2 (the same relative names resolve to the /robot2 stack). Both
Nav2 stacks come up inactive (autostart:=false) so each controller drives its
own lifecycle. The coordinator fans a squad_navigation mission out to both.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('core_task_navigation')

    nav1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'autostart': 'false', 'use_sim_time': 'true'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'robot2_nav.launch.py')),
        launch_arguments={'autostart': 'false', 'use_sim_time': 'true'}.items())

    controller1 = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        output='screen',
        parameters=[{'use_sim_time': True}])

    controller2 = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        namespace='robot2',
        output='screen',
        parameters=[{'use_sim_time': True, 'dock': [0.0, 0.0, 0.0]}])

    coordinator = Node(
        package='core_task_controller',
        executable='squad_coordinator',
        name='squad_coordinator',
        output='screen')

    return LaunchDescription(
        [nav1, nav2, controller1, controller2, coordinator])
