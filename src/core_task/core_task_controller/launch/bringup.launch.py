#!/usr/bin/env python3
"""Full bringup: Gazebo warehouse sim + resident (inactive) Nav2 + the
Operation_controller state machine, in one launch.

Nav2 comes up with autostart:=false so the conductor owns activation via
lifecycle. slam_toolbox is started by the conductor as a subprocess during
mapping, so it is not launched here.

    ros2 launch core_task_controller bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('core_task_gazebo')
    nav_share = get_package_share_directory('core_task_navigation')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'warehouse.launch.py')))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'autostart': 'false'}.items())

    controller = Node(
        package='core_task_controller',
        executable='operation_controller',
        name='operation_controller',
        output='screen')

    # Give Gazebo a head start so Nav2 / the controller don't spam TF-lookup
    # failures before the robot has spawned.
    delayed = TimerAction(period=5.0, actions=[nav2, controller])

    return LaunchDescription([gazebo, delayed])
