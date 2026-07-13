#!/usr/bin/env python3
# Nav2 stack (map_server + AMCL localization + planner/controller/BT) for the
# TurtleBot3 Waffle Pi in the AWS warehouse. Assumes the sim is already up
# (ros2 launch core_task_gazebo warehouse.launch.py).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('core_task_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml = LaunchConfiguration(
        'map', default=os.path.join(pkg_share, 'map', 'warehouse.yaml'))
    params_file = LaunchConfiguration(
        'params_file', default=os.path.join(pkg_share, 'param', 'nav2_params.yaml'))
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Auto-activate Nav2 lifecycle on launch'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'map', 'warehouse.yaml'),
            description='Occupancy map yaml for localization'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'param', 'nav2_params.yaml'),
            description='Nav2 parameters (waffle_pi tuned)'),
    ]

    # nav2_bringup runs both localization (map_server+amcl) and navigation.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items())

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    ld = LaunchDescription()
    for d in declare:
        ld.add_action(d)
    ld.add_action(nav2)
    ld.add_action(rviz)
    return ld
