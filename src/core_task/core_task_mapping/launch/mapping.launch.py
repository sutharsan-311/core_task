#!/usr/bin/env python3
# slam_toolbox (online async) for the TurtleBot3 Waffle Pi in the AWS warehouse.
# Assumes the sim is already up (ros2 launch core_task_gazebo warehouse.launch.py).
# Drive the robot around (e.g. teleop) to build the map, then save it:
#   ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('core_task_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'))

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                pkg_share, 'config', 'mapper_params_online_async.yaml'),
            description='slam_toolbox parameters (waffle_pi / warehouse tuned)'),
    ]

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}])

    ld = LaunchDescription()
    for d in declare:
        ld.add_action(d)
    ld.add_action(slam)
    return ld
