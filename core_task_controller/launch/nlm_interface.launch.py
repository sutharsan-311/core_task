#!/usr/bin/env python3
"""Launch the natural-language mission interface.

Optional front-end: it only publishes /submit_mission, which you can just as
well publish by hand. Run it alongside Operation_controller.launch.py.

Needs ANTHROPIC_API_KEY in the environment or the node exits at startup.

    ros2 launch core_task_controller nlm_interface.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start nlm_interface with config/nlm_interface.yaml."""
    share = get_package_share_directory('core_task_controller')
    config = os.path.join(share, 'config', 'nlm_interface.yaml')

    return LaunchDescription([
        Node(
            package='core_task_controller',
            executable='nlm_interface',
            name='nlm_interface',
            output='screen',
            parameters=[config],
        )
    ])
