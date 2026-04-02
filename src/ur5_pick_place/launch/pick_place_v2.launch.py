#!/usr/bin/env python3
"""Pick and Place v2.0 Launch File - RViz2 only (no Gazebo)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""

    return LaunchDescription([
        Node(
            package='ur5_pick_place',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
