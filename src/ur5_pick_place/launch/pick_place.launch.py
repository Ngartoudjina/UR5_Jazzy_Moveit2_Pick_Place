"""
pick_place.launch.py
====================
Launches the complete UR5 pick & place pipeline:
  1. fake_object_pose  — publishes a simulated object location on /object_pose
  2. pick_place_node   — executes the enhanced pick & place sequence

Usage:
  ros2 launch ur5_pick_place pick_place.launch.py

Optional args:
  use_fake_pose:=true|false   (default true  — set false when using real camera)
  velocity_scale:=0.25        (arm speed 0.0–1.0)
  acceleration_scale:=0.25    (arm acceleration 0.0–1.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ── declare arguments ──────────────────────────────────────────────────
    use_fake_pose_arg = DeclareLaunchArgument(
        'use_fake_pose', default_value='true',
        description='Publish a simulated /object_pose (true) or rely on camera (false)')

    velocity_arg = DeclareLaunchArgument(
        'velocity_scale', default_value='0.25',
        description='MoveIt velocity scaling factor (0.0 – 1.0)')

    acceleration_arg = DeclareLaunchArgument(
        'acceleration_scale', default_value='0.25',
        description='MoveIt acceleration scaling factor (0.0 – 1.0)')

    # ── fake object pose publisher ─────────────────────────────────────────
    fake_pose_node = Node(
        package='ur5_pick_place',
        executable='fake_object_pose',
        name='fake_object_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_pose')),
    )

    # ── pick & place node (delayed 2 s so MoveIt is ready) ────────────────
    pick_place = Node(
        package='ur5_pick_place',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[{
            'velocity_scale':     LaunchConfiguration('velocity_scale'),
            'acceleration_scale': LaunchConfiguration('acceleration_scale'),
            'planning_time':      30.0,
            'planning_attempts':  30,
            'joint_tolerance':    0.05,
        }],
    )

    # Delay pick_place_node by 2 s to let fake_object_pose start publishing
    delayed_pick_place = TimerAction(period=2.0, actions=[pick_place])

    return LaunchDescription([
        use_fake_pose_arg,
        velocity_arg,
        acceleration_arg,
        fake_pose_node,
        delayed_pick_place,
    ])
