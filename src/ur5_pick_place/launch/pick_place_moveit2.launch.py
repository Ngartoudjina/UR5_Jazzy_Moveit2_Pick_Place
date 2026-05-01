#!/usr/bin/env python3
"""
Launch file pick & place UR5 — ROS2 Humble
Utilise async_pick_place_node (remplace pick_place_moveit2.py supprimé)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ur5_moveit_share = get_package_share_directory('ur5_moveit')

    # ÉTAPE 1 : MoveIt2 + ros2_control + RViz
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5_moveit_share, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'db': 'false',
        }.items()
    )

    # ÉTAPE 2 : Ajouter table + objet à la scène MoveIt2
    # 8s : ros2_control (3s) + MoveGroup (5s) + marge
    add_object_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='ur5_pick_place',
            executable='add_object',
            name='add_object_node',
            output='screen',
        )]
    )

    # ÉTAPE 3 : Pick & place — pointe vers async_pick_place_node
    # 13s : add_object a besoin d'1s pour appliquer la scène
    pick_place_node = TimerAction(
        period=13.0,
        actions=[Node(
            package='ur5_pick_place',
            executable='pick_place_node',   # ← CORRIGÉ : async_pick_place_node
            name='pick_place_node',
            output='screen',
            parameters=[{
                'velocity_scale':      0.45,
                'acceleration_scale':  0.35,
                'planning_time':       10.0,
                'planning_attempts':   5,
                # tool0 (TCP) Z au-dessus du centre objet ; voir pick_place_node.py
                'pre_pick_height':     0.20,
                'grasp_height':        0.0,
            }]
        )]
    )

    return LaunchDescription([
        moveit_launch,
        add_object_node,
        pick_place_node,
    ])
