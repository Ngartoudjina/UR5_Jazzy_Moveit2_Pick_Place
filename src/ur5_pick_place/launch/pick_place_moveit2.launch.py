#!/usr/bin/env python3
"""
Launch file pour pick_place_moveit2 avec MoveIt2 — ROS2 Humble.

CORRECTIONS :
  - use_sim_time: false (était 'true' — incorrect sans Gazebo, aucune horloge sim publiée)
  - add_object lancé avec délai pour attendre MoveIt2 ready (était immédiat)
  - pick_place lancé après add_object (scène doit être initialisée avant)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ur5_moveit_share = get_package_share_directory('ur5_moveit')

    # ÉTAPE 1 : MoveIt2 + ros2_control + RViz
    # use_sim_time=false : pas de Gazebo → pas d'horloge simulation publiée
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur5_moveit_share, 'launch', 'moveit.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',   # CORRIGÉ : était 'true' — erroné sans Gazebo
            'db': 'false',
        }.items()
    )

    # ÉTAPE 2 : Ajouter table + objet à la scène MoveIt2
    # Délai 8s : attend MoveGroup (5s) + marge pour /apply_planning_scene ready
    add_object_node = TimerAction(
        period=8.0,
        actions=[Node(
            package='ur5_pick_place',
            executable='add_object',
            name='add_object_node',
            output='screen',
        )]
    )

    # ÉTAPE 3 : Lancer le pick & place
    # Délai 12s : attend que add_object ait confirmé la scène
    pick_place_node = TimerAction(
        period=12.0,
        actions=[Node(
            package='ur5_pick_place',
            executable='pick_place_moveit2',
            name='pick_place_moveit2_node',
            output='screen',
        )]
    )

    return LaunchDescription([
        moveit_launch,
        add_object_node,
        pick_place_node,
    ])
