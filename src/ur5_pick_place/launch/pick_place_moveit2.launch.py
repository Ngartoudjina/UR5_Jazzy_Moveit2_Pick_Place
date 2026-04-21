#!/usr/bin/env python3
"""
Launch file pour tester pick_place_moveit2 avec MoveIt2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package paths
    ur5_moveit_share = get_package_share_directory('ur5_moveit')
    ur5_pick_place_share = get_package_share_directory('ur5_pick_place')
    
    return LaunchDescription([
        # ============== ÉTAPE 1: Lancer MoveIt2 (simulator + RViz) ==============
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ur5_moveit_share, 'launch', 'moveit.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'db': 'false',
            }.items()
        ),
        
        # ============== ÉTAPE 2: Lancer add_object (ajoute table + objet) ==============
        Node(
            package='ur5_pick_place',
            executable='add_object',
            name='add_object_node',
            output='screen',
        ),
        
        # ============== ÉTAPE 3: Lancer pick_place_moveit2 (notre nouveau code) ==============
        Node(
            package='ur5_pick_place',
            executable='pick_place_moveit2',
            name='pick_place_moveit2_node',
            output='screen',
        ),
    ])
