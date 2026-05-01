"""
pick_place.launch.py
====================
Lance le pipeline complet UR5 pick & place :
  1. fake_object_pose  — publie une position simulée sur /object_pose
  2. pick_place_node   — exécute la séquence pick & place

Ce launch file NE lance PAS moveit.launch.py (ros2_control + move_group + RViz).
Lancer d'abord dans un terminal séparé :
  ros2 launch ur5_moveit moveit.launch.py
Attendre le message "Move group ready" avant de lancer ce fichier.

Usage :
  ros2 launch ur5_pick_place pick_place.launch.py
  ros2 launch ur5_pick_place pick_place.launch.py use_fake_pose:=false  # caméra réelle
  ros2 launch ur5_pick_place pick_place.launch.py velocity_scale:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    use_fake_pose_arg = DeclareLaunchArgument(
        'use_fake_pose', default_value='true',
        description='Publier /object_pose simulé (true) ou utiliser la caméra (false)')

    velocity_arg = DeclareLaunchArgument(
        'velocity_scale', default_value='0.15',
        description='Facteur de vitesse MoveIt2 (0.0 – 1.0). '
                    '0.15 recommandé pour les tests avec mock_components.')

    acceleration_arg = DeclareLaunchArgument(
        'acceleration_scale', default_value='0.10',
        description='Facteur d\'accélération MoveIt2 (0.0 – 1.0)')

    # ── fake_object_pose ──────────────────────────────────────────────────────
    # Publie /object_pose à 0.5 Hz (toutes les 2s) — pose statique, pas besoin de plus.
    # Position alignée avec scene_config.yaml : x=0.50, y=0.10, z=0.05
    fake_pose_node = Node(
        package='ur5_pick_place',
        executable='fake_object_pose',
        name='fake_object_pose',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_fake_pose')),
    )

    # ── pick_place_node ───────────────────────────────────────────────────────
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
            # Hauteur de tool0 (TCP gripper) au-dessus du centre de l'objet :
            #   pre_pick : 20 cm de marge avant la descente
            #   grasp    : 0  → TCP exactement au centre de l'objet (saisie)
            'pre_pick_height': 0.20,
            'grasp_height':    0.0,
        }],
    )

    # Délai 5s : laisse fake_object_pose publier au moins 2 messages avant que
    # pick_place_node démarre sa boucle d'attente de 10s.
    # Si moveit.launch.py n'est pas encore prêt, pick_place_node attendra sur
    # /move_action et /compute_ik — pas de race condition.
    delayed_pick_place = TimerAction(period=5.0, actions=[pick_place])

    return LaunchDescription([
        use_fake_pose_arg,
        velocity_arg,
        acceleration_arg,
        fake_pose_node,
        delayed_pick_place,
    ])
