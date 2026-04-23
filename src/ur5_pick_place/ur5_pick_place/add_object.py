#!/usr/bin/env python3
"""
add_object.py — Ajoute table et objet à la scène de planification MoveIt2.

CORRECTIONS APPLIQUÉES :
  - call_async() + spin_until_future_complete() : attend la confirmation réelle
    (l'ancien code faisait call_async() fire-and-forget + spin_once(1s) risquant
     que MoveIt2 planifie avant que la table soit dans la scène)
  - Les deux objets sont ajoutés séquentiellement avec confirmation entre chaque
    pour éviter la race condition sur l'état de la scène (is_diff)
  - Chemins portables : plus de ~/ros2_humble_ws/ hardcodé
  - Le nœud ne se détruit qu'après confirmation des deux requêtes
"""

import os
import yaml

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

SCENE_SERVICE_TIMEOUT = 10.0  # secondes


class AddObject(Node):
    """Nœud one-shot pour initialiser la scène MoveIt2 avec table + objet."""

    def __init__(self):
        super().__init__('add_object')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Attente du service ApplyPlanningScene…')
        self.client.wait_for_service()
        self.get_logger().info('Service disponible ✅')
        self.config = self._load_config()

    def _load_config(self) -> dict:
        """Charge scene_config.yaml depuis des chemins portables (sans ros2_humble_ws)."""
        search_paths = [
            os.path.join(os.path.dirname(__file__), 'scene_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'config', 'scene_config.yaml'),
            os.path.join(
                os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0],
                'share', 'ur5_pick_place', 'scene_config.yaml'
            ),
        ]
        for path in search_paths:
            path = os.path.normpath(path)
            if os.path.exists(path):
                self.get_logger().info(f'Config chargée depuis : {path}')
                with open(path) as f:
                    return yaml.safe_load(f)

        self.get_logger().warn('scene_config.yaml introuvable — valeurs par défaut utilisées')
        return {
            'scene': {
                'table':       {'position': {'x': 0.65, 'y': 0.0,  'z': -0.025},
                                'size':     {'x': 0.8,  'y': 0.8,  'z': 0.05}},
                'pick_object': {'position': {'x': 0.5,  'y': 0.1,  'z': 0.05},
                                'size':     {'x': 0.05, 'y': 0.05, 'z': 0.10}},
            }
        }

    def add_scene_objects(self) -> bool:
        """Ajoute table puis objet avec confirmation après chaque ajout."""
        scene_cfg = self.config.get('scene', {})

        # 1. TABLE
        if not self._add_collision_object(
            'table',
            scene_cfg.get('table', {}).get('position', {'x': 0.65, 'y': 0.0, 'z': -0.025}),
            scene_cfg.get('table', {}).get('size',     {'x': 0.8,  'y': 0.8, 'z': 0.05}),
        ):
            self.get_logger().error('❌ Echec ajout table')
            return False

        # 2. OBJET
        if not self._add_collision_object(
            'pick_object',
            scene_cfg.get('pick_object', {}).get('position', {'x': 0.5, 'y': 0.1, 'z': 0.05}),
            scene_cfg.get('pick_object', {}).get('size',     {'x': 0.05,'y': 0.05,'z': 0.10}),
        ):
            self.get_logger().error('❌ Echec ajout objet')
            return False

        return True

    def _add_collision_object(self, obj_id: str, position: dict, size: dict) -> bool:
        """
        Envoie un objet de collision et ATTEND la confirmation.

        CORRECTION : remplace l'ancien call_async() sans await qui laissait
        MoveIt2 planifier sans garantie que la scène était à jour.
        """
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = 'world'
        co.header.stamp = self.get_clock().now().to_msg()

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(size.get('x', 0.1)),
                           float(size.get('y', 0.1)),
                           float(size.get('z', 0.1))]

        pose = Pose()
        pose.position.x = float(position.get('x', 0.0))
        pose.position.y = float(position.get('y', 0.0))
        pose.position.z = float(position.get('z', 0.0))
        pose.orientation.w = 1.0

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene.world.collision_objects.append(co)
        req.scene.is_diff = True

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SCENE_SERVICE_TIMEOUT)

        if not future.done():
            self.get_logger().error(f'⏱️ Timeout ajout "{obj_id}"')
            return False

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f'ApplyPlanningScene échoué pour "{obj_id}"')
            return False

        self.get_logger().info(
            f'✅ "{obj_id}" @ ({position.get("x",0):.2f},'
            f' {position.get("y",0):.2f}, {position.get("z",0):.2f})'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = AddObject()
    try:
        success = node.add_scene_objects()
        node.get_logger().info(
            '🎉 Scène initialisée' if success else '❌ Initialisation incomplète'
        )
    except Exception as e:
        node.get_logger().error(f'Erreur : {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
