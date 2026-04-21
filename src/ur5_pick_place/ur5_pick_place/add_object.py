#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.srv import ApplyPlanningScene
import yaml
import os


class AddObject(Node):
    def __init__(self):
        super().__init__('add_object')
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info('Waiting for ApplyPlanningScene service...')
        self.client.wait_for_service()
        self.get_logger().info('Connected ✅')
        
        # Load configuration
        self.config = self._load_config()
    
    def _load_config(self):
        """Charger la config pour obtenir positions de table et objet"""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'scene_config.yaml'),
            '/ros2_ws/build/ur5_pick_place/ur5_pick_place/scene_config.yaml',
            '/ros2_ws/install/ur5_pick_place/lib/python3.10/site-packages/ur5_pick_place/scene_config.yaml',
            os.path.expandvars('~/ros2_humble_ws/src/ur5_pick_place/ur5_pick_place/scene_config.yaml'),
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                with open(path) as f:
                    return yaml.safe_load(f)
        
        # Config par défaut si fichier non trouvé
        return {
            'scene': {
                'table': {
                    'position': {'x': 0.5, 'y': 0.0, 'z': 0.0},
                    'size': {'x': 0.8, 'y': 0.8, 'z': 0.05}
                },
                'pick_object': {
                    'position': {'x': 0.5, 'y': 0.1, 'z': 0.05},
                    'size': {'x': 0.05, 'y': 0.05, 'z': 0.10}
                }
            }
        }

    def add_scene_objects(self):
        """Ajouter TABLE et OBJET à la scène MoveIt2"""
        scene = self.config.get('scene', {})
        
        # ═════════════════════════════════════════════════════════════
        # 1. AJOUTER LA TABLE
        # ═════════════════════════════════════════════════════════════
        table_cfg = scene.get('table', {})
        if table_cfg:
            table = CollisionObject()
            table.id = 'table'
            table.header.frame_id = 'world'
            
            table_size = table_cfg.get('size', {})
            table_pos = table_cfg.get('position', {})
            
            primitive_table = SolidPrimitive()
            primitive_table.type = SolidPrimitive.BOX
            primitive_table.dimensions = [
                table_size.get('x', 0.8),
                table_size.get('y', 0.8),
                table_size.get('z', 0.05)
            ]
            
            pose_table = Pose()
            pose_table.position.x = table_pos.get('x', 0.5)
            pose_table.position.y = table_pos.get('y', 0.0)
            pose_table.position.z = table_pos.get('z', 0.0)
            pose_table.orientation.w = 1.0
            
            table.primitives.append(primitive_table)
            table.primitive_poses.append(pose_table)
            table.operation = CollisionObject.ADD
            
            req = ApplyPlanningScene.Request()
            req.scene.world.collision_objects.append(table)
            req.scene.is_diff = True
            
            self.client.call_async(req)
            self.get_logger().info(
                f'📋 TABLE added at ({pose_table.position.x:.2f}, {pose_table.position.y:.2f}, {pose_table.position.z:.2f})'
            )
        
        # ═════════════════════════════════════════════════════════════
        # 2. AJOUTER L'OBJET
        # ═════════════════════════════════════════════════════════════
        obj_cfg = scene.get('pick_object', {})
        if obj_cfg:
            box = CollisionObject()
            box.id = 'pick_object'
            box.header.frame_id = 'world'
            
            obj_size = obj_cfg.get('size', {})
            obj_pos = obj_cfg.get('position', {})
            
            primitive_box = SolidPrimitive()
            primitive_box.type = SolidPrimitive.BOX
            primitive_box.dimensions = [
                obj_size.get('x', 0.05),
                obj_size.get('y', 0.05),
                obj_size.get('z', 0.10)
            ]
            
            pose_box = Pose()
            pose_box.position.x = obj_pos.get('x', 0.5)
            pose_box.position.y = obj_pos.get('y', 0.1)
            pose_box.position.z = obj_pos.get('z', 0.05)
            pose_box.orientation.w = 1.0
            
            box.primitives.append(primitive_box)
            box.primitive_poses.append(pose_box)
            box.operation = CollisionObject.ADD
            
            req = ApplyPlanningScene.Request()
            req.scene.world.collision_objects.append(box)
            req.scene.is_diff = True
            
            self.client.call_async(req)
            self.get_logger().info(
                f'📦 OBJECT added at ({pose_box.position.x:.2f}, {pose_box.position.y:.2f}, {pose_box.position.z:.2f})'
            )


def main():
    rclpy.init()
    node = AddObject()
    node.add_scene_objects()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
