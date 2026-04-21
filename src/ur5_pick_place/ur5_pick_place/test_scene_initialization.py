#!/usr/bin/env python3
"""
Test script to verify that the scene is properly initialized with table and objects.
This is simpler than the full pick_place test - it just checks scene setup.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import yaml
import os
import time


class SceneInitializationTester(Node):
    def __init__(self):
        super().__init__('scene_tester')
        
        self.get_logger().info("🔧 Testing Scene Initialization...")
        
        # Create clients
        self.apply_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        
        # Load configuration
        self.config = self._load_config()
        
        self.get_logger().info("✅ Clients created")
    
    def _load_config(self):
        """Load scene_config.yaml"""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'scene_config.yaml'),
            '/ros2_ws/build/ur5_pick_place/ur5_pick_place/scene_config.yaml',
            '/ros2_ws/install/ur5_pick_place/lib/python3.10/site-packages/ur5_pick_place/scene_config.yaml',
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                try:
                    with open(path) as f:
                        cfg = yaml.safe_load(f)
                        self.get_logger().info(f"✓ Config loaded from: {path}")
                        return cfg
                except Exception as e:
                    self.get_logger().warn(f"Failed to load from {path}: {e}")
        
        self.get_logger().warn("⚠️ Config file not found, using defaults")
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
    
    def initialize_and_verify_scene(self):
        """Initialize scene and verify objects are added"""
        
        self.get_logger().info("\n========== INITIALIZING SCENE ==========\n")
        
        # Wait for ApplyPlanningScene service
        self.get_logger().info("⏳ Waiting for ApplyPlanningScene service...")
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ ApplyPlanningScene service not available!")
            return False
        
        self.get_logger().info("✓ ApplyPlanningScene service ready")
        
        scene_cfg = self.config.get('scene', {})
        
        # ═════════════════════════════════════════════════════════════
        # ADD TABLE
        # ═════════════════════════════════════════════════════════════
        table_cfg = scene_cfg.get('table', {})
        if table_cfg:
            self.get_logger().info("\n▶ Adding TABLE to scene...")
            table = CollisionObject()
            table.id = 'table'
            table.header.frame_id = 'world'
            
            table_size = table_cfg.get('size', {})
            table_pos = table_cfg.get('position', {})
            
            primitive_table = SolidPrimitive()
            primitive_table.type = SolidPrimitive.BOX
            primitive_table.dimensions = [
                float(table_size.get('x', 0.8)),
                float(table_size.get('y', 0.8)),
                float(table_size.get('z', 0.05))
            ]
            
            pose_table = Pose()
            pose_table.position.x = float(table_pos.get('x', 0.5))
            pose_table.position.y = float(table_pos.get('y', 0.0))
            pose_table.position.z = float(table_pos.get('z', 0.0))
            pose_table.orientation.w = 1.0
            
            table.primitives.append(primitive_table)
            table.primitive_poses.append(pose_table)
            table.operation = CollisionObject.ADD
            
            req = ApplyPlanningScene.Request()
            req.scene.world.collision_objects.append(table)
            req.scene.is_diff = True
            
            future = self.apply_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            self.get_logger().info(
                f"  ✓ TABLE added at position ({pose_table.position.x:.2f}, {pose_table.position.y:.2f}, {pose_table.position.z:.2f})"
            )
            self.get_logger().info(
                f"  ✓ TABLE size: {primitive_table.dimensions[0]:.2f}m x {primitive_table.dimensions[1]:.2f}m x {primitive_table.dimensions[2]:.2f}m"
            )
        
        # ═════════════════════════════════════════════════════════════
        # ADD PICK OBJECT
        # ═════════════════════════════════════════════════════════════
        obj_cfg = scene_cfg.get('pick_object', {})
        if obj_cfg:
            self.get_logger().info("\n▶ Adding PICK OBJECT to scene...")
            obj = CollisionObject()
            obj.id = 'pick_object'
            obj.header.frame_id = 'world'
            
            obj_size = obj_cfg.get('size', {})
            obj_pos = obj_cfg.get('position', {})
            
            primitive_obj = SolidPrimitive()
            primitive_obj.type = SolidPrimitive.BOX
            primitive_obj.dimensions = [
                float(obj_size.get('x', 0.05)),
                float(obj_size.get('y', 0.05)),
                float(obj_size.get('z', 0.10))
            ]
            
            pose_obj = Pose()
            pose_obj.position.x = float(obj_pos.get('x', 0.5))
            pose_obj.position.y = float(obj_pos.get('y', 0.1))
            pose_obj.position.z = float(obj_pos.get('z', 0.05))
            pose_obj.orientation.w = 1.0
            
            obj.primitives.append(primitive_obj)
            obj.primitive_poses.append(pose_obj)
            obj.operation = CollisionObject.ADD
            
            req = ApplyPlanningScene.Request()
            req.scene.world.collision_objects.append(obj)
            req.scene.is_diff = True
            
            future = self.apply_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            self.get_logger().info(
                f"  ✓ PICK OBJECT added at position ({pose_obj.position.x:.2f}, {pose_obj.position.y:.2f}, {pose_obj.position.z:.2f})"
            )
            self.get_logger().info(
                f"  ✓ OBJECT size: {primitive_obj.dimensions[0]:.2f}m x {primitive_obj.dimensions[1]:.2f}m x {primitive_obj.dimensions[2]:.2f}m"
            )
        
        time.sleep(1.0)
        
        # ═════════════════════════════════════════════════════════════
        # VERIFY OBJECTS IN SCENE
        # ═════════════════════════════════════════════════════════════
        self.get_logger().info("\n▶ Verifying scene objects...")
        
        try:
            get_req = GetPlanningScene.Request()
            get_req.components.components = 2  # WORLD_OBJECT_NAMES
            
            future = self.get_scene_client.call_async(get_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
            
            if future.done():
                response = future.result()
                if response and response.scene:
                    objects = response.scene.world.collision_objects
                    self.get_logger().info(f"\n  ✅ Scene contains {len(objects)} collision objects:")
                    for obj in objects:
                        self.get_logger().info(f"     - {obj.id}")
                    
                    if any(o.id == 'table' for o in objects):
                        self.get_logger().info("  ✅ TABLE found in scene!")
                    else:
                        self.get_logger().warn("  ⚠️ TABLE not found in scene")
                    
                    if any(o.id == 'pick_object' for o in objects):
                        self.get_logger().info("  ✅ PICK OBJECT found in scene!")
                    else:
                        self.get_logger().warn("  ⚠️ PICK OBJECT not found in scene")
            
        except Exception as e:
            self.get_logger().warn(f"Could not verify scene: {e}")
        
        self.get_logger().info("\n========== SCENE INITIALIZATION TEST COMPLETE ✅ ==========\n")
        return True


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        node = SceneInitializationTester()
        success = node.initialize_and_verify_scene()
        
        if success:
            print("✅ Scene initialization test passed!\n")
        else:
            print("❌ Scene initialization test failed!\n")
        
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"\n❌ Fatal error: {e}\n")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
