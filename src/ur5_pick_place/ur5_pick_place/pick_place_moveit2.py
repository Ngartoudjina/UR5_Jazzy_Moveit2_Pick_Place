#!/usr/bin/env python3
"""
================================================================================
pick_place_moveit2.py - FollowJointTrajectory direct (ROS2 Jazzy compatible)
================================================================================

Approche : ros2_control FollowJointTrajectory action directe.
Compatible ROS2 Humble ET Jazzy (pas de dépendance moveit_py).

CORRECTIONS APPLIQUÉES :
  - En-tête corrigé (n'est plus "Humble only")
  - Chemins scene_config.yaml portables (sans ~/ros2_humble_ws/)
  - _initialize_scene() attend la confirmation ApplyPlanningScene
  - Logs "Humble Mode" remplacés par des logs neutres
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

import yaml
import os
import time


class PickPlaceMoveIt2(Node):
    def __init__(self):
        super().__init__('pick_place_moveit2')
        
        self.get_logger().info("🔧 Initializing Pick & Place node (Humble compatible)...")
        
        # Initialize action client for ros2_control
        try:
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/ur5_arm_controller/follow_joint_trajectory'
            )
            self.get_logger().info("✓ Action client created")
        except Exception as e:
            self.get_logger().error(f"Failed to create action client: {e}")
            raise
        
        # Create client for MoveIt2 scene
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        
        # Load configuration
        self.config = self._load_config()
        
        # Joint names for UR5
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        # Default positions (if config not loaded)
        self.positions = self._load_positions()
        
        # Initialize scene with table and object
        self._initialize_scene()
        
        self.get_logger().info("✅ Pick & Place Node Initialized")
    
    def _load_config(self):
        """Load scene_config.yaml — chemins portables (sans ros2_humble_ws)."""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'scene_config.yaml'),
            os.path.join(os.path.dirname(__file__), '..', 'config', 'scene_config.yaml'),
            os.path.join(
                os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0],
                'share', 'ur5_pick_place', 'scene_config.yaml'
            ),
            os.path.join(
                os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0],
                'share', 'ur5_pick_place', 'config', 'scene_config.yaml'
            ),
        ]

        for path in possible_paths:
            path = os.path.normpath(path)
            if os.path.exists(path):
                try:
                    with open(path) as f:
                        cfg = yaml.safe_load(f)
                        self.get_logger().info(f"✓ Config chargée depuis : {path}")
                        return cfg
                except Exception as e:
                    self.get_logger().warn(f"Echec lecture {path}: {e}")

        self.get_logger().warn("⚠️ scene_config.yaml introuvable — valeurs par défaut")
        return {}
    
    def _load_positions(self):
        """Load joint positions from config"""
        positions = {}
        
        # Try to load from config
        poses_cfg = self.config.get('poses', {})
        
        for pose_name, pose_data in poses_cfg.items():
            if isinstance(pose_data, dict):
                positions[pose_name] = [
                    float(pose_data.get('shoulder_pan', 0.0)),
                    float(pose_data.get('shoulder_lift', -1.57)),
                    float(pose_data.get('elbow', 1.57)),
                    float(pose_data.get('wrist_1', -1.57)),
                    float(pose_data.get('wrist_2', -1.57)),
                    float(pose_data.get('wrist_3', 0.0)),
                ]
        
        # Add defaults if not in config
        if 'HOME' not in positions:
            positions['HOME'] = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        
        if 'APPROCHE_PICK' not in positions:
            positions['APPROCHE_PICK'] = [0.0, -1.0, 1.57, -1.57, -1.57, 0.0]
        
        if 'PICK' not in positions:
            positions['PICK'] = [0.0, -1.45, 1.57, -1.57, -1.57, 0.0]
        
        if 'LIFT' not in positions:
            positions['LIFT'] = [0.0, -0.8, 1.57, -1.57, -1.57, 0.0]
        
        self.get_logger().info(f"✓ Loaded {len(positions)} positions")
        return positions
    
    def _initialize_scene(self):
        """Initialize MoveIt2 scene with table and pick object.

        CORRECTION : appels call_async() avec spin_until_future_complete()
        pour attendre la confirmation avant de continuer.
        """
        try:
            if not self.scene_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn("⚠️ ApplyPlanningScene non disponible — scène non initialisée")
                return

            scene_cfg = self.config.get('scene', {})

            for obj_key, obj_id, defaults in [
                ('table',       'table',       {'pos': (0.65, 0.0, -0.025), 'size': (0.8, 0.8, 0.05)}),
                ('pick_object', 'pick_object', {'pos': (0.5,  0.1,  0.05),  'size': (0.05,0.05,0.10)}),
            ]:
                cfg = scene_cfg.get(obj_key, {})
                pos = cfg.get('position', {})
                sz  = cfg.get('size', {})

                co = CollisionObject()
                co.id = obj_id
                co.header.frame_id = 'world'
                co.header.stamp = self.get_clock().now().to_msg()

                prim = SolidPrimitive()
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [
                    float(sz.get('x', defaults['size'][0])),
                    float(sz.get('y', defaults['size'][1])),
                    float(sz.get('z', defaults['size'][2])),
                ]

                pose = Pose()
                pose.position.x = float(pos.get('x', defaults['pos'][0]))
                pose.position.y = float(pos.get('y', defaults['pos'][1]))
                pose.position.z = float(pos.get('z', defaults['pos'][2]))
                pose.orientation.w = 1.0

                co.primitives.append(prim)
                co.primitive_poses.append(pose)
                co.operation = CollisionObject.ADD

                req = ApplyPlanningScene.Request()
                req.scene.world.collision_objects.append(co)
                req.scene.is_diff = True

                future = self.scene_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

                if future.done() and future.result() and future.result().success:
                    self.get_logger().info(
                        f"✅ '{obj_id}' @ ({pose.position.x:.2f},"
                        f" {pose.position.y:.2f}, {pose.position.z:.2f})"
                    )
                else:
                    self.get_logger().warn(f"⚠️ '{obj_id}' non confirmé par ApplyPlanningScene")

        except Exception as e:
            self.get_logger().warn(f"⚠️ Echec initialisation scène : {e}")
    
    def _create_trajectory(self, joint_positions, duration=3.0):
        """Create a JointTrajectory for ros2_control"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Single point at end of trajectory
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(self.joint_names)
        
        # Set duration
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
        return trajectory
    
    def move_to_position(self, pose_name: str, duration=3.0) -> bool:
        """Move robot to named position (blocking)"""
        if pose_name not in self.positions:
            self.get_logger().error(f"❌ Unknown position: {pose_name}")
            return False
        
        self.get_logger().info(f"→ Moving to {pose_name}")
        
        positions = self.positions[pose_name]
        self.get_logger().debug(f"   Positions: {positions}")
        
        # Create trajectory
        trajectory = self._create_trajectory(positions, duration=duration)
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        try:
            # Wait for server with much longer timeout - spawners can be slow
            self.get_logger().info("⏳ Waiting for ros2_control action server...")
            if not self.action_client.wait_for_server(timeout_sec=25.0):
                self.get_logger().error("❌ ros2_control action server not ready after 25 seconds!")
                self.get_logger().error("   Possible cause: ur5_arm_controller spawner failed to load/activate")
                self.get_logger().error("   Try launching: ros2 launch ur5_moveit moveit.launch.py")
                return False
            
            self.get_logger().info("✓ Action server ready!")
            
            # Send goal and wait
            send_goal_future = self.action_client.send_goal_async(goal_msg)
            
            # This will block until goal is processed
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            if send_goal_future.done():
                goal_handle = send_goal_future.result()
                if goal_handle:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 2.0)
                    
                    if result_future.done():
                        self.get_logger().info(f"✓ {pose_name} completed")
                        return True
            
            self.get_logger().error(f"❌ Move to {pose_name} failed")
            return False
            
        except Exception as e:
            self.get_logger().error(f"❌ Exception during move: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run_pick_place(self):
        """Run complete pick-place sequence"""
        self.get_logger().info("\n========== DÉMARRAGE PICK & PLACE ==========\n")
        
        try:
            # Sequence of moves
            sequence = [
                ('HOME', 1.0),
                ('APPROCHE_PICK', 1.0),
                ('PICK', 2.0),
            ]
            
            for move_name, wait_time in sequence:
                if not self.move_to_position(move_name, duration=3.0):
                    self.get_logger().error(f"❌ Pick-place failed at {move_name}")
                    return False
                
                time.sleep(wait_time)
            
            # Gripper simulation
            self.get_logger().info("→ Gripper FERMÉ (objet saisi)")
            time.sleep(1.0)
            
            # Remaining moves
            remaining = [
                ('LIFT', 1.0),
                ('HOME', 2.0),
            ]
            
            for move_name, wait_time in remaining:
                if not self.move_to_position(move_name, duration=3.0):
                    self.get_logger().error(f"❌ Pick-place failed at {move_name}")
                    return False
                
                time.sleep(wait_time)
            
            self.get_logger().info("→ Gripper OUVERT (objet posé)")
            time.sleep(0.5)
            
            self.get_logger().info("\n========== PICK & PLACE TERMINÉ ✅ ==========\n")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main entry point"""
    rclpy.init()
    
    try:
        node = PickPlaceMoveIt2()
        
        # Run pick-place
        success = node.run_pick_place()
        
        if success:
            print("\n✅ Pick-place completed successfully!\n")
        else:
            print("\n❌ Pick-place failed!\n")
        
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
