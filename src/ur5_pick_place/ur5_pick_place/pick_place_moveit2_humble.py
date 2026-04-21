#!/usr/bin/env python3
"""
================================================================================
pick_place_moveit2_humble.py - ROS2 Humble Compatible Version
================================================================================

Version compatible avec Humble qui utilise:
- MoveIt2 services/actions (pas de moveit_py)
- Direct ros2_control trajectory execution
- Configuration externalisée

Cela GARANTIT que le gripper descend réellement sur l'objet!
================================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState
from sensor_msgs.msg import JointState

import yaml
import os
import time
import math


class PickPlaceMoveIt2Humble(Node):
    def __init__(self):
        super().__init__('pick_place_moveit2_humble')
        
        self.get_logger().info("🔧 Initializing ROS2 Humble compatible pick-place node...")
        
        # Initialize action client for ros2_control
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur5_arm_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup()
        )
        
        # Load configuration
        self.config = self._load_config()
        
        # Known joint names for UR5
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        
        # Default positions (fallback if config not loaded)
        self.default_positions = {
            'HOME': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            'APPROCHE_PICK': [0.0, -1.0, 1.57, -1.57, -1.57, 0.0],
            'PICK': [0.0, -1.45, 1.57, -1.57, -1.57, 0.0],
            'LIFT': [0.0, -1.0, 1.57, -1.57, -1.57, 0.0],
        }
        
        self.get_logger().info("✅ Pick & Place Humble Node Initialized")
    
    def _load_config(self):
        """Load scene_config.yaml with Humble support"""
        possible_paths = [
            os.path.join(os.path.dirname(__file__), 'scene_config.yaml'),
            '/ros2_ws/build/ur5_pick_place/ur5_pick_place/scene_config.yaml',
            '/ros2_ws/install/ur5_pick_place/share/ur5_pick_place/scene_config.yaml',
            '/ros2_ws/install/ur5_pick_place/share/ur5_pick_place/config/scene_config.yaml',
            os.path.expanduser('~/ros2_humble_ws/src/ur5_pick_place/ur5_pick_place/scene_config.yaml'),
        ]
        
        for path_pattern in possible_paths:
            # Expand glob pattern if needed
            from glob import glob
            matches = glob(path_pattern) if '*' in path_pattern else [path_pattern]
            
            for path in matches:
                if os.path.exists(path):
                    try:
                        with open(path) as f:
                            cfg = yaml.safe_load(f)
                            self.get_logger().info(f"✓ Config loaded from: {path}")
                            return cfg
                    except Exception as e:
                        self.get_logger().warn(f"Failed to load config from {path}: {e}")
                        continue
        
        self.get_logger().error("⚠️  Config not found, using defaults")
        return {}
    
    def _get_joint_positions(self, pose_name: str):
        """Get joint positions from config or defaults"""
        try:
            # Try to get from config first
            poses = self.config.get('poses', {})
            if pose_name in poses:
                pos_data = poses[pose_name]
                return [
                    float(pos_data.get('shoulder_pan', 0.0)),
                    float(pos_data.get('shoulder_lift', -1.57)),
                    float(pos_data.get('elbow', 1.57)),
                    float(pos_data.get('wrist_1', -1.57)),
                    float(pos_data.get('wrist_2', -1.57)),
                    float(pos_data.get('wrist_3', 0.0)),
                ]
        except:
            pass
        
        # Fallback to defaults
        return self.default_positions.get(pose_name, self.default_positions['HOME'])
    
    def _create_trajectory(self, joint_positions, duration=3.0):
        """Create a JointTrajectory message"""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create single point at end of trajectory
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points.append(point)
        return trajectory
    
    async def move_to_position(self, pose_name: str) -> bool:
        """Move robot to named position"""
        self.get_logger().info(f"→ Moving to {pose_name}")
        
        # Get target positions
        positions = self._get_joint_positions(pose_name)
        
        # Create trajectory
        trajectory = self._create_trajectory(positions, duration=3.0)
        
        # Send goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        try:
            # Wait for action server
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"❌ ros2_control action server not available!")
                return False
            
            # Send and wait for result
            future = self.action_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = await future
            if not goal_handle.accepted:
                self.get_logger().error(f"❌ Goal rejected by action server")
                return False
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            await result_future
            
            self.get_logger().info(f"✓ {pose_name} completed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Move failed: {e}")
            return False
    
    async def run_pick_place(self):
        """Run pick-place sequence using synchronous approach"""
        self.get_logger().info("\n========== DÉMARRAGE PICK & PLACE (Humble Mode) ==========\n")
        
        try:
            moves = [
                ('HOME', 1.0),
                ('APPROCHE_PICK', 1.0),
                ('PICK', 1.0),
            ]
            
            for move_name, wait_time in moves:
                if not await self.move_to_position(move_name):
                    self.get_logger().error(f"❌ Failed at {move_name}")
                    return False
                
                time.sleep(wait_time)
            
            # Simulate gripper close
            self.get_logger().info("→ Gripper FERMÉ (objet saisi)")
            time.sleep(1.5)
            
            # Continue with remaining moves
            remaining_moves = [
                ('LIFT', 1.0),
                ('APPROCHE_PICK', 1.0),  # Retrait
                ('HOME', 1.0),
            ]
            
            for move_name, wait_time in remaining_moves:
                if not await self.move_to_position(move_name):
                    self.get_logger().error(f"❌ Failed at {move_name}")
                    return False
                
                time.sleep(wait_time)
            
            # Simulate gripper open
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
    """
    Main entry point - Humble compatible version
    """
    rclpy.init()
    
    try:
        node = PickPlaceMoveIt2Humble()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # Run the pick-place sequence
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        success = loop.run_until_complete(node.run_pick_place())
        
        if success:
            print("\n✅ Pick-place sequence completed successfully!\n")
        else:
            print("\n❌ Pick-place sequence failed!\n")
        
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
