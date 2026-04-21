#!/usr/bin/env python3
"""
MoveIt2 System Diagnostics - Complete Health Check
Provides detailed diagnostics for MoveIt2/ros2_control setup before pick-place execution.

Usage:
    ros2 run ur5_pick_place moveit_system_diagnostics
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPlanningScene
from tf2_ros import TransformListener, Buffer
import time


class MoveItSystemDiagnostics(Node):
    """Complete diagnostic suite for MoveIt2 system"""
    
    def __init__(self):
        super().__init__('moveit_diagnostics')
        
        # TF buffer for transform checking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Expected components
        self.expected_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.expected_frames = [
            'world',
            'base_link',
            'shoulder_link',
            'upper_arm_link',
            'forearm_link',
            'wrist_1_link',
            'wrist_2_link',
            'wrist_3_link'
        ]
        
        self.issues = []
        self.warnings = []
        self.get_logger().info("🔍 MoveIt2 System Diagnostics Initialized")
    
    def run_diagnostics(self) -> bool:
        """Run complete diagnostic suite"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("MOVEIT2 SYSTEM DIAGNOSTICS")
        self.get_logger().info("="*70 + "\n")
        
        # Run all diagnostic checks
        all_ok = True
        all_ok &= self._check_ros2_nodes()
        all_ok &= self._check_tf_tree()
        all_ok &= self._check_planning_scene()
        all_ok &= self._check_action_server()
        all_ok &= self._check_joint_state_publisher()
        
        # Print summary
        self._print_summary(all_ok)
        
        return all_ok
    
    def _check_ros2_nodes(self) -> bool:
        """Check if critical ROS2 nodes are running"""
        self.get_logger().info("\n▶ CHECKING ROS2 NODES")
        self.get_logger().info("─" * 70)
        
        critical_nodes = [
            '/robot_state_publisher',
            '/controller_manager',
            '/move_group',
        ]
        
        all_ok = True
        
        try:
            node_names = self.get_node_names()
            
            for node_name in critical_nodes:
                if node_name in node_names:
                    self.get_logger().info(f"  ✓ {node_name:<40} RUNNING")
                else:
                    self.get_logger().error(f"  ✗ {node_name:<40} MISSING")
                    self.issues.append(f"Node {node_name} not running")
                    all_ok = False
            
        except Exception as e:
            self.get_logger().error(f"  ✗ Error checking nodes: {e}")
            all_ok = False
        
        return all_ok
    
    def _check_tf_tree(self) -> bool:
        """Check TF tree completeness"""
        self.get_logger().info("\n▶ CHECKING TF TREE")
        self.get_logger().info("─" * 70)
        
        all_ok = True
        time.sleep(0.5)  # Give TF time to populate
        
        for frame in self.expected_frames:
            try:
                transform = self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                self.get_logger().info(f"  ✓ {frame:<40} FOUND")
            except Exception as e:
                self.get_logger().error(f"  ✗ {frame:<40} MISSING - {str(e)[:40]}")
                self.warnings.append(f"Transform {frame} not found")
                all_ok = False
        
        return all_ok
    
    def _check_planning_scene(self) -> bool:
        """Check if planning scene is available"""
        self.get_logger().info("\n▶ CHECKING PLANNING SCENE")
        self.get_logger().info("─" * 70)
        
        try:
            scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
            
            # Try to get planning scene
            if not scene_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("  ✗ /get_planning_scene service        NOT AVAILABLE")
                self.issues.append("Planning scene service not available")
                return False
            
            self.get_logger().info("  ✓ /get_planning_scene service         AVAILABLE")
            
            req = GetPlanningScene.Request()
            req.components.components = 31  # All components
            
            future = scene_client.call_async(req)
            
            # Wait for response
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                if response and response.scene:
                    self.get_logger().info(f"  ✓ Planning scene accessible          {len(response.scene.world.collision_objects)} objects")
                    return True
            
            self.get_logger().error("  ✗ Cannot query planning scene")
            return False
            
        except Exception as e:
            self.get_logger().error(f"  ✗ Planning scene error: {str(e)[:60]}")
            return False
    
    def _check_action_server(self) -> bool:
        """Check if action server is available"""
        self.get_logger().info("\n▶ CHECKING ACTION SERVER")
        self.get_logger().info("─" * 70)
        
        try:
            action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/ur5_arm_controller/follow_joint_trajectory'
            )
            
            self.get_logger().info("  ⏳ Waiting for action server (timeout: 10s)...")
            
            if action_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().info("  ✓ /ur5_arm_controller/follow_joint_trajectory   READY")
                return True
            else:
                self.get_logger().error("  ✗ Action server timeout (10s)         NOT READY")
                self.issues.append("Action server timeout - controller may not be activated")
                return False
                
        except Exception as e:
            self.get_logger().error(f"  ✗ Action server check failed: {e}")
            return False
    
    def _check_joint_state_publisher(self) -> bool:
        """Check joint state publisher"""
        self.get_logger().info("\n▶ CHECKING JOINT STATE PUBLISHER")
        self.get_logger().info("─" * 70)
        
        try:
            from sensor_msgs.msg import JointState
            
            # Subscribe to joint_states with QoS
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=1
            )
            
            joint_state_received = False
            
            def js_callback(msg):
                nonlocal joint_state_received
                joint_state_received = True
                received_joints = list(msg.name)
                missing = [j for j in self.expected_joints if j not in received_joints]
                
                if not missing:
                    self.get_logger().info(f"  ✓ All {len(received_joints)} expected joints publishing")
                else:
                    self.get_logger().warn(f"  ⚠ Missing joints: {missing}")
                    self.warnings.append(f"Joint state missing: {missing}")
            
            sub = self.create_subscription(JointState, '/joint_states', js_callback, qos)
            
            # Wait for message
            start = time.time()
            while not joint_state_received and (time.time() - start) < 3.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            self.destroy_subscription(sub)
            
            if not joint_state_received:
                self.get_logger().error("  ✗ No joint_states received (timeout: 3s)")
                self.issues.append("Joint state publisher not working")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"  ✗ Joint state check failed: {e}")
            return False
    
    def _print_summary(self, all_ok: bool):
        """Print diagnostic summary"""
        self.get_logger().info("\n" + "="*70)
        
        if self.issues:
            self.get_logger().error(f"\n❌ CRITICAL ISSUES ({len(self.issues)}):")
            for i, issue in enumerate(self.issues, 1):
                self.get_logger().error(f"   {i}. {issue}")
        
        if self.warnings:
            self.get_logger().warn(f"\n⚠️  WARNINGS ({len(self.warnings)}):")
            for i, warn in enumerate(self.warnings, 1):
                self.get_logger().warn(f"   {i}. {warn}")
        
        if all_ok:
            self.get_logger().info("\n✅ ALL CHECKS PASSED - System ready for pick-place")
            self.get_logger().info("\nYou can now run:")
            self.get_logger().info("  ros2 run ur5_pick_place pick_place_moveit2")
        else:
            self.get_logger().error("\n❌ SYSTEM NOT READY - Fix issues before running pick-place")
            if self.issues:
                self.get_logger().info("\n💡 TROUBLESHOOTING SUGGESTIONS:")
                self._print_troubleshooting()
        
        self.get_logger().info("\n" + "="*70 + "\n")
    
    def _print_troubleshooting(self):
        """Print troubleshooting suggestions based on issues"""
        for issue in self.issues:
            if "Node" in issue and "not running" in issue:
                self.get_logger().info("   • Make sure MoveIt2 is running:")
                self.get_logger().info("     ros2 launch ur5_moveit moveit.launch.py")
            elif "Action server" in issue:
                self.get_logger().info("   • Controller may not be activated. Check:")
                self.get_logger().info("     1. ros2 controller list")
                self.get_logger().info("     2. Check /tmp/.ros/latest_log for errors")
                self.get_logger().info("     3. Verify ur5_arm_controller is in ros2_controllers.yaml")
            elif "Planning scene" in issue:
                self.get_logger().info("   • move_group may not be running properly")
                self.get_logger().info("   • Check the launch output for 'move_group' errors")


def main():
    rclpy.init()
    
    try:
        node = MoveItSystemDiagnostics()
        success = node.run_diagnostics()
        
        exit_code = 0 if success else 1
        
    except KeyboardInterrupt:
        node.get_logger().info("\n⚠️  Interrupted by user")
        exit_code = 130
    except Exception as e:
        print(f"\n❌ Fatal error: {e}\n")
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
    
    return exit_code


if __name__ == '__main__':
    exit(main())
