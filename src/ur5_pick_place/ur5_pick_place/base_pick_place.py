#!/usr/bin/env python3
"""Base class for UR5 pick and place operations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Callable, Union, Dict
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5_pick_place.config_loader import ConfigLoader, JointPosition
from ur5_pick_place.gripper_controller import AbstractGripperController, GripperControllerFactory
from ur5_pick_place.validators import SafetyValidator
from ur5_pick_place.ik_solver import UR5IKSolver, JOINT_NAMES


@dataclass
class StageResult:
    """Result of a single stage execution."""
    stage_name: str
    success: bool
    duration_sec: float = 0.0


class PickPlaceBase(Node, ABC):
    """Base class for all pick and place operations."""

    def __init__(self, node_name: str):
        """Initialize base pick place node."""
        super().__init__(node_name)
        
        self.logger = self.get_logger()
        
        # Load configuration
        try:
            self.config = ConfigLoader()
            ConfigLoader.load_from_file('config/ur5_pick_place_config.yaml')
            self.logger.info("Configuration loaded")
        except Exception as e:
            self.logger.error(f"Failed to load configuration: {e}")
            raise

        # Initialize IK Solver
        try:
            self.solver = UR5IKSolver(self)
            self.logger.info("IK Solver initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize IK Solver: {e}")
            raise

        # Initialize gripper controller
        self.gripper: Optional[AbstractGripperController] = None
        self.gripper_ready = False

        # Trajectory publisher for direct joint control
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Execution state
        self.stages_executed: List[StageResult] = []

    def _initialize_gripper(self) -> bool:
        """Initialize gripper controller."""
        try:
            self.gripper = GripperControllerFactory.create_simulated(self)
            self.gripper_ready = True
            self.logger.info("Gripper controller initialized")
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize gripper: {e}")
            return False

    def move_to_position(self, position: Union[JointPosition, Dict], retries: int = 3) -> bool:
        """Move robot to position. Accepts both JointPosition and dict(x,y,z)."""
        
        for attempt in range(retries):
            try:
                # Handle both JointPosition and dict input
                if isinstance(position, dict):
                    # Cartesian input: x, y, z → solve IK
                    x = position.get('x', 0.0)
                    y = position.get('y', 0.0)
                    z = position.get('z', 0.0)
                    qx = position.get('qx', 0.0)
                    qy = position.get('qy', 0.707)
                    qz = position.get('qz', 0.0)
                    qw = position.get('qw', 0.707)
                    
                    ik_result = self.solver.solve(x, y, z, qx, qy, qz, qw)
                    if not ik_result:
                        self.logger.error(f"IK failed for x={x}, y={y}, z={z}")
                        return False
                    
                    joint_angles = ik_result['joints']
                    pos_name = f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
                    
                elif isinstance(position, JointPosition):
                    # Direct joint angles
                    joint_angles = position.angles
                    pos_name = position.name
                else:
                    self.logger.error(f"Invalid position type: {type(position)}")
                    return False

                # Validate before moving
                validation = SafetyValidator.pre_flight_check(joint_angles)
                if not validation.passed:
                    self.logger.error(f"Safety validation failed: {validation.message}")
                    continue

                # Publish trajectory
                msg = JointTrajectory()
                msg.joint_names = JOINT_NAMES
                msg.header.stamp = self.get_clock().now().to_msg()
                
                point = JointTrajectoryPoint()
                point.positions = joint_angles
                point.time_from_start.sec = 2
                msg.points.append(point)
                
                self.logger.info(f"Moving to '{pos_name}' (attempt {attempt+1}/{retries})")
                self.joint_pub.publish(msg)
                
                # Wait for trajectory to complete (rough estimate)
                rclpy.spin_once(self, timeout_sec=2.5)
                
                self.logger.info(f"Movement to '{pos_name}' completed")
                return True
                
            except Exception as e:
                self.logger.error(f"Error during movement: {e}")
                time.sleep(1.0)

        return False

    def execute_stage(self, stage_name: str, stage_fn: Callable) -> StageResult:
        """Execute a stage and track execution time."""
        try:
            start_time = time.time()
            
            self.logger.info(f"\nExecuting: {stage_name}")
            
            success = stage_fn()
            
            duration = time.time() - start_time
            
            result = StageResult(
                stage_name=stage_name,
                success=success,
                duration_sec=duration
            )
            
            if success:
                self.logger.info(f"✓ {stage_name} completed in {duration:.2f}s")
            else:
                self.logger.error(f"✗ {stage_name} FAILED")
            
            self.stages_executed.append(result)
            return result

        except Exception as e:
            self.logger.error(f"✗ {stage_name} EXCEPTION: {e}")
            return StageResult(stage_name=stage_name, success=False, duration_sec=0.0)

    @abstractmethod
    def run_sequence(self) -> bool:
        """Abstract method: implement specific sequence steps."""
        pass

    def run(self) -> bool:
        """Main execution orchestrator."""
        try:
            self.logger.info("Starting pick and place sequence")

            if not self._initialize_gripper():
                return False

            success = self.run_sequence()
            self._print_execution_summary(success)
            return success

        except Exception as e:
            self.logger.error(f"Fatal error: {e}")
            return False

    def _print_execution_summary(self, success: bool):
        """Print execution summary."""
        self.logger.info(f"\n{'='*60}")
        self.logger.info(f"EXECUTION SUMMARY - {'SUCCESS ✓' if success else 'FAILED ✗'}")
        self.logger.info(f"{'='*60}")
        
        total_time = sum(s.duration_sec for s in self.stages_executed)
        self.logger.info(f"Total execution time: {total_time:.2f}s")
        
        for stage in self.stages_executed:
            status = "✓" if stage.success else "✗"
            self.logger.info(f"{status} {stage.stage_name:<40} {stage.duration_sec:>7.2f}s")
        
        self.logger.info(f"{'='*60}\n")
