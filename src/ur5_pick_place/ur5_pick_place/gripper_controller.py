#!/usr/bin/env python3
"""Gripper abstraction layer for UR5 gripper control."""

import asyncio
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
import rclpy
from rclpy.node import Node


@dataclass
class GripperProfile:
    """Gripper command profile."""
    position: float
    effort: float
    duration_sec: float = 2.0


class AbstractGripperController(ABC):
    """Abstract base class for gripper control."""

    def __init__(self, node: Node, timeout: float = 5.0):
        self.node = node
        self.timeout = timeout
        self.logger = node.get_logger()

    @abstractmethod
    def async_open(self) -> bool:
        """Open gripper."""
        pass

    @abstractmethod
    def async_close(self) -> bool:
        """Close gripper."""
        pass

    @abstractmethod
    def async_grasp(self, profile: GripperProfile) -> bool:
        """Execute grasp with profile."""
        pass


class SimulatedGripperController(AbstractGripperController):
    """Simulated gripper controller for testing."""

    def __init__(self, node: Node, timeout: float = 5.0):
        super().__init__(node, timeout)
        self.current_position = 0.0
        self.logger.info("SimulatedGripperController initialized")

    def async_open(self) -> bool:
        """Simulate opening gripper (non-blocking)."""
        profile = GripperProfile(position=0.0, effort=0.0, duration_sec=1.5)
        return self.async_grasp(profile)

    def async_close(self) -> bool:
        """Simulate closing gripper (non-blocking)."""
        profile = GripperProfile(position=0.72, effort=135.0, duration_sec=2.0)
        return self.async_grasp(profile)

    def async_grasp(self, profile: GripperProfile) -> bool:
        """Simulate gripper motion WITHOUT BLOCKING.
        
        Uses a callback-based approach instead of time.sleep to avoid
        blocking the ROS2 event loop and preventing joint_states updates.
        """
        self.logger.info(f"[SIM] Gripper command: pos={profile.position:.2f}, duration={profile.duration_sec}s")
        
        try:
            # Schedule the completion asynchronously using a timer
            def _complete_motion():
                self.current_position = profile.position
                self.logger.info(f"[SIM] Gripper motion completed: pos={profile.position:.2f}")
            
            # Use a ROS2 timer to simulate motion without blocking
            timer = self.node.create_timer(
                profile.duration_sec,
                _complete_motion,
                clock=self.node.get_clock()
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Simulated gripper failed: {e}")
            return False


class GripperControllerFactory:
    """Factory for creating gripper controllers."""

    @staticmethod
    def create_simulated(node: Node, timeout: float = 5.0) -> SimulatedGripperController:
        """Create simulated gripper controller."""
        return SimulatedGripperController(node, timeout)
