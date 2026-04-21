#!/usr/bin/env python3
"""Safety validation framework for UR5 pick & place operations."""

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional


@dataclass
class ValidationResult:
    """Result of validation check."""
    passed: bool
    message: str
    details: str = ""
    warnings: List[str] = field(default_factory=list)


class JointValidator:
    """Validates joint angles against UR5 hardware limits."""

    JOINT_LIMITS = [
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi * 2, math.pi * 2),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi * 2, math.pi * 2),
    ]

    JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

    @staticmethod
    def validate_angles(angles: List[float]) -> ValidationResult:
        """Validate joint angles against limits."""
        
        if not angles or len(angles) != 6:
            return ValidationResult(False, f"Expected 6 joints, got {len(angles)}")

        violations = []
        for i, (angle, (min_limit, max_limit)) in enumerate(zip(angles, JointValidator.JOINT_LIMITS)):
            # For joints with ±π limits, normalize to [-π, π]
            # For joints with ±2π limits, only normalize if value exceeds bounds
            if abs(max_limit - min_limit) == 2 * math.pi:
                # ±2π case: allow full rotation, but check actual value
                normalized = math.atan2(math.sin(angle), math.cos(angle))
            else:
                # ±π case: normalize and validate
                normalized = math.atan2(math.sin(angle), math.cos(angle))
            
            # Check if normalized angle actually respects the limits
            if normalized < min_limit or normalized > max_limit:
                # Also check unreduced angle in case it's legitimately out of range
                if angle < min_limit or angle > max_limit:
                    violations.append(
                        f"{JointValidator.JOINT_NAMES[i]}: {math.degrees(normalized):.1f}° "
                        f"(limits: {math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)"
                    )

        if violations:
            return ValidationResult(
                False, 
                f"Joint limits violated ({len(violations)} joints)",
                details="; ".join(violations)
            )

        return ValidationResult(True, "All joints within limits")


class WorkspaceValidator:
    """Validates end-effector position within workspace."""

    REACH_MAX = 1.3
    HEIGHT_MAX = 1.5

    @staticmethod
    def validate_position(x: float, y: float, z: float) -> ValidationResult:
        """Validate end-effector position in workspace."""
        
        xy_distance = math.sqrt(x**2 + y**2)
        
        if xy_distance > WorkspaceValidator.REACH_MAX:
            return ValidationResult(False, f"Position out of reach: {xy_distance:.3f}m")
        
        if z > WorkspaceValidator.HEIGHT_MAX:
            return ValidationResult(False, f"Height exceeds maximum: {z:.3f}m")
        
        return ValidationResult(True, "Position within workspace")


class SafetyValidator:
    """Comprehensive safety validation combining all checks."""

    @staticmethod
    def pre_flight_check(joint_angles: List[float],
                        ee_position: Optional[Tuple[float, float, float]] = None) -> ValidationResult:
        """Perform comprehensive pre-flight validation."""
        
        joint_result = JointValidator.validate_angles(joint_angles)
        if not joint_result.passed:
            return joint_result

        if ee_position:
            workspace_result = WorkspaceValidator.validate_position(*ee_position)
            if not workspace_result.passed:
                return workspace_result

        return ValidationResult(True, "Pre-flight check PASSED ✓")
