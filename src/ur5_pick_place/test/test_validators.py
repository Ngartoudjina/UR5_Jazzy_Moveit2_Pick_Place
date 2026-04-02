#!/usr/bin/env python3
"""Unit tests for validators module."""

import pytest
import math
from ur5_pick_place.validators import JointValidator, WorkspaceValidator, SafetyValidator


class TestJointValidator:
    """Tests for joint angle validation."""

    def test_valid_home_position(self):
        """Test valid home position."""
        angles = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        result = JointValidator.validate_angles(angles)
        assert result.passed

    def test_invalid_joint_count(self):
        """Test error on incorrect number of joints."""
        angles = [0.0, 0.0, 0.0, 0.0]
        result = JointValidator.validate_angles(angles)
        assert not result.passed


class TestWorkspaceValidator:
    """Tests for workspace position validation."""

    def test_valid_position(self):
        """Test valid position within workspace."""
        result = WorkspaceValidator.validate_position(0.5, 0.3, 0.8)
        assert result.passed

    def test_out_of_reach(self):
        """Test detection of unreachable position."""
        result = WorkspaceValidator.validate_position(1.5, 0.0, 0.5)
        assert not result.passed

    def test_height_exceeds_maximum(self):
        """Test detection of height exceeding maximum."""
        result = WorkspaceValidator.validate_position(0.5, 0.3, 1.6)
        assert not result.passed


class TestSafetyValidator:
    """Tests for comprehensive safety validation."""

    def test_preflight_check_all_good(self):
        """Test pre-flight check with all parameters valid."""
        angles = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
        ee_pos = (0.5, 0.3, 0.8)
        
        result = SafetyValidator.pre_flight_check(angles, ee_position=ee_pos)
        assert result.passed

    def test_preflight_check_joint_violation(self):
        """Test pre-flight check with joint violation."""
        angles = [3*math.pi, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = SafetyValidator.pre_flight_check(angles)
        assert not result.passed


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
