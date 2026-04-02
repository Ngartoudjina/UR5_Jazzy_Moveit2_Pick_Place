#!/usr/bin/env python3
"""Configuration loader for UR5 Pick & Place system."""

import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Any
import yaml


@dataclass
class JointPosition:
    """Represents a named joint position."""
    name: str
    angles: List[float]
    description: str = ""


@dataclass
class GripperProfile:
    """Gripper configuration for different grasping scenarios."""
    name: str
    position: float
    effort: float
    duration_sec: float
    description: str = ""


class ConfigLoader:
    """Singleton configuration loader with validation."""

    _instance = None
    _config_data: Dict[str, Any] = {}
    _joint_positions: Dict[str, JointPosition] = {}
    _gripper_profiles: Dict[str, GripperProfile] = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
        return cls._instance

    @classmethod
    def load_from_file(cls, config_path: str) -> bool:
        """Load configuration from YAML file."""
        # Support both absolute and relative paths
        if not os.path.isabs(config_path):
            # Try multiple locations for relative paths
            possible_paths = [
                config_path,
                os.path.join(os.getcwd(), config_path),
                os.path.join(os.path.dirname(__file__), '..', '..', 'config', os.path.basename(config_path)),
                os.path.expanduser(f"~/ros2_humble_ws/UR5_Jazzy_Moveit2_Pick_Place/src/ur5_pick_place/{config_path}"),
                os.path.expanduser(f"~/ros2_humble_ws/UR5_Jazzy_Moveit2_Pick_Place/install/ur5_pick_place/share/ur5_pick_place/{config_path}"),
            ]
            
            found_path = None
            for path in possible_paths:
                if os.path.exists(path):
                    found_path = path
                    break
            
            if found_path:
                config_path = found_path
            else:
                raise FileNotFoundError(f"Config file not found: {config_path}\nTried: {possible_paths}")
        
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        try:
            with open(config_path, 'r') as f:
                cls._config_data = yaml.safe_load(f)
            
            if not cls._config_data:
                raise ValueError("Empty configuration file")
            
            cls._parse_named_positions()
            cls._parse_gripper_profiles()
            
            return cls.validate()
        except Exception as e:
            raise RuntimeError(f"Failed to load configuration: {e}")

    @classmethod
    def _parse_named_positions(cls):
        """Parse named joint positions from config."""
        cls._joint_positions = {}
        positions_config = cls._config_data.get("named_positions", {})
        
        for name, config in positions_config.items():
            angles = config.get("joints", [])
            description = config.get("description", "")
            
            if len(angles) != 6:
                raise ValueError(f"Position '{name}' must have 6 joint angles")
            
            cls._joint_positions[name] = JointPosition(
                name=name,
                angles=angles,
                description=description
            )

    @classmethod
    def _parse_gripper_profiles(cls):
        """Parse gripper profiles from config."""
        cls._gripper_profiles = {}
        gripper_config = cls._config_data.get("gripper", {})
        profiles = gripper_config.get("profiles", {})
        
        for name, config in profiles.items():
            cls._gripper_profiles[name] = GripperProfile(
                name=name,
                position=float(config.get("position", 0.0)),
                effort=float(config.get("effort", 0.0)),
                duration_sec=float(config.get("duration_sec", 1.0)),
                description=config.get("description", "")
            )

    @classmethod
    def validate(cls) -> bool:
        """Validate configuration structure."""
        required_sections = ["robot", "planning", "gripper", "named_positions"]
        
        for section in required_sections:
            if section not in cls._config_data:
                raise ValueError(f"Missing required config section: {section}")
        
        if len(cls._joint_positions) < 3:
            raise ValueError(f"Minimum 3 named positions required")
        
        return True

    @classmethod
    def get_position(cls, name: str) -> Optional[JointPosition]:
        """Get named position by name."""
        return cls._joint_positions.get(name)

    @classmethod
    def get_gripper_profile(cls, name: str) -> Optional[GripperProfile]:
        """Get gripper profile by name."""
        return cls._gripper_profiles.get(name)

    @classmethod
    def get_group_name(cls) -> str:
        """Get MoveGroup name from config."""
        return cls._config_data.get("robot", {}).get("group_name", "manipulator")

    @classmethod
    def get_planning_time(cls) -> float:
        """Get planning time limit."""
        return float(cls._config_data.get("planning", {}).get("planning_time", 15.0))
