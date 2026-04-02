# UR5 Pick & Place v2.0 - User Guide

## Quick Start

```bash
# 1. Load configuration
source install/setup.bash

# 2. Build the package
colcon build --packages-select ur5_pick_place

# 3. Run pick and place
ros2 run ur5_pick_place pick_place_node

# 4. Move to specific position
ros2 run ur5_pick_place pose_sender_node --position home
ros2 run ur5_pick_place pose_sender_node --joints 0 -1.571 1.571 -1.571 -1.571 0
```

## Configuration

Edit `config/ur5_pick_place_config.yaml`:

```yaml
robot:
  name: ur5_with_rg2
  group_name: manipulator
  base_frame: world
  end_effector_frame: tool0

gripper:
  enabled: true
  profiles:
    open: {position: 0.0, effort: 0.0}
    medium_grasp: {position: 0.6, effort: 100.0}
    heavy_grasp: {position: 0.72, effort: 135.0}

named_positions:
  home: {joints: [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]}
```

## Command Reference

### Pick & Place
```bash
ros2 run ur5_pick_place pick_place_node
```
Executes full 8-stage pick and place sequence:
1. Open gripper
2. Move to home
3. Move to pre-pick
4. Move to grasp
5. Close gripper
6. Move to lift
7. Move to pre-place
8. Move to place
9. Open and retreat

### Pose Sender
```bash
# Move to named position
ros2 run ur5_pick_place pose_sender_node --position home
ros2 run ur5_pick_place pose_sender_node --position pre_pick

# Move to custom angles (radians)
ros2 run ur5_pick_place pose_sender_node --joints 0 -1.571 1.571 -1.571 -1.571 0
```

## Module Overview

### config_loader.py
- Loads and validates YAML configuration
- Provides named positions and gripper profiles
- Singleton pattern for global access

### gripper_controller.py
- Abstract gripper interface
- Simulated and real controller implementations
- Factory for automatic controller selection

### validators.py
- Joint angle validation against UR5 limits
- Workspace boundary checking
- Pre-flight safety validation

### base_pick_place.py
- Base class for all pick & place nodes
- Shared movement logic
- Stage execution tracking

### pick_place_node_v2.py
- Main 8-stage pick and place implementation
- Calls base class methods for each stage
- Error recovery with retries

### pose_sender_v2.py
- Utility for moving to specific positions
- Supports both named and custom positions
- CLI interface with argument parsing

## Testing

```bash
# Run unit tests
pytest test/test_validators.py -v

# Run specific test
pytest test/test_validators.py::TestJointValidator::test_valid_home_position -v
```

## Troubleshooting

**Configuration not found:**
```bash
export ROS_PACKAGE_PATH=/path/to/ur5_pick_place:$ROS_PACKAGE_PATH
```

**Module import error:**
```bash
colcon build --packages-select ur5_pick_place
source install/setup.bash
```

**Gripper not responding:**
- Check `/robotiq_85_gripper_controller/gripper_cmd` topic
- Verify gripper hardware connection
- Switch to simulated mode (automatic fallback)

## Logging

Set ROS logging level:
```bash
export RCL_LOG_LEVEL=DEBUG
ros2 run ur5_pick_place pick_place_node
```

## Dependencies

- rclpy (ROS2 Python client)
- PyYAML (configuration parser)
- pytest (testing framework)
- MoveIt2 (motion planning)
