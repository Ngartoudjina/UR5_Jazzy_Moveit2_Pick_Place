# UR5 Pick & Place v2.0 - Quick Command Reference

## Setup Commands

```bash
# Build package
colcon build --packages-select ur5_pick_place

# Source environment
source install/setup.bash

# View logs
export RCL_LOG_LEVEL=DEBUG
```

## Main Executables

### Pick & Place
```bash
# Full 8-stage sequence
ros2 run ur5_pick_place pick_place_node
```

Sequence:
```
[0/8] SETUP - Open gripper
[1/8] HOME - Return to safe position
[2/8] PRE_PICK - Hover above object
[3/8] GRASP - Move to object
[4/8] CLOSE - Close gripper
[5/8] LIFT - Lift object
[6/8] PRE_PLACE - Move to drop zone
[7/8] PLACE - Place object
[8/8] RELEASE - Open and retreat
```

### Pose Sender
```bash
# Move to named position
ros2 run ur5_pick_place pose_sender_node --position home
ros2 run ur5_pick_place pose_sender_node --position pre_pick
ros2 run ur5_pick_place pose_sender_node --position grasp
ros2 run ur5_pick_place pose_sender_node --position lift
ros2 run ur5_pick_place pose_sender_node --position pre_place
ros2 run ur5_pick_place pose_sender_node --position place
ros2 run ur5_pick_place pose_sender_node --position retreat

# Move to custom joint angles (in radians)
ros2 run ur5_pick_place pose_sender_node --joints 0 -1.571 1.571 -1.571 -1.571 0
```

## Testing Commands

```bash
# Run all tests
pytest test/test_validators.py -v

# Run specific test class
pytest test/test_validators.py::TestJointValidator -v
pytest test/test_validators.py::TestWorkspaceValidator -v
pytest test/test_validators.py::TestSafetyValidator -v

# Run specific test
pytest test/test_validators.py::TestJointValidator::test_valid_home_position -v

# Run with coverage
pytest test/test_validators.py --cov=ur5_pick_place --cov-report=html
```

## Configuration Commands

```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config/ur5_pick_place_config.yaml')); print('✓ Valid')"

# Display configuration
python3 << EOF
import yaml
with open('config/ur5_pick_place_config.yaml') as f:
    config = yaml.safe_load(f)
print("Loaded positions:", list(config['named_positions'].keys()))
print("Gripper profiles:", list(config['gripper']['profiles'].keys()))
EOF

# Edit configuration (use your favorite editor)
nano config/ur5_pick_place_config.yaml
gedit config/ur5_pick_place_config.yaml
code config/ur5_pick_place_config.yaml
```

## ROS2 Topic/Service Commands

```bash
# Check available nodes
ros2 node list

# Check available services
ros2 service list

# Check available topics
ros2 topic list

# Monitor gripper topic (if published)
ros2 topic echo /robotiq_85_gripper_controller/gripper_cmd

# Inspect action servers
ros2 action list
ros2 action info /move_action

# View node parameters
ros2 param list
```

## Logging & Debugging

```bash
# Run with debug logging
RCL_LOG_LEVEL=DEBUG ros2 run ur5_pick_place pick_place_node

# Run with info logging (default)
RCL_LOG_LEVEL=INFO ros2 run ur5_pick_place pick_place_node

# Pipe output to file
ros2 run ur5_pick_place pick_place_node > execution.log 2>&1

# Filter logs
ros2 run ur5_pick_place pick_place_node 2>&1 | grep "ERROR"
ros2 run ur5_pick_place pick_place_node 2>&1 | grep "STAGE"
```

## Python Package Commands

```bash
# Check package.xml validity
ament_lint_cmake src/ur5_pick_place

# Run linting
python3 -m pylint ur5_pick_place/*.py

# Type checking
mypy ur5_pick_place/*.py

# Code formatting
black ur5_pick_place/

# Format check
isort --check-only ur5_pick_place/
```

## Development Workflow

```bash
# 1. Make changes
nano ur5_pick_place/validators.py

# 2. Rebuild
colcon build --packages-select ur5_pick_place

# 3. Source
source install/setup.bash

# 4. Test
pytest test/test_validators.py -v

# 5. Run
ros2 run ur5_pick_place pick_place_node
```

## Launch File Commands

```bash
# Launch with RViz2
ros2 launch ur5_pick_place pick_place_v2.launch.py

# View launch file info
ros2 launch ur5_pick_place pick_place_v2.launch.py --show-args

# Show what will be launched
ros2 launch ur5_pick_place pick_place_v2.launch.py --show-launch-summary
```

## Package Management

```bash
# List all executables
ros2 pkg executables ur5_pick_place

# Show package prefix
ros2 pkg prefix ur5_pick_place

# Dump package metadata
ros2 pkg dump-manifest ur5_pick_place

# Check dependencies
ros2 pkg dependencies ur5_pick_place
```

## Common Workflows

### Test New Gripper Profile
```bash
# 1. Add to config/ur5_pick_place_config.yaml
gripper:
  profiles:
    custom_grip:
      position: 0.5
      effort: 80.0
      duration_sec: 1.8

# 2. Test it
python3 << EOF
from ur5_pick_place.config_loader import ConfigLoader
ConfigLoader.load_from_file('config/ur5_pick_place_config.yaml')
profile = ConfigLoader.get_gripper_profile('custom_grip')
print(f"Position: {profile.position}, Effort: {profile.effort}")
EOF
```

### Test New Joint Position
```bash
# 1. Add to config/ur5_pick_place_config.yaml
named_positions:
  my_position:
    joints: [0.1, -1.4, 1.6, -1.5, -1.5, 0.1]
    description: "Custom test position"

# 2. Move to it
ros2 run ur5_pick_place pose_sender_node --position my_position
```

### Verify Validation
```bash
python3 << EOF
from ur5_pick_place.validators import SafetyValidator
import math

# Test valid position
angles = [0.0, -math.pi/2, math.pi/2, -math.pi/2, -math.pi/2, 0.0]
result = SafetyValidator.pre_flight_check(angles)
print(f"Valid angles: {result.passed}")

# Test invalid position
bad_angles = [3*math.pi, 0, 0, 0, 0, 0]
result = SafetyValidator.pre_flight_check(bad_angles)
print(f"Invalid angles: {result.passed}")
EOF
```

## Troubleshooting

```bash
# Error: ModuleNotFoundError
# Solution: Rebuild and source
colcon build --packages-select ur5_pick_place
source install/setup.bash

# Error: Configuration file not found
# Solution: Create config directory
mkdir -p config

# Error: YAML parse error
# Solution: Validate syntax
python3 -c "import yaml; yaml.safe_load(open('config/ur5_pick_place_config.yaml'))"

# Error: Permission denied
# Solution: Make executable
chmod +x ur5_pick_place/pick_place_node_v2.py

# Error: Port already in use
# Solution: Kill existing process
pkill -f pick_place_node
```

## Performance Monitoring

```bash
# Monitor CPU usage
top -p $(pgrep -f "ros2 run")

# Monitor network traffic (ROS2)
ros2 topic hz /robotiq_85_gripper_controller/gripper_cmd

# Show message rate
ros2 topic bw /robotiq_85_gripper_controller/gripper_cmd

# Measure latency
ros2 topic pub --once /test_topic std_msgs/String "data: test"
```

## Useful Environment Variables

```bash
# Set ROS domain ID (for multi-robot systems)
export ROS_DOMAIN_ID=1

# Set logging level
export RCL_LOG_LEVEL=DEBUG

# Set Python path for imports
export PYTHONPATH="/path/to/ur5_pick_place:$PYTHONPATH"

# Disable colored output
export RCUTILS_COLORIZED_OUTPUT=0

# Enable detailed error messages
export ROS_LOG_DIR=/tmp/ros_logs
mkdir -p /tmp/ros_logs
```
