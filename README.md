# UR5 MoveIt2 Jazzy Pick and Place

ROS 2 Jazzy and MoveIt 2 based project for simulating and controlling a UR5 robotic arm equipped with a Robotiq 85 gripper and pick-and-place capabilities.

The project includes:

* Full UR5 robot description in URDF/Xacro
* Gazebo simulation environment
* MoveIt 2 configuration for motion planning
* Pick-and-place Python nodes
* RViz visualization
* ROS 2 control integration

## Features

* UR5 robotic arm model with Robotiq 85 gripper
* Pick-and-place motion planning with MoveIt 2
* Gazebo world and simulated robot
* RViz interactive visualization
* ROS 2 Jazzy + ros2_control integration
* OMPL and Pilz planners support
* Custom object spawning and pose publishing

## Repository Structure

```text
ros2_jazzy_ws/
├── src/
│   ├── ur5_pick_place/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── ur5_pick_place/
│   │       ├── add_object.py
│   │       ├── fake_object_pose.py
│   │       └── pose_sender.py
│   │
│   └── ur5_ws/
│       └── src/
│           ├── hello_moveit/
│           │   └── src/hello_moveit.cpp
│           │
│           ├── ur5_controller/
│           │   ├── config/ros2_controllers.yaml
│           │   └── launch/controller.launch.py
│           │
│           ├── ur5_description/
│           │   ├── launch/
│           │   │   ├── gazebo.launch.py
│           │   │   └── rviz.launch.py
│           │   ├── urdf/
│           │   │   ├── ur5_robot.urdf.xacro
│           │   │   ├── ur5_robot.gazebo.xacro
│           │   │   └── ur5_robot.ros2_control.xacro
│           │   ├── meshes/
│           │   └── worlds/gazebo_world.sdf
│           │
│           └── ur5_moveit/
│               ├── config/
│               │   ├── kinematics.yaml
│               │   ├── ompl_planning.yaml
│               │   ├── moveit_controllers.yaml
│               │   └── ur5_robot.srdf
│               └── launch/
│                   ├── moveit.launch.py
│                   └── simulated_robot.launch.py
│
├── build/
├── install/
└── log/
```

## Main Packages

### `ur5_description`

Contains the UR5 robot model, meshes, Xacro/URDF files, Gazebo configuration and RViz setup.

Important files:

* `urdf/ur5_robot.urdf.xacro`
* `urdf/ur5_robot.gazebo.xacro`
* `urdf/ur5_robot.ros2_control.xacro`
* `launch/gazebo.launch.py`
* `launch/rviz.launch.py`

### `ur5_moveit`

Contains the complete MoveIt 2 configuration for the UR5 robot.

Important files:

* `config/kinematics.yaml`
* `config/joint_limits.yaml`
* `config/ompl_planning.yaml`
* `config/moveit_controllers.yaml`
* `config/ur5_robot.srdf`
* `launch/moveit.launch.py`
* `launch/simulated_robot.launch.py`

### `ur5_pick_place`

Python package containing the pick-and-place utilities.

Nodes:

* `add_object.py` – adds an object to the planning scene
* `fake_object_pose.py` – publishes a fake object pose
* `pose_sender.py` – sends target poses to the robot

## Requirements

* Ubuntu 24.04
* ROS 2 Jazzy Jalisco
* MoveIt 2
* Gazebo Harmonic
* `ros2_control`
* `colcon`
* Universal Robots dependencies

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_jazzy_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the Simulation

Launch the UR5 robot in Gazebo:

```bash
ros2 launch ur5_description gazebo.launch.py
```

Launch MoveIt 2:

```bash
ros2 launch ur5_moveit moveit.launch.py
```

Or launch the full simulated robot configuration:

```bash
ros2 launch ur5_moveit simulated_robot.launch.py
```

## Run Pick and Place Nodes

Spawn an object into the planning scene:

```bash
ros2 run ur5_pick_place add_object
```

Publish a fake object pose:

```bash
ros2 run ur5_pick_place fake_object_pose
```

Send a pose target:

```bash
ros2 run ur5_pick_place pose_sender
```

## Motion Planning

The project supports:

* Joint-space planning
* Cartesian path planning
* Pick-and-place trajectories
* OMPL planners
* Pilz industrial motion planner

## Future Improvements

* Real UR5 hardware integration
* Vision-based object detection
* Camera integration
* Multi-object pick and place
* Better grasp planning
* Chess-playing robotic arm integration

## Screenshot

Add a project screenshot here, for example:

```text
src/ur5_ws/sr
```
