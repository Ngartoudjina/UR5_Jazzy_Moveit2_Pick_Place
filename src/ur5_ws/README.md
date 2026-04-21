# UR5 ROS 2 Jazzy Workspace  

This repository contains a **ROS 2 Jazzy workspace** for simulating and controlling a **UR5 robotic arm** using **Gazebo Sim** and **MoveIt 2**.  

---

## ✨ Features
- ROS 2 **Jazzy Jalisco**  
- UR5 arm description (URDF + meshes)  
- Gazebo simulation with physics & sensors  
- MoveIt 2 integration for motion planning  
- Example launch files for quick testing  

---

## 📂 Workspace Structure


---

## ⚡ Installation  

### 1. Install ROS 2 Jazzy  
Follow the official [ROS 2 installation guide](https://docs.ros.org/en/jazzy/Installation.html) for your OS.  

### 2. Install Dependencies  
Make sure you have `rosdep` and required Gazebo + MoveIt 2 packages:  
```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-moveit*
```

## 📥 Clone the Workspace
```bash
git clone https://github.com/nithishreddy1101/ur5_ws.git
```
## Build using colcon 
```bash
cd ~ur5_ws/
colon build
```

## Run the project
```bash
source install/setup.bash
ros2 launch ur5_moveit simulated_robot.launch.py
```

![UR5 Demo](src/ur5_ws.png)

This repository contains a **ROS 2 Jazzy workspace** for simulating and controlling a **UR5 robotic arm** using **Gazebo Sim** and **MoveIt 2**.  
