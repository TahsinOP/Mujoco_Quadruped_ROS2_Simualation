# 🐾 Robot Dog Simulation using MuJoCo and ROS 2

## Introduction

This task focuses on developing a simulated quadruped robot (robot dog) in the **MuJoCo** physics engine, integrated with **ROS 2 Humble** for visualization and waypoint navigation. The goal is to build a system capable of:

- Navigating from **point A to point B** using waypoints selected from **RViz**
- Detecting obstacles in the environment
- (Advanced) Performing **dynamic jumping** maneuvers onto and off a box

The project uses an **open-source quadruped robot model** (Unitree B2) to implement a **trot gait** based waypoint navigation, inverse kinematics, and stable locomotion. The system is modular and extensible, allowing future extensions such as obstacle avoidance and jumping behavior. The content is organized in shown below manner starting from model visualization in (RViz) and Mujuco, implementing a ROS2 Bridge to waypoint navigation 

---

## 📑 Table of Contents

1. [RViz Quadruped Visualization](#1-rviz-quadruped-visualization)  
2. [Waypoint Generation using Interactive Markers](#2-waypoint-generation-using-interactive-markers)  
3. [MuJoCo Quadruped Setup](#3-mujoco-quadruped-setup)  
4. [RViz-MuJoCo ROS 2 Bridge](#4-rviz-mujoco-ros-2-bridge)  
5. [PD Control for Holding Quadruped Target Joint Position](#5-pd-control-for-holding-quadruped-target-joint-position)  
6. [IK Generation](#6-ik-generation)  
7. [Joint Trajectory Calculation and Visualization](#7-joint-trajectory-calculation-and-visualization)  
8. [Trot Gait](#8-trot-gait)  
9. [Waypoint Navigation](#9-waypoint-navigation)  

---

## 1. RViz Quadruped Visualization
The Unitree B2 model is chosen due to its well-defined description packages ([link](https://github.com/unitreerobotics/unitree_ros2)). The URDF is loaded in RViz by launching the `robot_state_publisher` node using the simple launch file `robot_display_launch.py`.

To view the robot in RViz, run:

```bash
ros2 launch b2_description robot_display.launch.py
```

The model’s initial position is a crouched stance as shown below.

![image](https://github.com/user-attachments/assets/0974367f-7df5-4734-b893-5c5281213fbf)


> ✅ The robot model in RViz, showing the robot model, transforms (TFs), and joint states.

---

## 2. Waypoint Generation using Interactive Markers

Created a script `interactive_waypoint_publisher.py` that initiates a circular interactive marker server in RViz. On receiving feedback, it stores the waypoints (x, y) created by the user and publishes them to the `/waypoint` topic.

To start recording waypoints and publish the waypoints, first run the launch file command to load the URDF in RViz as mentioned in the above section, then run the script as shown below 
```
ros2 run b2_description interactive_waypoint_publisher.py
```
Drag the marker using the mouse according to the desired path and stop when completed; all the waypoints will be published 

[Screencast from 04-03-2025 10:54:27 PM.webm](https://github.com/user-attachments/assets/1f1a5921-39fb-4b58-9f50-c1b1dad1fd67)

Key challenge: creating multiple markers one after another in an orderly manner.

> ✅ A simple interface to generate and store the waypoints has been implemented and verified in RViz.

---

## 3. MuJoCo Quadruped Setup

Using an XML file of the quadruped robot defined in a world, spawned it, started the simulation using the MuJoCo-Viewer Python binding, published joint position, velocity, and effort on the `/mujoco_joint_states` topic . To spawn the robot in Mujoco, run this command
```
ros2 run b2_description mujoco_simulation.py
```
The simulation environment should look something like this 

![image](https://github.com/user-attachments/assets/d3895a63-2226-4ff3-a691-d59df077d5f0)

> ✅ The quadruped robot spawns correctly in MuJoCo using the xml file 

## 4. RViz-MuJoCo ROS 2 Bridge

This was a challenging part — went through various repositories for combining ROS 2 communication with MuJoCo and bridging it with RViz. Thanks to this [repo]() for giving a brief idea of how to extract joint and actuator data in Mujoco , publish `world` to `base` frame transforms (TFs) and `joint_states`, which RViz uses for real-time visualization.

To spawn the robot in Mujoco, load RViz with panels configured run the below command 

```bash
ros2 launch b2_description mujoco_rviz_combined_launch.py
```
The output should look something like this 

![image](https://github.com/user-attachments/assets/cc95a141-9d7e-41fd-81ac-a8b659692346)

> ✅ ROS2 communication bridge created between Mujoco and RViz successfully

### 5. PD Control for Holding Quadruped Target Joint Position

A proportional-derivative (PD) controller is implemented to hold each leg joint at its desired angle. This control is crucial for achieving a stable stance and movement during walking. An exmaple can be seen below where robot is held at the home position using appropriate actuator torques 

![image](https://github.com/user-attachments/assets/34ab8808-bd57-44f9-aca3-9a70f8a5bf86)

> ✅ PD controllers are tuned for joint stiffness and damping, holding poses accurately during testing.

---

### 6. IK Generation

Inverse Kinematics (IK) is implemented for each leg to compute joint angles given a desired foot position in 3D space. The IK solution ensures feasible leg configurations within joint limits.

> ✅ A geometric IK solver is implemented per leg, and tested for static poses.

---

### 7. Joint Trajectory Calculation and Visualization

For each waypoint or desired gait cycle, joint trajectories are generated and visualized in RViz. This forms the basis for smooth and continuous movement of the legs.

> ✅ Trajectory splines are generated and interpolated for joint-level control.

---

### 8. Trot Gait

A basic trotting gait is implemented for the quadruped, alternating diagonal legs (LF-RH and RF-LH) in motion. Gait parameters such as duty cycle and phase offset are adjustable.


---

### 9. Waypoint Navigation

The quadruped follows a sequence of waypoints published via RViz. The planner computes body trajectories and corresponding joint commands to move the robot from point A to point B

---

## 📁 Repository Structure 

```
📦 robot_dog_simulation
 ┣ 📂 b2_description
 ┃ ┣ 📜 CMakeLists.txt
 ┃ ┣ 📜 package.xml
 ┃ ┣ 📂 urdf
 ┃ ┃ ┣ 📜 b2.urdf.xacro
 ┃ ┣ 📂 launch
 ┃ ┃ ┣ 📜 robot_display_launch.py
 ┃ ┃ ┣ 📜 mujoco_rviz_combined_launch.py
 ┃ ┣ 📂 scripts
 ┃ ┃ ┣ 📜 interactive_waypoint_publisher.py
 ┣ 📂 mujoco_sim
 ┃ ┣ 📜 mujoco_viewer.py
 ┃ ┣ 📜 simulation.xml
 ┣ 📜 README.md
 ┗ 📜 .gitignore
```

## 🎥 Demonstration Video 

---

## 🚧 Progress Summary

- ✅ **70% of Part 1** completed (visualization, IK, gait trajectory generation, trotting gait completed --- waypoint navigation left)
- 🔜 Next steps: Add obstacle detection (Part 2) and jumping maneuvers (Part 3)
- 🛠️ Current challenge: Ensuring real-time stability and tuning gait parameters in MuJoCo

