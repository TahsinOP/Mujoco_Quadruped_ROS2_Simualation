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

The model's initial position is a crouched stance as shown below.

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
ros2 launch b2_description mujoco_launch.py
```
The output should look something like this 

![image](https://github.com/user-attachments/assets/cc95a141-9d7e-41fd-81ac-a8b659692346)

> ✅ ROS2 communication bridge created between Mujoco and RViz successfully

## 5. PD Control for Holding Quadruped Target Joint Position

A proportional-derivative (PD) controller is implemented to hold each leg joint at its desired angle. This control is crucial for achieving a stable stance and movement during walking. An exmaple can be seen below where robot is held at the home position using appropriate actuator torques . 

To view this run : 
```
ros2 launch b2_description Mujoco_rviz_combined_launch.py
```

![image](https://github.com/user-attachments/assets/34ab8808-bd57-44f9-aca3-9a70f8a5bf86)

> ✅ PD controllers are tuned for joint stiffness and damping, holding poses accurately during testing.

---

## 6. IK Generation

Inverse Kinematics (IK) is implemented for each leg to compute joint angles given a desired foot position in 3D space. The IK solution ensures feasible leg configurations within joint limits, in this part also few challenges were faced in understanding the axes conventions for measuring the `hip`, `thigh`, and `knee` angles. Tried two methods one implmenting a 3D IK solution considering the hip_angle also , this gave decent results but had some error in angle conventions which needs some changes , To test this, run the `ik_3D.py` script . 

![image](https://github.com/user-attachments/assets/8faf2d45-6669-4dd9-bfb3-6dc77339541f)


Another alternative implemented was a 2D IK solution considering the thigh and knee angles in a plane giving an accurate result , to run this use `ik_2D.py` script in the `waypoint_navigation` package.



> ✅ A geometric IK solver is implemented per leg which  for joint angle calculations used in trotting 

---

## 7. Joint Trajectory Calculation and Visualization

Joint trajectories are generated and visualized in RViz for each waypoint or desired gait cycle. This forms the basis for smooth and continuous leg movement, all the points are then passed through the IK solver to get joint angles finally published to individual foot topics in a systematic order .

![image](https://github.com/user-attachments/assets/cba2cb24-7cc4-4d54-be4d-16ce389c0a22)


> ✅ Trajectory splines are generated and interpolated for joint-level control.

---

## 8. Trot Gait

A basic trotting gait is implemented for the quadruped, alternating diagonal legs (LF-RH and RF-LH) in motion. The gait parameters are fully adjustable:

- **Phase Offset**: Diagonal leg pairs (FL-RR and FR-RL) are offset by 0.5 phase to maintain stability
- **Stride Length**: Configurable step length (default 0.1m) for forward/backward motion
- **Stride Height**: Adjustable foot clearance height (default 0.1m) during swing phase
- **Cycle Time**: Complete gait cycle duration (default 1.0s)
- **Number of Points**: Trajectory discretization points (configurable, typically 8-10 points per cycle)
- **Gait Direction**: Configurable direction (forward/backward) with automatic trajectory adjustment

The gait trajectories are calculated for both forward and backward motion and published to the following leg topics:
- Forward motion: 
  - `/FL_joint_trajectory_forward`
  - `/FR_joint_trajectory_forward`
  - `/RL_joint_trajectory_forward`
  - `/RR_joint_trajectory_forward`
- Backward motion:
  - `/FL_joint_trajectory_backward`
  - `/FR_joint_trajectory_backward`
  - `/RL_joint_trajectory_backward`
  - `/RR_joint_trajectory_backward`

To visualize the trotting gait in the MuJoCo environment, run:
```bash
ros2 launch b2_description mujoco_trott_launch.py
ros2 run waypoint_navigation ik_2D.py
```

> ✅ Trot gait implementation with configurable parameters and bidirectional motion support.

## 9. Waypoint Navigation

The quadruped follows a sequence of waypoints published via RViz using spline interpolation and heading control. The navigation system:

1. **Path Generation**:
   - Waypoints are collected from RViz interactive markers
   - A smooth path is generated using cubic spline interpolation between waypoints
   - Path visualization is published for monitoring in RViz

2. **Navigation Control**:
   - Robot's current position is tracked using TF transforms
   - Heading angle is calculated using IMU yaw data
   - Target yaw is computed for each path segment
   - Hip angles are adjusted based on yaw error for turning
   - Distance threshold (0.1m) determines waypoint achievement

3. **Control Flow**:
   - Path following uses a state machine approach:
     1. Stabilize at current position
     2. Calculate target heading to next waypoint
     3. Adjust hip angles for turning
     4. Execute appropriate gait (forward/backward) to reach waypoint
     5. Repeat until final waypoint is reached

To run the waypoint navigation:
```bash
ros2 launch b2_description mujoco_trott_launch.py
ros2 run waypoint_navigation waypoint_navigation.py
```

The navigation process follows these interactive steps:
1. Press Enter to begin recording waypoints
2. Create waypoints in RViz using interactive markers
3. Press Enter again to complete waypoint generation
4. Navigation automatically begins following the recorded waypoints

> ✅ Complete waypoint navigation system with spline interpolation and heading control implemented.

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

