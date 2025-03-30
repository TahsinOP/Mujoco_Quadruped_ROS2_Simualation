# 🐾 Robot Dog Simulation using MuJoCo and ROS 2

## Introduction

This project focuses on developing a simulated quadruped robot (robot dog) in the **MuJoCo** physics engine, integrated with **ROS 2 Humble** for visualization and waypoint navigation. The goal is to build a system capable of:

- Navigating from **point A to point B** using waypoints selected from **RViz**
- Detecting obstacles in the environment
- (Advanced) Performing **dynamic jumping** maneuvers onto and off a box

This simulation tests our understanding of **robot kinematics**, **perception**, **motion planning**, and **control** in a physics-based simulation environment.

The project uses an **open-source quadruped robot model** (e.g., MIT Mini Cheetah or Stanford Doggo), with the intent to implement a **trot gait**, inverse kinematics, and stable locomotion. The system is modular and extensible, allowing future extensions such as obstacle avoidance and jumping behavior.

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

### 1. RViz Quadruped Visualization

RViz is used to visualize the robot's pose, joint states, and trajectory within a simulated environment. The URDF of the quadruped robot is loaded into RViz, enabling real-time feedback during navigation.

> ✅ The robot model loads correctly in RViz, showing joint frames and interactive visualization.

---

### 2. Waypoint Generation using Interactive Markers

Interactive markers in RViz allow the user to place navigation waypoints for the quadruped. These are published as `geometry_msgs::Pose` messages and subscribed by the navigation planner.

> ✅ A simple interface to generate waypoints has been implemented and verified in RViz.

---

### 3. MuJoCo Quadruped Setup

MuJoCo is configured as the core physics simulator for the quadruped. The model (based on MIT Mini Cheetah or Stanford Doggo) includes all kinematic chains and joint constraints necessary for accurate simulation.

> ✅ The robot loads and simulates correctly in MuJoCo with controllable joints.

---

### 4. RViz-MuJoCo ROS 2 Bridge

A custom ROS 2 bridge node synchronizes joint positions and sensor data between MuJoCo and RViz. This allows RViz to reflect the robot's real-time state as simulated in MuJoCo.

> ✅ A ROS 2 node bridges joint states and commands between MuJoCo and the visualization stack.

---

### 5. PD Control for Holding Quadruped Target Joint Position

A proportional-derivative (PD) controller is implemented to hold each leg joint at its desired angle. This control is crucial for achieving stable stance and movement during walking.

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

> ✅ Initial trot gait is implemented and shows stable motion under simulation.

---

### 9. Waypoint Navigation

The quadruped follows a sequence of waypoints published via RViz. The planner computes body trajectories and corresponding joint commands to move the robot from point A to point B.

> ✅ The robot starts walking toward selected waypoints using the trot gait and joint trajectories.

---

## 📁 Repository Structure 
## 🎥 Demonstration Video (Optional)

---

## 🚧 Progress Summary

- ✅ **70% of Part 1** completed (visualization, IK, gait, and navigation tested)
- 🔜 Next steps: Add obstacle detection (Part 2) and jumping maneuvers (Part 3)
- 🛠️ Current challenge: Ensuring real-time stability and tuning gait parameters in MuJoCo

