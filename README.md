# Vision-Based Robotic Pick-and-Place System

### Autonomous Manipulation using RGB-D Perception and ROS2

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-green)](https://python.org)
[![MoveIt2](https://img.shields.io/badge/MoveIt2-Latest-orange)](https://moveit.ros.org)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-red)](https://gazebosim.org)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 🎯 Overview

A complete autonomous robotic manipulation system that uses
RGB-D vision to detect, localize, and manipulate objects
without human intervention.

## 📊 Key Results

- ✅ 100% task success rate (10/10 trials)
- ✅ 1.9mm placement accuracy
- ✅ 52s average execution time
- ✅ 98% IK success rate across workspace

## 🛠️ Tech Stack

- ROS2 Humble
- MoveIt2 (motion planning)
- OpenCV (computer vision)
- Gazebo (simulation)
- Python 3.10
- TF2 (coordinate transforms)

## 🤖 System Capabilities

- RGB-D object detection and 3D localization
- Inverse kinematics with collision avoidance
- Safe waypoint-based motion planning
- Gripper geometry compensation
- Robust error handling and recovery

## 📁 Repository Structure

ur5e_golf_pick_place/
├── ur5e_golf_pick_place/
│   ├── pick_place_exercise/
│   │   └── vision_based_pick_and_place.py
├── launch/
│   └── golf_pick_place.launch.py
├── config/
│   ├── ur5e_controllers.yaml
│   └── moveit_config.yaml
├── worlds/
│   └── golf_pick_place.world
├── urdf/
│   └── ur5e_robotiq85.urdf.xacro
└── README.md

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS
- ROS2 Humble
- MoveIt2
- Gazebo 11

### Installation

# Clone repository

git clone https://github.com/nelsonj97/ur5e_pick_place.git
cd ur5e_pick_place

# Install dependencies

rosdep install --from-paths src --ignore-src -r -y

# Build

colcon build --symlink-install
source install/setup.bash

### Run

# Terminal 1: Launch simulation

ros2 launch ur5e_golf_pick_place golf_pick_place.launch.py

# Terminal 2: Run pick-and-place

ros2 run ur5e_golf_pick_place vision_based_pick_and_place

## 🏗️ System Architecture

┌─────────────────────────────────────┐
│         Application Layer                                                           │
│   vision_based_pick_and_place.py                                       │
└─────────────────┬───────────────────┘
                  │
    ┌─────────────┼─────────────┐
    ▼                            ▼             ▼
    VisionSystem      RobotArm    RobotGripper
    (OpenCV/TF2)     (MoveIt2/IK) (Controllers)
    │             │             │
    └─────────────┼─────────────┘
                  ▼
            ROS2 Humble
    (Topics/Services/Actions)
                  │
    ┌─────────────┼─────────────┐
    ▼             ▼             ▼
  Gazebo       MoveIt2     Controllers

## 📈 Performance Results

| Metric              | Result         |
| ------------------- | -------------- |
| Task Success Rate   | 100% (10/10)   |
| Placement Accuracy  | 1.9mm ± 0.8mm |
| IK Success Rate     | 98%            |
| IK Computation Time | 14ms avg       |
| Total Execution     | 52.3s ± 0.5s  |

## 🔬 Technical Highlights

### Vision Pipeline

* HSV color segmentation for object detection
* Multi-modal 3D localization (depth + point cloud)
* TF2-based coordinate transformation
* Real-time processing (12-16ms per frame)

### Motion Planning

* MoveIt2 IK with KDL solver
* Safe waypoint-based trajectory planning
* Explicit gripper geometry compensation
* Collision-aware planning scene management

### System Robustness

* Three-level error classification
* Exponential backoff retry mechanism
* Thread-safe data management
* Graceful failure recovery

## 📚 Documentation

* [Thesis Report]()
* [Algorithm Documentation]()
* [Setup Guide]()
* [Troubleshooting]()

## 🎓 Academic Context

This project was developed as part of a thesis in Computer engineering at University of Applied Science Koblenz, Germany, demonstrating practical application of robotics, computer vision, and motion planning.

## 📧 Contact

* Email: nelsonjorvany@gmail.com
* LinkedIn: linkedin.com/in/yourprofile
