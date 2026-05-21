# matlab_code

A comprehensive collection of MATLAB projects developed by the GRAAL Lab, focused on robotics, control, and motion planning.

## Overview

This repository contains several specialized MATLAB projects for robotic systems, task-based inverse kinematics, and utility tools for mathematical computations and robot modeling.

## Directory Structure

### 📁 **dextimus** - Underwater Manipulator Simulation
Simulation framework for an Underwater Vehicle-Manipulator System (UVMS). This project includes:
- **DextimusMain.m**: Main entry point for the simulation
- **UvmsModel.m** & **UvmsSim.m**: UVMS dynamics and simulation engine
- **armModel.m** & **arm.urdf**: Robotic arm kinematics and URDF model definition
- **UnityInterface.m**: Integration with Unity game engine for visualization
- **Task classes**: Task-specific implementations including:
  - `TaskHorizontalAttitude.m`: Horizontal attitude maintenance tasks
  - `TaskVehicleOrientation.m`: Vehicle orientation control
  - `TaskTool.m` & `TaskVehicle.m`: Tool and vehicle manipulation tasks
- **SimulationLogger.m**: Logging and data recording utilities
- **Helper utilities**: `manipulator.m`, `floatingBaseHelper.m`, `robot_urdf.m`, `draft.m`

### 📁 **icat** - Task-based Inverse Kinematics
Implementation of the incremental Complementary Augmented Tasks (iCAT) control framework:
- **iCAT_task.m**: Main task definition and management
- **iCAT_pseudoInverse.m**: Pseudo-inverse computation for iCAT
- **RegPseudoInverse.m**: Regularized pseudo-inverse implementation

This module handles hierarchical task-based control for robot manipulators.

### 📁 **tools** - Utility Functions & Mathematical Tools
A collection of helper functions for robotics and control applications:

**Transformation & Rotation Tools:**
- `RotMatrix2RPY.m`: Convert rotation matrices to Roll-Pitch-Yaw angles
- `pqr2rpy.m`: Convert angular rates (p,q,r) to RPY rates
- `skew.m`: Skew-symmetric matrix generation
- `rotation.m`: Rotation matrix utilities

**Task & Control Tools:**
- `CartError.m`: Cartesian error computation
- `Saturate.m`: Signal saturation for control limits
- `integrate_vehicle.m`: Vehicle integration utilities

**Mathematical Functions:**
- `VersorLemma.m`: Versor lemma implementation
- `ReducedVersorLemma.m`: Reduced versor lemma for optimization
- `IncreasingBellShapedFunction.m`: Smooth increasing transition function
- `DecreasingBellShapedFunction.m`: Smooth decreasing transition function
- `SlowdownToRealtime.m`: Real-time simulation adaptation

### 📁 **tpik** - Task Priority Inverse Kinematics
Framework for task-priority based inverse kinematics:
- **ActionManager.m**: Manages prioritized actions for the robot
- **Task.m**: Base task definition

## Getting Started

### Prerequisites
- MATLAB R2019b or later
- Robotics System Toolbox (for URDF support in dextimus)
- Optional: Unity 2020+ (for visualization with Dextimus)

### Usage

1. **Clone the repository:**
   ```bash
   git clone https://github.com/GRAAL-Lab/matlab_code.git
   cd matlab_code
