# FASTLIO-Offroad-Sim

## Overview
This repository provides a custom **Clearpath Husky** simulation environment tailored for testing research on **FAST-LIO** in challenging off-road terrains. 

The environment is based on the modern **Clearpath ROS 2 Pipeline**, where we have adapted the latest **Jazzy-based clearpath_simulator** architecture to be fully compatible with **ROS 2 Humble**. This allows us to use the latest simulation features in a stable LTS environment.

### Development Background
- **Environment Base**: Built following the [Official Clearpath ROS 2 Humble Installation Guide](https://docs.clearpathrobotics.com/docs/ros2humble/ros/tutorials/simulator/install/).
- **Pipeline Migration**: While the official pipeline is moving towards ROS 2 Jazzy, we have backported and adapted the [clearpath_simulator (Jazzy branch)](https://github.com/clearpathrobotics/clearpath_simulator) architecture to work seamlessly on **ROS 2 Humble**.
- **Integration**: Combines the robust CPR Husky platform with a Velodyne VLP-16 LiDAR and an IMU to support **FAST-LIO** SLAM.

## Setup
### Prerequisites
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble (Desktop Install)
- **Simulator**: Gazebo Ignition Fortress

### Installation
1. **Clone and Setup Workspace:**
   ```bash
   mkdir -p ~/your_ws/src
   cd ~/your_ws
   git clone https://github.com/Kangsoonhyuk/FASTLIO-Offroad-Sim.git .
   ```

2. **Import Dependencies & Build:**
   ```bash
   # Import required repositories (Clearpath, Velodyne, FAST-LIO, etc.)
   vcs import src < dependencies.repos
   
   # Install system dependencies (including PCL and ROS packages)
   rosdep install --from-paths src --ignore-src -r -y
   
   # Build the workspace
   colcon build --symlink-install
   
   # Source the setup script
   source install/setup.bash
   ```

## Usage
### Launching the Simulation

- **Scenario 1 & 2.1 (Baseline)**:
  ```bash
  ros2 launch fast_lio fastlio_sim.launch.py scenario:=base
  ```

- **Scenario 2.2 (Test Case)**:
  ```bash
  ros2 launch fast_lio fastlio_sim.launch.py scenario:=test
  ```

### Robot Configuration & Teleoperation
This simulation uses a standard **Clearpath Husky A200** equipped with:
- **LiDAR**: Velodyne VLP-16
- **Camera**: Intel RealSense D435

**Note on Teleoperation**: 
All robot topics are namespaced. When using the **Gazebo Teleop plugin**, you **MUST** change the topic from `/cmd_vel` to:
> **/(your_hostname)/cmd_vel**

### World Modification
The simulation world is located at `src/clearpath_simulator/clearpath_gz/worlds/pipeline.sdf`.

## Scenarios
This simulation is designed to test three main scenarios for off-road autonomous navigation:

1. **Off-road Autonomous Navigation**: 
   - The Husky robot navigates towards a goal in a rough terrain environment (Inspection World) filled with slopes and dense trees.
     <br/><img src="doc/img/scenario_offroad.png" width="60%" />

2. **Performance Comparison**:
   - **Case 2.1 (Baseline)**: Navigation in an obstacle-free, flat terrain.
     <br/><img src="doc/img/scenario_baseline.jpg" width="60%" />
   - **Case 2.2 (Test Case)**: Navigation in the "Inspection World" with obstacles (trees) and significant slopes.
     <br/><img src="doc/img/scenario_test.jpg" width="60%" />
   
   The goal is to compare the robustness and efficiency of the navigation stack (FAST-LIO + Nav2) between these conditions.

## Demo Video

## Troubleshooting

- **Error**: FAST-LIO logs `Leaf size is too small` and `Integer indices overflow`, causing immediate failure.
  - **Cause**: **IMU Initialization Failure** due to motion or impact. FAST-LIO assumes the robot is **stationary** for the first few frames (see `IMU_Processing.hpp`) to calculate gravity and bias. If the robot drops (even 0.2cm) during this phase, the massive acceleration spike distorts this calculation, causing the EKF to diverge and the position to shoot to infinity.
  - **Solution**: Ensure the robot is **completely stationary** and the simulation is stable before launching FAST-LIO. Do not launch FAST-LIO while the robot is still settling or moving after spawn.

- **Error**: FAST-LIO logs `Failed to find match for field 'time'.`
  - **Cause**: Gazebo Fortress often optimizes out the `time` field from point cloud data, but FAST-LIO's `Velodyne (2)` mode requires this field for deskewing.
  - **Solution**: Set `preprocess.lidar_type` to **5 (Generic)** to bypass hardware-specific field checks.

- **Error**: FAST-LIO warning `[laser_mapping]: No point, skip this scan!`
  - **Cause**: Transient simulation artifacts (e.g., raycasting hitting sky or NaNs during rapid movement) result in frames with zero valid points after filtering.
  - **Solution**: **Safe to ignore**. This is a common simulation quirk and does not affect mapping if the robot pose continues to update correctly.


---

## Acknowledgments
This project makes use of the following open-source packages:

- **[Clearpath Robotics](https://github.com/clearpathrobotics)**: Husky platform, simulation environment, and configuration tools.
- **[Velodyne Simulator](https://github.com/ToyToyota/velodyne_simulator)**: Simulation plugins for Velodyne LiDAR.
- **[FAST_LIO](https://github.com/hku-mars/FAST_LIO)**: Fast-LIO2 framework for SLAM.