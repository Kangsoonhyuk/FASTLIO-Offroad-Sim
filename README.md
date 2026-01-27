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
   mkdir -p ~/clearpath_ws/src
   cd ~/clearpath_ws
   git clone https://github.com/Kangsoonhyuk/FASTLIO-Offroad-Sim.git .
   ```

2. **Import Dependencies & Build:**
   ```bash
   vcs import src < dependencies.repos
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage
### Launching with FAST-LIO
To launch the full simulation including Gazebo, the robot, and the FAST-LIO mapping stack:
```bash
# This launch file triggers the robot spawn and initializes FAST-LIO + RViz
ros2 launch setup/simulation_with_fastlio.launch.py
```

### Scenarios
Select your terrain by passing the `scenario` argument:
- `test`: Inspection World with trees and slopes.
- `base`: Obstacle-free flat terrain for baseline testing.

## Troubleshooting

### Symptom
FAST-LIO logs "Leaf size is too small" and "Integer indices overflow," causing mapping to fail immediately.

### The Lesson: Calibration & Initialization Stability
The root cause was the **initial robot spawn condition**. Spawning the robot too high above the ground causes it to drop and impact the terrain upon start.

- **The Impact**: This sudden physical jar creates a massive spike in the IMU data at the exact moment FAST-LIO is attempting to initialize its state and extrinsic calibration.
- **The Result**: The "exploding" coordinates are a mathematical side-effect of the EKF diverging due to this initial shock. 
- **The Solution**: **Ensuring the robot is spawned flush with the ground** is critical. A stable, impact-free start ensures a "clean" IMU signal, allowing the filters to converge correctly without the need for external data pre-processing.


---

## Acknowledgments
This project makes use of the following open-source packages:

- **[Clearpath Robotics](https://github.com/clearpathrobotics)**: Husky platform, simulation environment, and configuration tools.
- **[Velodyne Simulator](https://github.com/ToyToyota/velodyne_simulator)**: Simulation plugins for Velodyne LiDAR.
- **[FAST_LIO](https://github.com/hku-mars/FAST_LIO)**: Fast-LIO2 framework for mapping.