# FASTLIO-Offroad-Sim

## Overview
A custom Clearpath Husky simulation environment designed for testing FAST-LIO in off-road terrains. 
**Note**: The "Inspection World" included in this repository (`src/cpr_gazebo`) is originally a ROS 1 asset converted for use in ROS 2 (Ignition Gazebo). We have customized it with additional vegetation and physics tuning.

## Setup
### Prerequisites
**This setup is designed to work on any machine meeting these requirements:**
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill (Desktop Install)
- **Simulator**: Gazebo Ignition Fortress
- **Tools**: `ros-dev-tools` (colcon, rosdep, vcs)

*If these prerequisites are met, the steps below will create a complete, isolated workspace.*

### Installation
1. **Clone this repository:**
   ```bash
   mkdir -p ~/clearpath_ws/src
   cd ~/clearpath_ws
   git clone https://github.com/Kangsoonhyuk/FASTLIO-Offroad-Sim.git .
   ```

2. **Import Dependencies:**
   This project relies on standard Clearpath packages.
   ```bash
   vcs import src < dependencies.repos
   ```

3. **Install ROS Dependencies:**
   ```bash
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Workspace:**
   ```bash
   colcon build --symlink-install
   ```

5. **Source the Workspace:**
   ```bash
   source install/setup.bash
   ```

## Launch
### Launching the Simulation
This launches the detailed inspection world with the customized Husky robot.
```bash
ros2 launch setup/launch_inspection.launch.py
```

### Robot Configuration & Topics **(IMPORTANT)**
The robot is configured in `setup/robot.yaml` with the following specs:
- **Robot Model**: Clearpath Husky A200
- **Sensors**: 
  - **LiDAR**: Velodyne VLP-16 (3D LiDAR) mounted on a sensor arch.
  - **Camera**: Intel RealSense D435 (RGB-D) mounted on the front bumper.
- **Hostname**: `cpr-a200-0000`
- **Serial Number**: `a200-0000`

**Topic Namespacing**:
Because the `serial_number` is set to `a200-0000`, all ROS 2 topics will be namespaced accordingly. To control the robot or view sensor data, you MUST use the correct topic names:
- **Lidar**: `/a200_0000/sensors/lidar3d_0/points`
- **Camera**: `/a200_0000/sensors/camera_0/...`
- **Cmd Vel**: `/a200_0000/cmd_vel`
- **Odometry**: `/a200_0000/odometry`

*Check `setup/robot.yaml` if you wish to change the hostname or serial number.*

### World Modification
The simulation world is located at `src/cpr_gazebo/cpr_inspection_gazebo/worlds/inspection.sdf`.

## Scenarios
This simulation is designed to test three main scenarios for off-road autonomous navigation:

1. **Off-road Autonomous Navigation**: 
   - The Husky robot navigates towards a goal in a rough terrain environment (Inspection World) filled with slopes and dense trees.

2. **Performance Comparison**:
   - **Case 2.1 (Baseline)**: Navigation in an obstacle-free, flat terrain.
   - **Case 2.2 (Test Case)**: Navigation in the "Inspection World" with obstacles (trees) and significant slopes.
   
   The goal is to compare the robustness and efficiency of the navigation stack (FAST-LIO + Nav2) between these conditions.

## Demo Video
*(Coming Soon)*

## Acknowledgments
This project makes use of the following open-source packages:

- **[clearpath_simulator](https://github.com/clearpathrobotics/clearpath_simulator)**: Core simulation packages for Clearpath robots.
- **[clearpath_config](https://github.com/clearpathrobotics/clearpath_config)**: Configuration system for Clearpath robots.
- **[clearpath_common](https://github.com/clearpathrobotics/clearpath_common)**: Common URDFs and messages.
- **[cpr_gazebo](https://github.com/clearpathrobotics/cpr_gazebo)**: Gazebo world environments (Inspection World).
- **[velodyne_simulator](https://github.com/ToyToyota/velodyne_simulator)**: Simulation plugins for Velodyne LiDAR.
- **[realsense_ros](https://github.com/IntelRealSense/realsense-ros)**: ROS 2 wrapper for Intel RealSense cameras.