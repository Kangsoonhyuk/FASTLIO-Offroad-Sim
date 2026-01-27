# FASTLIO-Offroad-Sim

A custom Clearpath Husky simulation environment designed for testing FAST-LIO in off-road terrains. This repository includes a modified inspection world with dense vegetation and custom physics settings for rough terrain simulation.

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04 recommended)
- **Gazebo Ignition Fortress**

## Installation

1. **Clone the repository:**
   ```bash
   mkdir -p ~/clearpath_ws
   cd ~/clearpath_ws
   git clone https://github.com/Kangsoonhyuk/FASTLIO-Offroad-Sim.git .
   ```

2. **Install dependencies:**
   ```bash
   # Import dependencies from .repos file
   vcs import src < dependencies.repos 

   # Install ROS dependencies
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Launching the Simulation
This launches the detailed inspection world with the customized Husky robot (LiDAR + Camera + High Friction Wheels).

```bash
ros2 launch setup/launch_inspection.launch.py
```

### Robot Configuration
The robot configuration is defined in `setup/robot.yaml`. It mimics a Clearpath Husky A200 with:
- Velodyne VLP-16 LiDAR
- Intel RealSense D435 Camera
- Custom wheel slip parameters for high traction

### World Modification
The simulation world is located at `src/cpr_gazebo/cpr_inspection_gazebo/worlds/inspection.sdf`.

## Troubleshooting
- If the robot slips too much, check `src/clearpath_common/clearpath_platform_description/urdf/a200/a200.urdf.xacro` and ensure `slip_compliance` is set to `0.0`.
