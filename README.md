# Teleop Slave Package

This ROS 2 (Humble) package enables real-time teleoperation of a **Fairino FR5 manipulator** and a **Tesollo Delto M Gripper** using a Manus VR glove as the master device. It features a continuous joint state streaming architecture and uses **NVIDIA cuRobo** for GPU-accelerated, collision-free motion generation (MPPI solver).

## Features
- **Real-Time Teleoperation Pipeline**: Receives end-effector poses from a master device (e.g., Manus VR glove) via `master_bridge_node` and maps it to target end-effector coordinates.
- **Continuous Joint Streaming**: A custom streaming mode (`ServoJ`) in the `fairino_lowlevel_controller_node` executing at 250Hz.
- **NVIDIA cuRobo MPPI Solver**: High-performance, CUDA-accelerated parallel trajectory optimization solver ensuring the manipulator avoids self-collisions and environmental collisions dynamically.
- **Embedded SDKs**: Contains self-contained, embedded C++ SDKs for both the Fairino FR5 (`libfairino.so.2.3.3`) and Tesollo Delto M gripper (`delto_hardware`, `delto_tcp_comm`). No external hardware configuration repositories are required for building.
- **Simulation/Dummy Mode**: Run the entire pipeline in an RViz simulation safely without connecting to physical hardware.

## Architecture & Nodes
- `master_bridge_node`: Receives raw VR glove tracking signals via UDP and maps them to ROS 2 target poses and joint states.
- `fairino_slave_node`: High-level controller capturing Cartesian objectives, preventing ground collisions, and publishing to cuRobo.
- `curobo_mppi_solver.py`: Receives cartesian pose objectives (`/curobo/pose_target`) and solves for a continuous smooth stream of joint targets via MPPI.
- `fairino_lowlevel_controller_node`: Connects directly to the robot hardware via the Fairino C++ SDK (`executeServoJ()`) or acts as a mocked publisher (`dummy_mode:=true`).
- `tesollo_slave_node`: Handles finger motion replication to the Tesollo Delto M gripper.

## Setup & Installation
### Prerequisites
- Ubuntu 22.04 + ROS 2 Humble
- NVIDIA GPU with CUDA Toolkit installed
- **NVIDIA cuRobo** (Install instructions: [cuRobo Getting Started](https://curobo.org/get_started/1_install.html))
- Fairino MoveIt2 config packages (`fairino5_v6_moveit2_config` for RViz visualization/robot description)

### Build
Because the SDKs for the hardware targets are native to this repository, you only need to run a standard colcon build.

```bash
cd ~/your_ros2_ws
colcon build --packages-select teleop_slave --symlink-install
source install/setup.bash
```

## Running the Simulation in RViz
You can verify the teleoperation pipeline works correctly using a simulated visualization (RViz). The simulation runs the underlying low-level controller in `dummy_mode`, meaning it bypasses the physical hardware connection attempts.

```bash
ros2 launch teleop_slave teleop_rviz.launch.py
```
This single launch file starts:
1. `rviz2` (with the `fairino5_v6` robot description)
2. `fairino_lowlevel_controller_node` (Simulation Mode enabled)
3. `fairino_slave_node` (High-level controller)
4. `curobo_mppi_solver` (Motion generation solver)

To test the motion, simply publish mock Cartesian targets to the `/manus/wrist_pose` topic, or run the `master_bridge_node` while using the VR glove.

## Working with Hardware
To use this with the actual Fairino and Tesollo robots, set `dummy_mode:=false` when running the low-level controller. Ensure the host machine is on the same local subnet as the FR5 control box (default `192.168.58.2`).
