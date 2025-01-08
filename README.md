# edu-franka_simulation
Franka Panda ROS 2 Ignition Gazebo Simulation for ROBO.720 Advanced Robotics

This repository provides a simulation environment for the Franka Panda robotic arm, utilizing ROS 2 and Ignition Gazebo. The project was developed as part of a university project to facilitate research and experimentation with robotic arm simulations. This repository evolves from the original [edu-elfin_simulation (ROS1)](https://github.com/tau-alma/edu-elfin_simulation), replacing the Elfin manipulator with the Franka Panda.


## Features
- Simulates the Franka Panda robotic arm in Ignition Gazebo.
- Supports ROS 2 integration for controlling the robot.

## Prerequisites
- Ubuntu 22.04
- ROS 2 Humble - [Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### (Optional) VMware
Suboptimal performance in Gazebo simulations due to only CPU rendering being available. Use the following argument when launching simulations in VMware:

```
LIBGL_ALWAYS_SOFTWARE=1
```
Example:
```
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch franka_gazebo computed_torque_controller.launch.py

## Installation

### Install Effort Controllers for Torque-Control Interface
```
sudo apt install ros-humble-ros2-control ros-humble-ign-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher-gui ros-humble-ros-gz
```

### Install Gazebo ROS Packages and Gazebo ROS Control
```
sudo apt-get install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-effort-controllers
```

### Install Controller Interface, Control Messages, and KDL Parser
```
sudo apt install ros-humble-controller-interface
sudo apt install ros-humble-control-msgs
sudo apt install ros-humble-kdl-parser
```

### Download, Create a Workspace, and Build
```
mkdir -p edu-franka_simulation_ws/src
cd ~/edu-franka_simulation_ws/src
git clone https://github.com/tau-alma/edu-franka_simulation.git
cd ~/edu-franka_simulation_ws/
colcon build
source install/setup.bash
```

## Running the Simulation
1. Open a new terminal and source the workspace:
   ```bash
   cd edu-franka_simulation_ws/
   colcon build
   source install/setup.bash
   ```
2. Launch a controller:
   ```bash
   ros2 launch franka_gazebo "INSERT_CONTROLLER_NAME_HERE".launch.py
   ```

### Available Controllers
- `computed_torque_controller`
- `time_delay_controller`
- `joint_impedance_example_controller`
- `joint_position_example_controller`
- `joint_velocity_example_controller`

## Licensing
This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

### Credit to Original Authors
The URDF and visuals for the Franka Panda robotic arm in this repository are based on the work from [franka_ros](https://github.com/frankaemika/franka_ros), which is licensed under the Apache 2.0 License. Modifications were made to adapt these files for the ROS 2 Ignition Gazebo simulation. The original authors retain copyright over their contributions, and their work is used in compliance with the terms of the Apache 2.0 License.

A copy of the Apache 2.0 License can be found in the [franka_ros](https://github.com/frankaemika/franka_ros) repository.

Additional references:
- [Modulabs Arm-Control Repository](https://github.com/modulabs/arm-control) for controller implementations
- [ROS 2 KDL Parser](https://github.com/ros/kdl_parser/tree/humble)
- [ROS 2 Controller Package Framework](https://control.ros.org/humble/doc/ros2_controllers/doc/writing_new_controller.html)
- [ROS 2 Migration Guide](https://docs.ros.org/en/foxy/The-ROS2-Project/Contributing/Migration-Guide.html)

## Acknowledgments
- [Franka Emika](https://www.franka.de/) for the design of the Franka Panda robotic arm.
- [franka_ros](https://github.com/frankaemika/franka_ros) repository for providing the base URDF and visual assets.