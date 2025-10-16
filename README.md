# Setting up the simulation environment for Visual Servoing

This branch presents a simulation of Franka Panda arm with a camera sensor in gazebo environment. A custom world with an aruco marker is simulated and its pose is estimated by a pose estimation script. Simulation of gazebo sensors ( eg: camera) was part of gazebo classic (EOL 2025) and does not support Ignition gazebo (Fortress/Garden/Harmonic, a.k.a. `gz sim`). In `gz_sim` sensors are first-class `SDF` <sensor> elements. When you simulate an `URDF` file in ignition gazebo ( here after reffered to as gazebo) it is automatically converted to a `SDF` file during spawning. Defining a gazebo sensor inside `URDF` file does not accurfately convert into a working `SDF` file. Therefore, the recommended approach is to convert the `URDF` file to a `SDF` file offline and combine it with a `SDF`file of well defined camera model.


![Vis_serv_demo](./assets/vs_demo.gif)


## 1. Installation

### 1.1 Install and Initialize rosdep
```
sudo apt-get install python3-rosdep
```
If this is the first time using rosdep, it must be initialized via:
```
sudo rosdep init
rosdep update
```
### 1.2 Download, Create a Workspace, and Build

1.2.1 create the direectories

```
mkdir -p edu-franka_vs_ws/src
cd edu-franka_vs_ws/src
```
1.2.2 Clone the repository:
- Using HTTPS:
  ```
  git clone https://github.com/tau-alma/edu-franka_simulation.git -b visual_serv .
  ```
1.2.3 Install dependencies using rosdep:
```
cd ..
rosdep install --from-paths src -y --ignore-src
```
1.2.4 Build the workspace:
```
colcon build
source install/setup.bash
```

### 1.3 Generate the SDF files from URDF

1.3.1 convert the urdf.xacro to a complete urdf with inertial values:
```
ros2 run xacro xacro \
  src/franka_description/urdf/effort_panda_arm.urdf.xacro \
  sim_ignition:=true \
  -o src/franka_description/urdf/effort_panda_arm.urdf
```
1.3.2 Add a small inertia to `panda_link0` and `panda_link8`in the `effort_panda_arm.urdf`. 
( Add the following `<inertia>..</inertia>` snippets carefully to the relavant links in `urdf`. 
Otherwise the generate `.sdf` file will be inaccurate)

```
<link name="panda_link0">
  <inertial>
    <mass value="1e-3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
  <!-- your existing visual/collision here -->
</link>

<link name="panda_link8">
  <inertial>
    <mass value="1e-3"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
  <!-- your existing visual/collision here -->
</link>
```
1.3.3 populate the sdf file in model directory

```
gz sdf -p src/franka_description/urdf/effort_panda_arm.urdf > src/franka_gazebo/models/panda/model.sdf
```

1.3.4 Re-Build the workspace:
```
colcon build
source install/setup.bash
```
1.3.5 Launch the simulation
```
ros2 launch franka_gazebo launch_with_camera.launch.py
```

* If you face numpy versioning errors against opencv build, temporarily remove the numpy library from system packages using "pip uninstall numpy"


## 2. Implementation

- The SDF files of the Franka Panda robot and RGB camera are located at `franka_gazebo/models/panda` and `franka_gazebo/models/rgb_camera`  directories repectively.

- The combined model is at `franka_gazebo/models/panda_with_cam`

- The convertion of `URDF-->SDF ` is done as follows : ( make sure the urdf file exists in the current woorking directory)
    ```
    $ gz sdf -p effort_panda_arm.urdf > effort_panda_arm.sdf
    ```

- An aruco marker is simulated as a model in  `franka_gazebo/models/aruco_marker_0-100mm`. 
    - The `franka_gazebo/models/aruco_marker_0-100mm/materials/textures/aruco_0.png` is generated using the opencv based pathon scripts at `franka_gazebo/scripts/make_aruco.py`
        ```
        $ python3 "$(ros2 pkg prefix --share franka_gazebo)"/scripts/make_aruco.py --id 0 --dict DICT_4X4_50 --pixels 50 --outfile "$(ros2 pkg prefix --share franka_gazebo)"/models/aruco_marker_0_100mm/materials/textures/aruco_0.png
        ```
- A customized gazebo world with models `table` and `aruco_marker_0-100mm` is launched at simulation time located at `franka_gazebo/worlds/cam_world2.py`

- An inverse kinematic controller is configured and controlled using a predefined trajectory from `franka_gazebo/scripts/tri_wave.py`

- Finally, `franka_gazebo/scripts/pose_estimate.py` is an opencv based aruco marker pose estimation ros2 script which subscribes to rgb topics and visualizes estimated poses and display on an opencv window.


In addition following information are provided for your reference
## 3. Gazebo basics

Familiarize yourself with basics if you are new to gazebo environment at [gazebosim.org](gazebosim.org)

- [Simulating worlds and models in gazebo](https://gazebosim.org/docs/latest/building_robot/)
- [Adding more functionalities to gazebo world](https://gazebosim.org/docs/latest/building_robot/)
- [Spawning a custom gazebo world in ROS2 launch file](https://gazebosim.org/docs/latest/ros2_launch_gazebo/)
- [Bridging messages between ignition transport and ROS2](https://gazebosim.org/docs/latest/ros2_integration/)

## 4. Opencv Aruco

- Aruco marker creation and pose estimation is adopted from the opencv [documenatation](https://docs.opencv.org/3.4/d5/dae/tutorial_aruco_detection.html) 





