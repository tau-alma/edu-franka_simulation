cmake_minimum_required(VERSION 3.5)

project(franka_gazebo)

set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(franka_description REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch config example_controllers_launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

#Install the script so ros2 run can find it
install(PROGRAMS
   scripts/tri_wave.py
   scripts/pose_estimate.py
   DESTINATION lib/${PROJECT_NAME})

# Absolute share dir for this install
set(PKG_SHARE "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# Generate wrapper with absolute file:// URIs
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda_with_cam/model.sdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/models/panda_with_cam/model.sdf
  @ONLY
)

# Install the generated wrapper + its model.config
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/panda_with_cam/model.sdf
  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda_with_cam/model.config
  DESTINATION share/${PROJECT_NAME}/models/panda_with_cam
)

# (and keep your existing rule that installs models/ and worlds/)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "panda_with_cam/model.sdf.in" EXCLUDE) # don't install the template
  
# Absolute share dir of franka_description in the final install
set(FR_SHARE "${CMAKE_INSTALL_PREFIX}/share/franka_description")

# Read the source Panda SDF and replace any URIs:
#  - model://franka_description           -> file://<FR_SHARE>
#  - (and fix any accidental wrong franka_gazebo-based path)
file(READ  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda/model.sdf  _PANDA_IN)
string(REPLACE "model://franka_description" "file://${FR_SHARE}" _PANDA_TMP "${_PANDA_IN}")
string(REPLACE "file:///franka_gazebo/share/franka_description" "file://${FR_SHARE}" _PANDA_OUT "${_PANDA_TMP}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/models/panda/model.sdf "${_PANDA_OUT}")

# Install the patched Panda SDF + model.config
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/panda/model.sdf
  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda/model.config
  DESTINATION share/${PROJECT_NAME}/models/panda)

# Install the rest of your models, but exclude the source panda/model.sdf so it doesn't overwrite the patched one
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "panda/model.sdf" EXCLUDE)
 
ament_package()
