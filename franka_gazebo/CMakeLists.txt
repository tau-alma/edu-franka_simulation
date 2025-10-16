
# franka_gazebo/CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
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


set(FR_SHARE "${CMAKE_INSTALL_PREFIX}/share/franka_description")
set(PKG_SHARE "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
set(CONTROLLERS_YAML "${PKG_SHARE}/config/franka_controllers.yaml")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda_with_cam/model.sdf.in
  ${CMAKE_CURRENT_BINARY_DIR}/models/panda_with_cam/model.sdf
  @ONLY
)


install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/panda_with_cam/model.sdf
  ${CMAKE_CURRENT_SOURCE_DIR}/models/panda_with_cam/model.config
  DESTINATION share/${PROJECT_NAME}/models/panda_with_cam
)


install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  PATTERN "panda_with_cam/model.sdf.in" EXCLUDE
  PATTERN "panda_with_cam/model.sdf" EXCLUDE
)


install(DIRECTORY launch config example_controllers_launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/tri_wave.py
  scripts/pose_estimate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

