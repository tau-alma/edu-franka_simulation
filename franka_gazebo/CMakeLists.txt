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
    launch config example_controllers_launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
