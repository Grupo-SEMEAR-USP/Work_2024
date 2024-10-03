cmake_minimum_required(VERSION 3.0.2)
project(robot_block_finder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  apriltag_ros
  message_generation
)

## Declare ROS messages and services
add_service_files(
  FILES
  SearchBlock.srv
)

catkin_install_python(PROGRAMS
  scripts/align_block.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy geometry_msgs apriltag_ros
)
