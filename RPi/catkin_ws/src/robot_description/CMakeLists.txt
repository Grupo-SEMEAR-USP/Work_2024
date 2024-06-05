cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  robot_control
)

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS GAZEBO
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(robot_drive_plugin src/robot_drive_plugin.cc)

target_link_libraries(robot_drive_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
