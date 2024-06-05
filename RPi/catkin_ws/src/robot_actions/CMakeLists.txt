cmake_minimum_required(VERSION 3.0.2)
project(robot_actions)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

add_action_files(
  FILES
  MoveToShelf.action
  Countdown.action
)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs actionlib_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


