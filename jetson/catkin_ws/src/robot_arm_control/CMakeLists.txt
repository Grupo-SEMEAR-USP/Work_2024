cmake_minimum_required(VERSION 3.0.2)
project(robot_arm_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

## Gerar mensagens e serviços
add_message_files(
  FILES
  DualServoCommand.msg
  MoveToPoseCommand.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_install_python(PROGRAMS
  scripts/pose_transition_node.py
  scripts/arm_teleop.py
  src/arm_hw_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
