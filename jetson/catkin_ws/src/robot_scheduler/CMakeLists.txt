cmake_minimum_required(VERSION 3.0.2)
project(robot_scheduler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

# Instalação dos scripts Python
catkin_install_python(PROGRAMS
  scripts/scheduler_basic_test1.py
  scripts/scheduler_basic_test2.py
  scripts/scheduler_basic_test3.py
  scripts/scheduler_advanced_test1.py
  scripts/scheduler_advanced_test2.py
  scripts/scheduler_advanced_test3.py
  scripts/scheduler_finals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instalação de outros arquivos, se necessário
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
