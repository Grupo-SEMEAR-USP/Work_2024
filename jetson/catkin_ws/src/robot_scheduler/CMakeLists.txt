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
  scripts/scheduler_basic_task1.py
  scripts/scheduler_basic_task2.py
  scripts/scheduler_basic_task3.py
  scripts/scheduler_advanced_task1.py
  scripts/scheduler_advanced_task2.py
  scripts/scheduler_advanced_task3.py
  scripts/scheduler_finals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Instalação de outros arquivos, se necessário
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
