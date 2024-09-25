cmake_minimum_required(VERSION 3.1.3)
project(robot_arm_control)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Adiciona os diretórios de cabeçalhos e bibliotecas do Gazebo
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Evita que o compilador faça strip de variáveis de ambiente do Gazebo
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(auto_grab_plugin SHARED plugin/auto_grab_plugin.cpp)
target_link_libraries(auto_grab_plugin ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
