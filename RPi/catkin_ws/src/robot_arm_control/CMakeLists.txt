cmake_minimum_required(VERSION 3.1.3)
project(robot_arm_control)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Adicionar os arquivos de serviço
add_service_files(
  FILES
  ControlArm.srv
)

## Gerar mensagens e serviços
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Pacote catkin com as dependências
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

## Instalação dos diretórios de launch e config
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Incluir diretórios do Gazebo
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Evitar que o compilador faça strip de variáveis de ambiente do Gazebo
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

## Adicionar plugin e linkar bibliotecas do Gazebo
add_library(auto_grab_plugin SHARED plugin/auto_grab_plugin.cpp)
target_link_libraries(auto_grab_plugin ${GAZEBO_LIBRARIES})

## Instalar os scripts Python
catkin_install_python(PROGRAMS
  scripts/arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
