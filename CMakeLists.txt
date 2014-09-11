cmake_minimum_required(VERSION 2.8.3)
project(amigo_hardware)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_control_components
  )

include_directories(${catkin_INCLUDE_DIRS})

include_directories(${USE_OROCOS_INCLUDE_DIRS})	
	
orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_control_components
)

include_directories(${catkin_INCLUDE_DIRS})
