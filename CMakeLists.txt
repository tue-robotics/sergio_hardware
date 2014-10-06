cmake_minimum_required(VERSION 2.8.3)
project(sergio_hardware)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_roscomm
  rtt_rosclock
  rtt_rosdeployment
  rtt_rosnode
  rtt_ros_msgs
  std_msgs
  )

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${scl_filters_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS})
	
file(GLOB_RECURSE HEADER_FILES include/*.hpp)

orocos_install_headers(DIRECTORY 
	include)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

install(FILES
sergiohardware.launch
soem.ops
base_controller.ops
base_controller_odom.ops
base_controller_odom_ff.ops
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Specific hardware components for testing torso ###
orocos_component(MotorTest src/MotorTest.cpp ${HEADER_FILES})
target_link_libraries(MotorTest ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
