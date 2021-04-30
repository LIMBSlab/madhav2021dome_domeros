cmake_minimum_required(VERSION 2.8.3)
project(daq_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dome_common_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)
find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
DIRECTORY
	msg
	FILES
        DaqEvent.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY
	srv
	FILES
        ReadEncoder.srv
        WriteDigital.srv
        ReadDigital.srv
        WriteAnalog.srv
        ReadAnalog.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY
  action
  FILES
  PulseDigital.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES daq_interface
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS dome_common_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions("-std=c++0x")
include_directories(include ${catkin_INCLUDE_DIRS} /usr/xenomai/include/cobalt /usr/xenomai/include)
link_directories(/usr/xenomai/lib)

add_library(daqapi src/daqapi.cpp)
target_link_libraries(daqapi analogy cobalt)
add_dependencies(daqapi daq_interface_gencpp)

add_executable(daq_interface src/daq_interface.cpp)
target_link_libraries(daq_interface daqapi ${catkin_LIBRARIES})
add_dependencies(daq_interface daq_interface_generate_messages_cpp)
