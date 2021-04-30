cmake_minimum_required(VERSION 2.8.3)
project(dome)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  dome_common_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  daq_interface)


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
#  LIBRARIES dome
#  DEPENDS system_lib
CATKIN_DEPENDS roscpp rospy std_msgs dome_common_msgs message_runtime
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
include_directories(include ${catkin_INCLUDE_DIRS} /usr/xenomai/include) 
link_directories(/usr/xenomai/lib /usr/lib/nvidia-304)

add_library(textfile src/VisualScene/textfile.cpp)

add_library(helper src/VisualScene/helper.cpp)

add_library(DomeShape src/VisualScene/DomeShape.cpp)
target_link_libraries(DomeShape ${catkin_LIBRARIES} helper)

add_library(DomeShapeLibrary src/VisualScene/DomeShapeLibrary.cpp)
target_link_libraries(DomeShapeLibrary DomeShape)

add_executable(VisualScene src/VisualScene/VisualScene.cpp)
target_link_libraries(VisualScene ${catkin_LIBRARIES} pthread GL GLEW GLU glut textfile helper DomeShapeLibrary)

add_executable(EncoderPublisher src/Experiment/EncoderPublisher.cpp)
target_link_libraries(EncoderPublisher ${catkin_LIBRARIES})

add_executable(EncoderTester src/Experiment/EncoderTester.cpp)
target_link_libraries(EncoderTester ${catkin_LIBRARIES})

add_executable(ZListener src/Experiment/ZListener.cpp)
target_link_libraries(ZListener ${catkin_LIBRARIES})

add_executable(SyncSender src/Experiment/SyncSender.cpp)
target_link_libraries(SyncSender ${catkin_LIBRARIES})

add_executable(FeedServer src/Experiment/FeedServer.cpp)
target_link_libraries(FeedServer ${catkin_LIBRARIES})
