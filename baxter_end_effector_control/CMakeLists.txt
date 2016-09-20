cmake_minimum_required(VERSION 2.8.3)
project(baxter_end_effector_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  visualization_msgs
  interactive_markers
  sensor_msgs
  baxter_core_msgs
  visualization_msgs
  trac_ik_lib
  eigen_conversions
  kdl_conversions
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    trac_ik_lib
    kdl_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${eigen_INCLUDE_DIRS}
)

## build end_effector_marker_control
add_executable(end_effector_marker_control src/end_effector_marker_control.cpp)
target_link_libraries(end_effector_marker_control ${catkin_LIBRARIES})

## build basic_controls.cpp
add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls ${catkin_LIBRARIES})

## build trac_ik_test.cpp
add_executable(trac_ik_test src/trac_ik_test.cpp)
target_link_libraries(trac_ik_test ${catkin_LIBRARIES})
