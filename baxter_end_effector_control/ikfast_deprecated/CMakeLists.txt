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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   geometry_msgs
#   sensor_msgs
#   baxter_core_msgs
#   visualization_msgs
#)

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

## Declare a cpp library
# add_library(end_effector_teleoperation_control
#   src/${PROJECT_NAME}/end_effector_teleoperation_control.cpp
# )

## Declare a cpp executable
# add_executable(end_effector_teleoperation_control_node src/end_effector_teleoperation_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(end_effector_teleoperation_control_node end_effector_teleoperation_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(end_effector_teleoperation_control_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS end_effector_teleoperation_control end_effector_teleoperation_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_end_effector_teleoperation_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


## build end_effector_marker_control
add_executable(end_effector_marker_control src/end_effector_marker_control.cpp)
target_link_libraries(end_effector_marker_control ${catkin_LIBRARIES})

## build ikfastdemoright.cpp
#add_executable(ikfastdemoright src/ikfastdemoright.cpp)
#target_link_libraries(ikfastdemoright ${catkin_LIBRARIES} stdc++ lapack rt)

## build ikfastdemoleft.cpp
#add_executable(ikfastdemoleft src/ikfastdemoleft.cpp)
#target_link_libraries(ikfastdemoleft ${catkin_LIBRARIES} stdc++ lapack rt)

## end_effector_command_solver_ikfast.cpp
#add_executable(end_effector_command_solver_ikfast src/end_effector_command_solver_ikfast.cpp)
#target_link_libraries(end_effector_command_solver_ikfast ${catkin_LIBRARIES} stdc++ lapack rt)

## end_effector_command_solver_ikfast_left.cpp
#add_executable(end_effector_command_solver_ikfast_left src/end_effector_command_solver_ikfast_left.cpp)
#target_link_libraries(end_effector_command_solver_ikfast_left ${catkin_LIBRARIES} stdc++ lapack rt)

## end_effector_command_solver_ikfast_right.cpp
#add_executable(end_effector_command_solver_ikfast_right src/end_effector_command_solver_ikfast_right.cpp)
#target_link_libraries(end_effector_command_solver_ikfast_right ${catkin_LIBRARIES} stdc++ lapack rt)

## end_effector_marker_control_ikfast.cpp
#add_executable(end_effector_marker_control_ikfast src/end_effector_marker_control_ikfast.cpp)
#target_link_libraries(end_effector_marker_control_ikfast ${catkin_LIBRARIES} stdc++ lapack rt)

## build basic_controls.cpp
add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls ${catkin_LIBRARIES})

## build trac_ik_test.cpp
add_executable(trac_ik_test src/trac_ik_test.cpp)
target_link_libraries(trac_ik_test ${catkin_LIBRARIES})
