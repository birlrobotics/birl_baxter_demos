cmake_minimum_required(VERSION 1.0)
project(baxter_dmp_rl)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
cmake_modules
roscpp
rospy
message_generation
)
find_package(Eigen REQUIRED)

add_message_files(
   FILES
   DMPData.msg
   DMPPoint.msg
   DMPTraj.msg
 )

add_service_files(
   FILES
   GetDMPPlan.srv
   LearnDMPFromDemo.srv
   SetActiveDMP.srv
 )

generate_messages()

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS common_msgs image_common message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(baxter_dmp_rl src/dmp.cpp src/fourier_approx.cpp src/radial_approx.cpp src/linear_approx.cpp)
add_executable(dmp_server nodes/dmp_server.cpp)
target_link_libraries(dmp_server baxter_dmp_rl ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
