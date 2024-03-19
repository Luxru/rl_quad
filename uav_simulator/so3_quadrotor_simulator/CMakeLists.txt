cmake_minimum_required(VERSION 2.8.3)
project(so3_quadrotor_simulator)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wno-deprecated-declarations  -g")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  uav_utils
  cmake_utils
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###########
## Build ##
###########

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ode)
##
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

find_package(Sophus REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
  CATKIN_DEPENDS roscpp quadrotor_msgs uav_utils geometry_msgs rospy std_msgs tf2 tf2_ros
  DEPENDS Eigen3 system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(quadrotor_dynamics 
src/dynamics/Quadrotor.cpp src/simulator.cc
)

target_link_libraries(quadrotor_dynamics
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  Sophus::Sophus
)

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3 
   quadrotor_dynamics
)
