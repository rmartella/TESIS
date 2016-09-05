cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  common
)

catkin_package(
        CATKIN_DEPENDS std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_library(LowLevelControl src/LowLevelControl.cpp)
target_link_libraries(LowLevelControl ${catkin_LIBRARIES})

add_executable(basic_motion src/BasicMotionActions.cpp)
target_link_libraries(basic_motion ${catkin_LIBRARIES} LowLevelControl)