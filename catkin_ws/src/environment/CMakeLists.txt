cmake_minimum_required(VERSION 2.8.3)
project(environment)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  visualization_msgs
  common
)

catkin_package(
        CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_executable(environment src/environment.cpp)
target_link_libraries(environment ${catkin_LIBRARIES})