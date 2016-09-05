cmake_minimum_required(VERSION 2.8.3)
project(laser_simul)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  common
)

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${common_INCLUDE_DIRS}
)

add_executable(laser_scan_publisher src/laser_scan_publisher.cpp)
target_link_libraries(laser_scan_publisher ${catkin_LIBRARIES})