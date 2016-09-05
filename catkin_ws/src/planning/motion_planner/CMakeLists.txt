cmake_minimum_required(VERSION 2.8.3)
project(motion_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  common
)

catkin_package(
        CATKIN_DEPENDS std_msgs nav_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_executable(motion_planner src/motion_planner.cpp)
target_link_libraries(motion_planner ${common_LIBRARIES} ${catkin_LIBRARIES} PathPlanningUtil)