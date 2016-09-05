cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
  costmap_2d
  dynamicvoronoi
  common
)

find_package(CGAL QUIET COMPONENTS Core )

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

catkin_package(
        CATKIN_DEPENDS std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamicvoronoi_INCLUDE_DIRS}
  ${common_INCLUDE_DIRS}
)

add_executable(path_planning src/path_planning.cpp src/VoronoiPlanner.cpp)
target_link_libraries(path_planning ${common_LIBRARIES} ${dynamicvoronoi_LIBRARIES} ${catkin_LIBRARIES} ${CGAL_LIBRARY} ${GMP_LIBRARIES})
