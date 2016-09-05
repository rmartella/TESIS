cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  tf_conversions
  message_generation
  actionlib_msgs
  geometry_msgs
)

find_package(Eigen REQUIRED)

SET(CMAKE_CXX_FLAGS "-frounding-math -std=c++11")

add_action_files(
        DIRECTORY action
        FILES
        GoalDist.action
        GoalDistAngle.action
        GoalPath.action
        GoalPose.action
)

add_service_files(
   FILES
   Environment.srv
   GoalDistAngle.srv
   GoalDist.srv
   GoalPose.srv
   GoalPath.srv
   PathPlanning.srv
   MotionPlannerXYA.srv
   MotionPlannerLoc.srv
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
        DEPENDS Eigen
        CATKIN_DEPENDS message_runtime actionlib_msgs nav_msgs std_msgs tf geometry_msgs
        INCLUDE_DIRS include/
        LIBRARIES EnvironmentUtil NavigationUtil PointClickSubscriber
)

include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(EnvironmentUtil src/EnvironmentUtil.cpp)
target_link_libraries(EnvironmentUtil ${catkin_LIBRARIES})
add_dependencies(EnvironmentUtil common_gencpp)

add_library(NavigationUtil src/NavigationUtil.cpp)
target_link_libraries(NavigationUtil ${catkin_LIBRARIES})
add_dependencies(NavigationUtil common_gencpp)

add_library(PointClickSubscriber src/PointClickSubscriber.cpp)
target_link_libraries(PointClickSubscriber ${catkin_LIBRARIES})
add_dependencies(PointClickSubscriber common_gencpp)

add_executable(testNavigation src/TestNavigation.cpp)
target_link_libraries(testNavigation ${catkin_LIBRARIES} NavigationUtil)
add_dependencies(testNavigation common_gencpp)

add_library(PathPlanningUtil src/PathPlanningUtil.cpp)
target_link_libraries(PathPlanningUtil ${catkin_LIBRARIES} NavigationUtil)
add_dependencies(PathPlanningUtil common_gencpp)