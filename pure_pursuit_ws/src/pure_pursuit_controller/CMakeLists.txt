cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_controller)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  roslib
)

## Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pure_pursuit_controller
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs tf roslib
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Build the library
add_library(pure_pursuit_controller src/pure_pursuit_controller.cpp)
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})

## Build the executable
# add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)
# target_link_libraries(pure_pursuit_controller_node
#   pure_pursuit_controller   # Link with the library
#   ${catkin_LIBRARIES}
# )

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)

target_link_libraries(pure_pursuit_controller_node
  pure_pursuit_controller
  ${catkin_LIBRARIES}      # This links roslib too
)