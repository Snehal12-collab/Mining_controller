cmake_minimum_required(VERSION 3.0.2)
project(articulation_stanley_controller)



find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  roslib  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stanley_controller
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(stanley_controller_node
  src/stanley_controller_node.cpp
  src/stanley_controller.cpp
)

target_link_libraries(stanley_controller_node
  ${catkin_LIBRARIES}
)

add_dependencies(stanley_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
