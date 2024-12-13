cmake_minimum_required(VERSION 3.0.2)
project(virtual_wall)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2_msgs
  tf2
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Wall.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES virtual_wall
  CATKIN_DEPENDS costmap_2d message_generation roscpp std_msgs  
#  DEPENDS system_lib
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(virtual_wall
  src/virtual_wall.cpp
)

add_dependencies(virtual_wall ${catkin_EXPORTED_TARGETS})

target_link_libraries(virtual_wall 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
