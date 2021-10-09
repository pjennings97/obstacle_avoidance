cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  rospy
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(goal_publisher src/goal_publisher.cpp)
target_link_libraries(goal_publisher ${catkin_LIBRARIES})