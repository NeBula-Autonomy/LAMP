cmake_minimum_required(VERSION 2.8.3)
project(uwb_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation  
)

add_message_files(
  DIRECTORY msg
  FILES
    Anchor.msg
    InterVehicleRange.msg
  )

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime geometry_msgs)
