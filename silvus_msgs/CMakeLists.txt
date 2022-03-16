cmake_minimum_required(VERSION 2.8.3)
project(silvus_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  roadmap_msgs
  message_generation 
)

add_message_files(
  DIRECTORY
    msg
  FILES
    SilvusDualEdgeList.msg
    SilvusDualEdge.msg
    SilvusHealth.msg
    SilvusRouteList.msg
    SilvusRoute.msg
    SilvusStreamscape.msg
    SilvusStreamscapeNeighbor.msg
    SilvusStreamscapeNode.msg
    key_value_int32_int32.msg
    SilvusRoadmap.msg
)

#add_service_files(
#  FILES
#)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs roadmap_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime geometry_msgs sensor_msgs roadmap_msgs)
