cmake_minimum_required(VERSION 2.8.3)

project(artifact_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)

## Declare ROS messages, services and actions
add_message_files(
  FILES
    ScorabilityMetrics.msg
    Artifact.msg
    PointSourceDetection.msg
    WiFiScanRaw.msg
)


add_service_files(
  DIRECTORY
  srv
  FILES
  GetNetvladDesc.srv
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)

## Declare as a catkin package
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
)
