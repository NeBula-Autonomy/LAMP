cmake_minimum_required(VERSION 2.8.3)
project(localizer_zero_velocity_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(yaml-cpp)

add_message_files(
  FILES
  Stationary.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES very_stable_genius
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs visualization_msgs geometry_msgs
#  DEPENDS yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(very_stable_genius src/very_stable_genius.cpp)
target_link_libraries(very_stable_genius
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(very_stable_genius_visualizer src/very_stable_genius_visualizer.cpp)
target_link_libraries(very_stable_genius_visualizer
  ${catkin_LIBRARIES}
  very_stable_genius
  yaml-cpp
)

add_executable(very_stable_genius_test test/very_stable_genius_test.cpp)
target_link_libraries(very_stable_genius_test
  ${catkin_LIBRARIES}
  very_stable_genius
  yaml-cpp
)

add_executable(very_stable_genius_node src/very_stable_genius_node.cpp)
target_link_libraries(very_stable_genius_node
  ${catkin_LIBRARIES}
  very_stable_genius
  yaml-cpp
)

add_dependencies(very_stable_genius_node
  localizer_zero_velocity_detector_generate_messages_cpp
)
