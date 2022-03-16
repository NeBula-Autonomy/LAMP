cmake_minimum_required(VERSION 2.8.3)
project(roadmap_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    roscpp
)

find_package(Boost REQUIRED
  COMPONENTS
    graph
)

# Python package
catkin_python_setup()

# ROS messages
add_message_files(
  DIRECTORY
    msg
  FILES
    MLPFeedback.msg
    Roadmap.msg
    RoadmapNode.msg
    RoadmapEdge.msg
    RoadmapAnchor.msg
    RoadmapProperty.msg
    DlioMetrics.msg
)

add_service_files(
  FILES
  GetRoadmap.srv
  UpdateRoadmap.srv
  PlanPath.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    nav_msgs
)

# Package definition
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roadmap_bridge
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
)

# Build
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(roadmap_bridge
  src/roadmap_bridge.cpp
  src/roadmap_graph.cpp
)

add_dependencies(roadmap_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(roadmap_bridge
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_bridge_py.test)
endif()
