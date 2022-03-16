cmake_minimum_required(VERSION 2.8.3)
project(pose_graph_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  PoseGraph.msg
  PoseGraphNode.msg
  PoseGraphEdge.msg
  KeyedScan.msg
  KeyValue.msg
  LoopCandidate.msg
  LoopCandidateArray.msg
  LoopComputationStatus.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
)

# Test
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_bridge.test)
endif()
