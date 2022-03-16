cmake_minimum_required(VERSION 2.8.3)
project(pose_graph_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  PoseGraph.msg
  PoseGraphNode.msg
  PoseGraphEdge.msg
  PoseAndScan.msg
  KeyedScan.msg
  KeyValue.msg
  LoopCandidate.msg
  LoopCandidateArray.msg
  LoopComputationStatus.msg
  CommNodeInfo.msg
  CommNodeStatus.msg
  MapInfo.msg
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