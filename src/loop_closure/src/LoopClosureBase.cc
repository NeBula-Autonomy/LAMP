/*
LoopClosureBase.cc
Author: Yun Chang
Loop closure base class
*/
#include "loop_closure/LoopClosureBase.h"

#include <pose_graph_msgs/PoseGraph.h>

LoopClosure::LoopClosure(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  loop_closure_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("loop_closures", 10, false);
  keyed_stamps_sub_ = nl.subscribe<pose_graph_msgs::PoseGraphNode>(
      "new_node", 10, &LoopClosure::InputCallback, this);
}

LoopClosure::~LoopClosure(){};

void LoopClosure::InputCallback(
    const pose_graph_msgs::PoseGraphNode::ConstPtr& node_msg) {
  gtsam::Key new_key = node_msg->key;            // extract new key
  ros::Time timestamp = node_msg->header.stamp;  // extract new timestamp

  std::vector<pose_graph_msgs::PoseGraphEdge> loop_closure_edges;
  // first find loop closures
  if (FindLoopClosures(new_key, &loop_closure_edges)) {
    PublishLoopClosures(loop_closure_edges);
  }

  // then add thew key and stamp to keyed_stamps_
  keyed_stamps_[new_key] = timestamp;
}

void LoopClosure::PublishLoopClosures(
    const std::vector<pose_graph_msgs::PoseGraphEdge> edges) const {
  // For now publish PoseGraph messsage type (but only populate edges)
  pose_graph_msgs::PoseGraph graph;
  graph.edges = edges;

  loop_closure_pub_.publish(graph);
}
