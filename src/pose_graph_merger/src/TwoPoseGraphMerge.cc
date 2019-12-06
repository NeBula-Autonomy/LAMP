/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <pose_graph_merger/TwoPoseGraphMerge.h>



TwoPoseGraphMerge::TwoPoseGraphMerge() {}

TwoPoseGraphMerge::~TwoPoseGraphMerge() {}


bool TwoPoseGraphMerge::Initialize(const ros::NodeHandle& n) {

  CreatePublishers(n);
  CreateSubscribers(n);

  return true;
}

// Create Publishers
bool TwoPoseGraphMerge::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  merged_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("merged_pose_graph", 10, false);
}

// Create Subscribers
bool TwoPoseGraphMerge::CreateSubscribers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  base_pose_graph_sub_ = nl.subscribe("base_graph",
                                          10,
                                          &TwoPoseGraphMerge::ProcessBaseGraph, this);
  robot_pose_graph_sub_ = nl.subscribe("robot_graph",
                                          10,
                                          &TwoPoseGraphMerge::ProcessRobotGraph, this);
}


void TwoPoseGraphMerge::ProcessBaseGraph(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  // Store the graph as the slow graph
  merger_.OnSlowGraphMsg(msg);
  ROS_INFO("Received graph from base station");
  return;
}

void TwoPoseGraphMerge::ProcessRobotGraph(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  // Combine the graph with the stored base graph and publish result
  
  // Add new posegraph
  merger_.OnFastGraphMsg(msg);

  // Get the fused graph
  pose_graph_msgs::PoseGraphConstPtr fused_graph(
    new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));

  // Publish the graph
  merged_graph_pub_.publish(*fused_graph);

  // TODO - think about timestamps 
  
  
  return;
}