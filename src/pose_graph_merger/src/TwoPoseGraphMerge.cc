/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <pose_graph_merger/TwoPoseGraphMerge.h>



TwoPoseGraphMerge::TwoPoseGraphMerge() : first_call_(true), robot_prefix_('Z') {}

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
  rob_node_pose_pub_ =
      nl.advertise<geometry_msgs::PoseStamped>("robot_last_node", 10, false);
  merged_node_pose_pub_ =
      nl.advertise<geometry_msgs::PoseStamped>("merged_last_node", 10, false);
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

geometry_msgs::PoseStamped TwoPoseGraphMerge::GetLatestOdomPose(const pose_graph_msgs::PoseGraphConstPtr& msg) {

  geometry_msgs::PoseStamped latest;
  GraphNode target;

  // Find the latest odom node from this robot
  // NOTE - assumes nodes are stored in timestamped order
  for (int i = msg->nodes.size() - 1; i >= 0; i--) {
    GraphNode n = msg->nodes[i];

    // Skip this node if it isn't from this robot
    char prefix = gtsam::Symbol(n.key).chr();
    if ((first_call_ && utils::IsRobotPrefix(prefix)) || prefix == robot_prefix_) {
      target = n;
      break;
    }
  }

  // On first call of this function, store the prefix of the current robot
  if (first_call_) {
    robot_prefix_ = gtsam::Symbol(target.key).chr();
    first_call_ = false;
  }

  latest.pose = target.pose;
  latest.header.stamp = target.header.stamp;

  return latest;
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

  // Poses from the robot-only graph and the merged graph
  robot_pose_ = GetLatestOdomPose(msg);
  robot_pose_.header.frame_id = "world";
  merged_pose_ = GetLatestOdomPose(fused_graph);
  merged_pose_.header.frame_id = "world_prime";

  // Publish
  PublishPoses();

  return;
}

pose_graph_msgs::PoseGraph TwoPoseGraphMerge::GetMergedGraph(){
  return merger_.GetCurrentGraph();
}

void TwoPoseGraphMerge::PublishPoses(){

  // Publish poses
  rob_node_pose_pub_.publish(robot_pose_);
  merged_node_pose_pub_.publish(merged_pose_);
}
