/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <pose_graph_merger/TwoPoseGraphMerge.h>



TwoPoseGraphMerge::TwoPoseGraphMerge() : robot_prefix_('Z'), b_have_first_robot_graph_(false) {}

TwoPoseGraphMerge::~TwoPoseGraphMerge() {}


bool TwoPoseGraphMerge::Initialize(const ros::NodeHandle& n) {

  CreatePublishers(n);
  CreateSubscribers(n);

  n.param<std::string>("frame_id_world1", this->world_fid_,  "/world");
  n.param<std::string>("frame_id_world2", this->world2_fid_, "/world_prime");

  n.param<bool>("b_publish_on_slow_graph", this->b_publish_on_slow_graph_, true);
  
  robot_prefix_ = utils::GetRobotPrefix(GetRobotName(n));

  return true;
}

std::string TwoPoseGraphMerge::GetRobotName(const ros::NodeHandle& n) {
  std::string ns = n.getNamespace();
  std::stringstream ss(ns);
  std::string item;
  std::getline(ss, item, '/');
  std::getline(ss, item, '/');
  return item;
}

// Create Publishers
bool TwoPoseGraphMerge::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  merged_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("merged_pose_graph", 10, true);
  rob_node_pose_pub_ =
      nl.advertise<geometry_msgs::PoseStamped>("robot_last_node_pose", 10, true);
  merged_node_pose_pub_ =
      nl.advertise<geometry_msgs::PoseStamped>("merged_last_node_pose", 10, true);
  merged_pose_pub_ = 
      nl.advertise<geometry_msgs::PoseStamped>("merged_pose", 10, false);
  return true;
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
  robot_pose_sub_ = nl.subscribe("robot_pose",
                                 10,
                                 &TwoPoseGraphMerge::ProcessRobotPose, this);
  return true;
}

geometry_msgs::PoseStamped TwoPoseGraphMerge::GetLatestOdomPose(
        const pose_graph_msgs::PoseGraphConstPtr& msg, 
        char target_prefix) {

  geometry_msgs::PoseStamped latest;
  GraphNode target;

  // Find the latest odom node from this robot
  // NOTE - assumes nodes are stored in timestamped order
  for (int i = msg->nodes.size() - 1; i >= 0; i--) {
    GraphNode n = msg->nodes[i];

    // Skip this node if it isn't from the right robot
    char prefix = gtsam::Symbol(n.key).chr();
    if (prefix == target_prefix) {
      target = n;
      break;
    }
  }

  latest.pose = target.pose;
  latest.header.stamp = target.header.stamp;

  return latest;
}

void TwoPoseGraphMerge::ProcessBaseGraph(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  // Store the graph as the slow graph
  merger_.OnSlowGraphMsg(msg);
  ROS_INFO("Received graph from base station");

  // Process again straight away 
  if (b_publish_on_slow_graph_){
    if (last_robot_graph_ != NULL){
      ProcessRobotGraph(last_robot_graph_);
    } else {
      ProcessRobotGraph(msg);
    }
  }
  return;
}

void TwoPoseGraphMerge::ProcessRobotGraph(const pose_graph_msgs::PoseGraphConstPtr& msg) {
  // Combine the graph with the stored base graph and publish result
  pose_graph_msgs::PoseGraph merged_graph;

  // Store the latest message
  last_robot_graph_ = msg;

  // Add new posegraph
  merger_.OnFastGraphMsg(msg);

  // Get the fused graph
  pose_graph_msgs::PoseGraphConstPtr fused_graph(
    new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));

  // Publish the graph
  merged_graph_pub_.publish(*fused_graph);

  // Poses from the robot-only graph and the merged graph
  robot_pose_ = GetLatestOdomPose(msg, robot_prefix_);
  robot_pose_.header.frame_id = this->world_fid_;
  merged_pose_ = GetLatestOdomPose(fused_graph, robot_prefix_);
  merged_pose_.header.frame_id = this->world2_fid_;

  // Publish
  PublishPoses();

  return;
}

void TwoPoseGraphMerge::ProcessRobotPose(const geometry_msgs::PoseStampedConstPtr& msg) {
  gtsam::Pose3 robot_pose = utils::ToGtsam(msg->pose);

  gtsam::Pose3 robot_node_pose = utils::ToGtsam(robot_pose_.pose);
  gtsam::Pose3 merged_node_pose = utils::ToGtsam(merged_pose_.pose);

  gtsam::Pose3 delta = robot_node_pose.between(robot_pose);
  gtsam::Pose3 new_pose = merged_node_pose.compose(delta);

  // Convert to ROS to publish
  geometry_msgs::PoseStamped output;
  output.pose = utils::GtsamToRosMsg(new_pose);
  output.header.frame_id = this->world2_fid_;
  output.header.stamp = msg->header.stamp;

  merged_pose_pub_.publish(output);
}

pose_graph_msgs::PoseGraph TwoPoseGraphMerge::GetMergedGraph(){
  return merger_.GetCurrentGraph();
}

void TwoPoseGraphMerge::PublishPoses(){

  // Publish poses
  rob_node_pose_pub_.publish(robot_pose_);
  merged_node_pose_pub_.publish(merged_pose_);
}
