/*
LoopClosureBase.cc
Author: Yun Chang
Loop closure base class
*/
#include "loop_closure/LoopClosureBase.h"

#include <pose_graph_msgs/PoseGraph.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

LoopClosure::LoopClosure(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing

  keyed_stamps_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental", 100000, &LoopClosure::InputCallback, this);

  // False by default, set to true within derived class initializations
  b_check_for_loop_closures_ = false;
}

LoopClosure::~LoopClosure(){};

void LoopClosure::InputCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {

  bool b_is_new_node = true;

  // Loop through each node (but expect only one at a time)
  ROS_INFO_STREAM("In Input Callback of loop closure (" << graph_msg->nodes.size() <<
      " nodes received)");
  pose_graph_msgs::PoseGraphNode node_msg;
  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Key new_key = node_msg.key;            // extract new key
    ros::Time timestamp = node_msg.header.stamp;  // extract new timestamp

    // Check if the node is new 
    if (keyed_poses_.count(new_key) > 0){
      b_is_new_node = false;
    } else { 
      b_is_new_node = true;
    }

    // also extract poses (NOTE(Yun) this pose will not be updated...)
    gtsam::Pose3 new_pose;
    gtsam::Point3 pose_translation(node_msg.pose.position.x,
                                   node_msg.pose.position.y,
                                   node_msg.pose.position.z);
    gtsam::Rot3 pose_orientation(node_msg.pose.orientation.w,
                                 node_msg.pose.orientation.x,
                                 node_msg.pose.orientation.y,
                                 node_msg.pose.orientation.z);
    new_pose = gtsam::Pose3(pose_orientation, pose_translation);

    // add new key and stamp to keyed_stamps_
    keyed_stamps_[new_key] = timestamp;
    
    // add new key and pose to keyed_poses_
    keyed_poses_[new_key] = new_pose;

    // Skip next part if not checking for loop closures
    if (!b_check_for_loop_closures_ || !b_is_new_node) {
      continue;
    }

    std::vector<pose_graph_msgs::PoseGraphEdge> loop_closure_edges;
    // first find loop closures
    if (FindLoopClosures(new_key, &loop_closure_edges)) {
      ROS_INFO_STREAM(
          "----------------------Found Loop "
          "Closure----------------------\n from key "
          << new_key);
      PublishLoopClosures(loop_closure_edges);
    }
    ROS_INFO("No Loop Closure");
  }
}

void LoopClosure::PublishLoopClosures(
    const std::vector<pose_graph_msgs::PoseGraphEdge> edges) const {
  // For now publish PoseGraph messsage type (but only populate edges)
  pose_graph_msgs::PoseGraph graph;
  graph.edges = edges;

  loop_closure_pub_.publish(graph);
}
