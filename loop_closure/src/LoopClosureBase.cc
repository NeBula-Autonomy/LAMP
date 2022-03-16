/*
LoopClosureBase.cc
Author: Yun Chang
Loop closure base class
*/
#include "loop_closure/LoopClosureBase.h"

#include <geometry_utils/GeometryUtilsROS.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <utils/CommonFunctions.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

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
  ROS_DEBUG_STREAM("In Input Callback of loop closure ("
                   << graph_msg->nodes.size() << " nodes received)");
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
  }
}

void LoopClosure::PublishLoopClosures(
    const std::vector<pose_graph_msgs::PoseGraphEdge> edges) const {
  // For now publish PoseGraph messsage type (but only populate edges)
  pose_graph_msgs::PoseGraph graph;
  graph.edges = edges;

  loop_closure_pub_.publish(graph);
}

pose_graph_msgs::PoseGraphEdge LoopClosure::CreateLoopClosureEdge(
    const gtsam::Symbol& key1,
    const gtsam::Symbol& key2,
    const geometry_utils::Transform3& delta,
    const gtsam::Matrix66& covariance) {
  // Store last time a new loop closure was added
  if (key1 > last_closure_key_[{key1.chr(), key2.chr()}]) {
    last_closure_key_[{key1.chr(), key2.chr()}] = key1;
  }
  if (key2 > last_closure_key_[{key2.chr(), key1.chr()}]) {
    last_closure_key_[{key2.chr(), key1.chr()}] = key2;
  }

  // Create the new loop closure edge
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key1;
  edge.key_to = key2;
  edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
  edge.pose = gr::ToRosPose(delta);

  // Convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      edge.covariance[6 * i + j] = covariance(i, j);
    }
  }

  return edge;
}

pose_graph_msgs::PoseGraphEdge LoopClosure::CreatePriorEdge(
    const gtsam::Symbol& key,
    const geometry_utils::Transform3& delta,
    const gtsam::Matrix66& covariance) {
  pose_graph_msgs::PoseGraphEdge prior;
  prior.key_from = key;
  prior.key_to = key;
  prior.type = pose_graph_msgs::PoseGraphEdge::PRIOR;
  prior.pose.position.x = delta.translation.X();
  prior.pose.position.y = delta.translation.Y();
  prior.pose.position.z = delta.translation.Z();
  prior.pose.orientation.w = utils::ToGtsam(delta).rotation().quaternion()[0];
  prior.pose.orientation.x = utils::ToGtsam(delta).rotation().quaternion()[1];
  prior.pose.orientation.y = utils::ToGtsam(delta).rotation().quaternion()[2];
  prior.pose.orientation.z = utils::ToGtsam(delta).rotation().quaternion()[3];

  // Convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      if (i == j) {
        prior.covariance[6 * i + j] = covariance(i, j);
      }
    }
  }
  return prior;
}