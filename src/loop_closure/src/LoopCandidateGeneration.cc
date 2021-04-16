/**
 * @file   LoopCandidateGeneration.cc
 * @brief  Base class for classes to find potentital loop closures
 * @author Yun Chang
 */
#include <pose_graph_msgs/PoseGraphNode.h>
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopCandidateGeneration.h"

namespace lamp_loop_closure {

LoopCandidateGeneration::LoopCandidateGeneration() {}
LoopCandidateGeneration::~LoopCandidateGeneration() {}

bool LoopCandidateGeneration::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool LoopCandidateGeneration::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 10, false);
  return true;
}

bool LoopCandidateGeneration::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental",
      100,
      &LoopCandidateGeneration::KeyedPoseCallback,
      this);
  return true;
}

void LoopCandidateGeneration::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  pose_graph_msgs::PoseGraphNode node_msg;
  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Key new_key = node_msg.key;            // extract new key
    ros::Time timestamp = node_msg.header.stamp;  // extract new timestamp

    // Check if the node is new
    if (keyed_poses_.count(new_key) > 0) {
      continue;  // Not a new node
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

    // add new key and pose to keyed_poses_
    keyed_poses_[new_key] = new_pose;

    GenerateLoopCandidates(new_key);
  }

  if (loop_candidate_pub_.getNumSubscribers() > 0) {
    PublishLoopCandidates();
  }
  return;
}

}  // namespace lamp_loop_closure