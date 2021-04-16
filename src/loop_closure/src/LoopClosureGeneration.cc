/**
 * @file   LoopCandidateGeneration.cc
 * @brief  Base class for classes to find potentital loop closures
 * @author Yun Chang
 */
#include <pose_graph_msgs/PoseGraphNode.h>

#include "loop_closure/LoopCandidateGeneration.h"

namespace lamp_loop_closure {

LoopCandidateGeneration() {}
~LoopCandidateGeneration() {}

bool LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 10, false);
  return true;
}

bool RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental",
      100,
      &LoopCandidateGeneration::KeyedPoseCallback,
      this);
  return true;
}

}  // namespace lamp_loop_closure