/**
 * @file   LoopCandidatePrioritization.cc
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopCandidatePrioritization.h"

namespace lamp_loop_closure {

LoopCandidatePrioritization::LoopCandidatePrioritization() {}
LoopCandidatePrioritization::~LoopCandidatePrioritization() {}

bool LoopCandidatePrioritization::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool LoopCandidatePrioritization::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "prioritized_loop_candidates", 10, false);
  return true;
}

bool LoopCandidatePrioritization::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates",
      100,
      &LoopCandidatePrioritization::InputCallback,
      this);
  return true;
}

void LoopCandidatePrioritization::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  for (auto candidate : input_candidates->candidates) {
    candidate_queue_.push(candidate);
  }
  return;
}
}  // namespace lamp_loop_closure