/**
 * @file   LoopTransformComputation.cc
 * @brief  Base class for classes to find transform of loop closures
 * @author Yun Chang
 */
#pragma once

#include "loop_closure/LoopTransformComputation.h"

namespace lamp_loop_closure {

LoopTransformComputation() {}
~LoopTransformComputation() {}

bool CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_closure_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("loop_closures", 10, false);
  return true;
}

bool RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates",
      100,
      &LoopCandidatePrioritization::InputCallback,
      this);
  return true;
}

void PublishLoopClosures() {
  pose_graph_msgs::PoseGraph loop_closures_msg;
  loop_closures_msg.edges = output_queue_;
  loop_closure_pub_.publish(loop_closures_msg);
  output_queue_.clear();
}

void InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  for (auto candidate : input_candidates->candidates) {
    candidate_queue_.push(candidate);
  }
  return;
}

}  // namespace lamp_loop_closure