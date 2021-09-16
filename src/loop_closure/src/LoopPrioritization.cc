/**
 * @file   LoopPrioritization.cc
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopPrioritization.h"

namespace lamp_loop_closure {

LoopPrioritization::LoopPrioritization() {}
LoopPrioritization::~LoopPrioritization() {}

bool LoopPrioritization::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool LoopPrioritization::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "prioritized_loop_candidates", 10, false);
  return true;
}

bool LoopPrioritization::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 100, &LoopPrioritization::InputCallback, this);



  return true;
}

std::vector<ros::AsyncSpinner> LoopPrioritization::SetAsyncSpinners(const ros::NodeHandle &n) {
  ros::NodeHandle nl(n);
  std::vector<ros::AsyncSpinner> spinners;
  ros::AsyncSpinner spinner_populate_queue_(1,&this->prioritize_queue_);
  spinners.emplace_back(spinner_populate_queue_);
  ros::TimerOptions options(ros::Duration(1), [this] (const ros::TimerEvent& ev) -> void {this->ProcessPopulateCallback(ev);}, &this->prioritize_queue_);
  populate_timer_ = nl.createTimer(options);
  return spinners;
}

void LoopPrioritization::ProcessPopulateCallback(const ros::TimerEvent& ev) {
  PopulatePriorityQueue();
}

void LoopPrioritization::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  for (auto candidate : input_candidates->candidates) {
    candidate_queue_.push(candidate);
  }
  return;
}
} // namespace lamp_loop_closure