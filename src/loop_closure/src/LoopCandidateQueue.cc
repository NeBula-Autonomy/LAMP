/**
 * @file   LoopCandidateQueue.cc
 * @brief  Consolidation queue for loop closure candidates before computing
 * transform
 * @author Yun Chang
 */
#pragma once

#include <utils/CommonFunctions.h>

#include "loop_closure/LoopCandidateQueue.h"

namespace lamp_loop_closure {

LoopCandidateQueue::LoopCandidateQueue() {}
LoopCandidateQueue::~LoopCandidateQueue() {}

bool LoopCandidateQueue::Initialize(const ros::NodeHandle& n) {
  std::string name = ros::names::append(n.getNamespace(), "LoopCandidateQueue");
  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name.c_str());
    return false;
  }

  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name.c_str());
    return false;
  }

  return true;
}

bool LoopCandidateQueue::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  param_ns_ = utils::GetParamNamespace(n.getNamespace());

  return true;
}

bool LoopCandidateQueue::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_pub_ = nl.advertise<pose_graph_msgs::LoopCandidateArray>(
      "output_loop_candidates", 10, false);
  return true;
}

bool LoopCandidateQueue::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "input_loop_candidates_prioritized", 100, &LoopCandidateQueue::InputCallback, this);

  loop_closure_status_sub_ = nl.subscribe<pose_graph_msgs::LoopComputationStatus>(
      "loop_computation_status", 100, &LoopCandidateQueue::LoopComputationStatusCallback, this);

  return true;
}

void LoopCandidateQueue::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  //ROS_INFO_STREAM("Recieved " << input_candidates->candidates.size());
  for (auto candidate : input_candidates->candidates) {
    queues[input_candidates->originator].push_back(candidate);
  }
  OnNewLoopClosure();
}

void LoopCandidateQueue::PublishAllLoopClosures(){
  pose_graph_msgs::LoopCandidateArray candidate_array;
  for (auto const& cur_queue : queues)
  {
    for (auto loop_candidate : cur_queue.second){
      candidate_array.candidates.push_back(loop_candidate);
    }
  }
  if (candidate_array.candidates.size() > 0) {
    PublishLoopCandidate(candidate_array);
  }
}

void LoopCandidateQueue::OnNewLoopClosure(){
  //By default, drain the whole system
  PublishAllLoopClosures();
}
void LoopCandidateQueue::OnLoopComputationCompleted(){
  //By default, drain the whole system
  PublishAllLoopClosures();
}


void LoopCandidateQueue::LoopComputationStatusCallback(const pose_graph_msgs::LoopComputationStatus::ConstPtr& status){

  if (status->type == status->COMPLETED_ALL){
    OnLoopComputationCompleted();
  }

}

void LoopCandidateQueue::PublishLoopCandidate(
    const pose_graph_msgs::LoopCandidateArray& candidates, bool check_sent) {
  if (check_sent) {
    pose_graph_msgs::LoopCandidateArray out_candidate_array;
    for (auto const &loop_candidate : candidates.candidates) {
      if (!LoopClosureHasBeenSent(loop_candidate)) {
        out_candidate_array.candidates.push_back(loop_candidate);
        AddLoopClosureToSent(loop_candidate);

      }
    }
    loop_candidate_pub_.publish(out_candidate_array);
  } else {
    loop_candidate_pub_.publish(candidates);
  }
}
std::string LoopCandidateQueue::make_key(const pose_graph_msgs::LoopCandidate& loop_closure){
  std::stringstream ss;
  ss << loop_closure.key_from <<"-" <<loop_closure.key_to <<"-"<<loop_closure.type;
  return ss.str();
}


bool LoopCandidateQueue::LoopClosureHasBeenSent(const pose_graph_msgs::LoopCandidate& loop_closure){
  auto got = sent_loop_closures_.find (make_key(loop_closure));

  return (got != sent_loop_closures_.end() );
}

void LoopCandidateQueue::AddLoopClosureToSent(const pose_graph_msgs::LoopCandidate& loop_closure){
  sent_loop_closures_.emplace(make_key(loop_closure));
}

} // namespace lamp_loop_closure