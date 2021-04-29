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
      "loop_candidates", 10, false);
  return true;
}

bool LoopCandidateQueue::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 100, &LoopCandidateQueue::InputCallback, this);
  return true;
}

void LoopCandidateQueue::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  PublishLoopCandidate(*input_candidates);
}

void LoopCandidateQueue::PublishLoopCandidate(
    const pose_graph_msgs::LoopCandidateArray& candidates) {
  loop_candidate_pub_.publish(candidates);
}

}  // namespace lamp_loop_closure