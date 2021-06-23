/**
 * @file   LoopCandidateQueue.h
 * @brief  Consolidation queue for loop closure candidates before computing
 * transform
 * @author Yun Chang
 */
#pragma once

#include <deque>
#include <map>
#include <queue>
#include <vector>

#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopCandidateQueue {
public:
  LoopCandidateQueue();
  ~LoopCandidateQueue();

  bool Initialize(const ros::NodeHandle& n);

  bool LoadParameters(const ros::NodeHandle& n);

  bool CreatePublishers(const ros::NodeHandle& n);

  bool RegisterCallbacks(const ros::NodeHandle& n);

protected:
  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);

  void
  PublishLoopCandidate(const pose_graph_msgs::LoopCandidateArray& candidates);

  // Define publishers and subscribers
  ros::Publisher loop_candidate_pub_;
  ros::Subscriber loop_candidate_sub_;

  std::string param_ns_;
};

} // namespace lamp_loop_closure