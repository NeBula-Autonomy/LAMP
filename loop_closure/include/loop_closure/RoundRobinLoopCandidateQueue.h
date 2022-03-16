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

#include "loop_closure/LoopCandidateQueue.h"

namespace lamp_loop_closure {

class RoundRobinLoopCandidateQueue : public LoopCandidateQueue  {
 public:
  RoundRobinLoopCandidateQueue();
  ~RoundRobinLoopCandidateQueue();

  virtual bool Initialize(const ros::NodeHandle& n);

  virtual bool LoadParameters(const ros::NodeHandle& n);


 protected:
  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);

  void LoopComputationStatusCallback(const pose_graph_msgs::LoopComputationStatus::ConstPtr& status);

  virtual void OnNewLoopClosure();

  virtual void OnLoopComputationCompleted();

  void FindNextSet();
  int key_;
  int amount_per_round_;
};

}  // namespace lamp_loop_closure