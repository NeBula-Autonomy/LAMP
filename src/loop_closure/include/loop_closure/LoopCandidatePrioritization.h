/**
 * @file   LoopCandidatePrioritization.h
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */
#pragma once

#include <deque>
#include <map>
#include <vector>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopCandidatePrioritization {
 public:
  LoopCandidatePrioritization();
  ~LoopCandidatePrioritization();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  inline pose_graph_msgs::PoseGraphEdge GetBestCandidate() {
    auto e = priority_queue_.front();
    priority_queue_.pop_front();
    return e;
  }

  virtual bool InsertToQueue(
      const pose_graph_msgs::PoseGraphEdge& candidate_loop) = 0;

 protected:
  // Loop closure candidates priority queue (high to low)
  std::deque<pose_graph_msgs::PoseGraphEdge> priority_queue_;

  std::string param_ns_;
};

}  // namespace lamp_loop_closure