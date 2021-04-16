/**
 * @file   LoopTransformComputation.h
 * @brief  Base class for classes to find transform of loop closures
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <queue>
#include <vector>

#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopTransformComputation {
 public:
  LoopTransformComputation();
  ~LoopTransformComputation();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  virtual bool LoadParameters(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  // Compute transform and populate output queue
  virtual bool ComputeTransforms() = 0;

 protected:
  void PublishLoopClosures();

  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);

  // Define publishers and subscribers
  ros::Publisher loop_closure_pub_;
  ros::Subscriber loop_candidate_sub_;

  // Computed loop closures
  std::vector<pose_graph_msgs::PoseGraphEdge> output_queue_;
  // Loop closure queue as received from candidate generation
  std::queue<pose_graph_msgs::LoopCandidate> input_queue_;

  std::string param_ns_;
};

}  // namespace lamp_loop_closure