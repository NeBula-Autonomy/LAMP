/**
 * @file   LoopComputation.h
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

class LoopComputation {
 public:
  LoopComputation();
  ~LoopComputation();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  virtual bool LoadParameters(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

 protected:
  // Compute transform and populate output queue
  virtual void ComputeTransforms() = 0;

  void PublishLoopClosures();

  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);

  pose_graph_msgs::PoseGraphEdge CreateLoopClosureEdge(
      const gtsam::Symbol& key1,
      const gtsam::Symbol& key2,
      const geometry_utils::Transform3& delta,
      const gtsam::Matrix66& covariance) const;

  // Define publishers and subscribers
  ros::Publisher loop_closure_pub_;
  ros::Subscriber loop_candidate_sub_;

  // Computed loop closures
  std::vector<pose_graph_msgs::PoseGraphEdge> output_queue_;
  // Loop closure queue as received from candidate generation
  std::queue<pose_graph_msgs::LoopCandidate> input_queue_;
  // Duration (sec) allowed to wait for keyed scans until removed
  double keyed_scans_max_delay_;

  std::string param_ns_;
};

}  // namespace lamp_loop_closure