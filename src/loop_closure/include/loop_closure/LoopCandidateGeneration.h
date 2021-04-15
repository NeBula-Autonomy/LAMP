/**
 * @file   LoopCandidateGeneration.h
 * @brief  Base class for classes to find potentital loop closures
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <map>
#include <queue>
#include <vector>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopCandidateGeneration {
 public:
  LoopCandidateGeneration();
  ~LoopCandidateGeneration();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  inline pose_graph_msgs::PoseGraphEdge GetCandidate() {
    auto e = candidates_.front();
    candidates_.pop();
    return e;
  }

 protected:
  // Key -> odometry-pose
  std::unordered_map<gtsam::Key, gtsam::Pose3> keyed_poses_;
  // Possible loop closure candidates along with their odometry transform
  std::queue<pose_graph_msgs::PoseGraphEdge> candidates_;

  // define publishers and subscribers
  ros::Publisher loop_closure_pub_;
  ros::Subscriber keyed_poses_sub_;

  virtual bool GenerateLoopCandidates(const gtsam::Key& new_key) = 0;

  void KeyedPoseCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

  std::string param_ns_;
};

}  // namespace lamp_loop_closure