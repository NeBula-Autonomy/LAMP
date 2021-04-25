/**
 * @file   LoopGeneration.h
 * @brief  Base class for classes to find potentital loop closures
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <map>
#include <queue>
#include <vector>

#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopGeneration {
 public:
  LoopGeneration();
  ~LoopGeneration();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  virtual bool LoadParameters(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  virtual bool RegisterCallbacks(const ros::NodeHandle& n) = 0;

 protected:
  // Key -> odometry-pose
  std::map<gtsam::Key, gtsam::Pose3> keyed_poses_;
  // Possible loop closure candidates along with their odometry transform
  std::vector<pose_graph_msgs::LoopCandidate> candidates_;

  // Define publishers and subscribers
  ros::Publisher loop_candidate_pub_;
  ros::Subscriber keyed_poses_sub_;

  virtual void KeyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) = 0;

  inline void PublishLoops() const {
    pose_graph_msgs::LoopCandidateArray candidates_msg;
    candidates_msg.candidates = candidates_;
    loop_candidate_pub_.publish(candidates_msg);
  }

  inline void ClearLoops() { candidates_.clear(); }

  std::string param_ns_;
};

}  // namespace lamp_loop_closure