/**
 * @file   LoopCandidateQueue.h
 * @brief  Consolidation queue for loop closure candidates before computing
 * transform
 * @author Yun Chang
 */
#pragma once

#include "loop_closure/LoopCandidateQueue.h"
#include "utils/PointCloudUtils.h"
#include <deque>
#include <gtsam/inference/Symbol.h>
#include <map>
#include <pose_graph_msgs/KeyedScan.h>
#include <queue>
#include <utils/CommonStructs.h>
#include <vector>

namespace lamp_loop_closure {

class ObservabilityQueue : public LoopCandidateQueue  {
  typedef pcl::PointCloud<pcl::Normal> Normals;

 public:
  ObservabilityQueue();
  ~ObservabilityQueue();

  virtual bool Initialize(const ros::NodeHandle& n) override;

  virtual bool LoadParameters(const ros::NodeHandle& n) override;

  virtual bool RegisterCallbacks(const ros::NodeHandle& n) override;


  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);
 protected:
//  void InputCallback(
//      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) override;

  void LoopComputationStatusCallback(const pose_graph_msgs::LoopComputationStatus::ConstPtr& status);


  virtual void OnNewLoopClosure();

  virtual void OnLoopComputationCompleted();

  double ComputeObservability(const pose_graph_msgs::LoopCandidate& candidate);

  void FindNextSet();
  int key_;
  int amount_per_round_;
  double normals_radius_;     // radius used for cloud normal computation
  double min_observability_;
  int num_threads_;

  ros::Subscriber keyed_scans_sub_;

  // Store keyed scans
  std::map<gtsam::Key, PointCloudConstPtr> keyed_scans_;

  struct ObservabilityCompare
  {
    bool operator()(const std::pair<float,pose_graph_msgs::LoopCandidate>& lhs, const std::pair<float,pose_graph_msgs::LoopCandidate>& rhs)
    {
      return lhs.first < rhs.first;
    }
  };

  std::priority_queue<std::pair<float,pose_graph_msgs::LoopCandidate>,std::vector<std::pair<float,pose_graph_msgs::LoopCandidate>>,ObservabilityCompare> observability_queue_;
};
}  // namespace lamp_loop_closure