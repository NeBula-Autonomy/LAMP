/**
 * @file   ObservabilityLoopPrioritization.h
 * @brief  Prioritize loop closures by observability
 * @author Yun Chang
 */
#pragma once

#include "lamp_utils/PointCloudUtils.h"
#include <gtsam/inference/Symbol.h>
#include <map>
#include <mutex>
#include <pose_graph_msgs/KeyedScan.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <lamp_utils/CommonStructs.h>

#include "loop_closure/LoopPrioritization.h"

namespace lamp_loop_closure {

class ObservabilityLoopPrioritization : public LoopPrioritization {
  typedef pcl::PointCloud<pcl::Normal> Normals;
  friend class TestLoopPrioritization;

public:
  ObservabilityLoopPrioritization();
  ~ObservabilityLoopPrioritization();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

protected:
  void PopulatePriorityQueue() override;

  void PrunePriorityQueue();

  void PublishBestCandidates() override;

  pose_graph_msgs::LoopCandidateArray GetBestCandidates() override;


  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);

  void ProcessTimerCallback(const ros::TimerEvent& ev);

  // Store keyed scans
  std::unordered_map<gtsam::Key, double> keyed_observability_;

  // Store observability in deque along with candidate
  std::deque<double> observability_score_;

  // Track max observability for each robot (different so need to normalize)
  std::unordered_map<char, double> max_observability_;

  // Define subscriber
  ros::Subscriber keyed_scans_sub_;

  // Timer
  ros::Timer update_timer_;

  // Paramters
  int publish_n_best_;       // Publish only the top n candidates
  double min_observability_; // Discard any candidate with observability
                             // below threshold
  double horizon_;           // time until a candidate is discarded

  int num_threads_; // number of threads for normal computation
};

} // namespace lamp_loop_closure