/**
 * @file   GenericLoopPrioritization.h
 * @brief  Prioritize loop closures by the order received
 * @author Yun Chang
 */
#pragma once

#include "loop_closure/PointCloudUtils.h"
#include <gtsam/inference/Symbol.h>
#include <map>
#include <pose_graph_msgs/KeyedScan.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <unordered_map>
#include <utils/CommonStructs.h>

#include "loop_closure/LoopPrioritization.h"

namespace lamp_loop_closure {

class GenericLoopPrioritization : public LoopPrioritization {
  typedef pcl::PointCloud<pcl::Normal> Normals;
  friend class TestLoopPrioritization;

public:
  GenericLoopPrioritization();
  ~GenericLoopPrioritization();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;


protected:
    void PopulatePriorityQueue() override;

  void PublishBestCandidates() override;

  pose_graph_msgs::LoopCandidateArray GetBestCandidates() override;

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);

  void ProcessTimerCallback(const ros::TimerEvent& ev);

  // Store keyed scans
  std::unordered_map<gtsam::Key, double> keyed_observability_;

  // Define subscriber
  ros::Subscriber keyed_scans_sub_;

  // Timer
  ros::Timer update_timer_;

  // Paramters
  double min_observability_; // Discard any candidate with observability
                             // below threshold
  double normals_radius_;    // radius used for cloud normal computation

  int num_threads_; // number of threads for normal computation

  bool choose_best_; // Send only best candidate
};

} // namespace lamp_loop_closure