/**
 * @file   ObservabilityLoopCandidatePrioritization.h
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */
#pragma once

#include <map>
#include "loop_closure/PointCloudUtils.h"

#include "loop_closure/LoopCandidatePrioritization.h"

namespace lamp_loop_closure {

class ObservabilityLoopCandidatePrioritization
    : public LoopCandidatePrioritization {
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloudConstPtr;
  typedef pcl::PointCloud<pcl::Normal> Normals;

 public:
  ObservabilityLoopCandidatePrioritization();
  ~ObservabilityLoopCandidatePrioritization();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

 protected:
  bool PopulatePriorityQueue() override;

  void PublishBestCandidates() override;

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);

  // Store keyed scans
  std::map<gtsam::Key, PointCloudConstPtr> keyed_scans_;

  // Define subscriber
  ros::Subscriber keyed_scans_sub_;

  // Timer
  double update_rate_;
  ros::Timer update_timer_;

  // Paramters
  size_t publish_n_best_;     // Publish only the top n candidates
  double min_observability_;  // Discard any candidate with observability
                              // below threshold
  double normals_radius_;     // radius used for cloud normal computation
};

}  // namespace lamp_loop_closure