/**
 * @file   GenericLoopPrioritization.cc
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */

#include "lamp_utils/PointCloudUtils.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <numeric>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>

#include "loop_closure/GenericLoopPrioritization.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

GenericLoopPrioritization::GenericLoopPrioritization() {}
GenericLoopPrioritization::~GenericLoopPrioritization() {}

bool GenericLoopPrioritization::Initialize(const ros::NodeHandle& n) {
  std::string name =
      ros::names::append(n.getNamespace(), "GenericLoopPrioritization");
  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name.c_str());
    return false;
  }

  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name.c_str());
    return false;
  }

  ROS_INFO_STREAM("Initialized GenericLoopPrioritization."
                  << "\nchoose_best: " << choose_best_
                  << "\nmin_observability: " << min_observability_);

  return true;
}

bool GenericLoopPrioritization::LoadParameters(const ros::NodeHandle& n) {
  if (!LoopPrioritization::LoadParameters(n))
    return false;

  if (!pu::Get(param_ns_ + "/keyed_scans_max_delay", keyed_scans_max_delay_))
    return false;

  if (!pu::Get(param_ns_ + "/gen_prioritization/min_observability",
               min_observability_))
    return false;

  if (!pu::Get(param_ns_ + "/gen_prioritization/choose_best", choose_best_))
    return false;

  return true;
}

bool GenericLoopPrioritization::CreatePublishers(const ros::NodeHandle& n) {
  if (!LoopPrioritization::CreatePublishers(n))
    return false;

  return true;
}

bool GenericLoopPrioritization::RegisterCallbacks(const ros::NodeHandle& n) {
  if (!LoopPrioritization::RegisterCallbacks(n))
    return false;

  ros::NodeHandle nl(n);
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans", 100000, &GenericLoopPrioritization::KeyedScanCallback, this);

  update_timer_ =
      nl.createTimer(ros::Duration(0.1),
                     &GenericLoopPrioritization::ProcessTimerCallback,
                     this);

  return true;
}

void GenericLoopPrioritization::ProcessTimerCallback(
    const ros::TimerEvent& ev) {
  if (priority_queue_.size() > 0 &&
      loop_candidate_pub_.getNumSubscribers() > 0) {
    PublishBestCandidates();
  }
}

void GenericLoopPrioritization::PopulatePriorityQueue() {
  size_t n = candidate_queue_.size();
  if (n == 0)
    return;
  auto best_candidate = candidate_queue_.front();
  double best_score = 0;
  for (size_t i = 0; i < n; i++) {
    auto candidate = candidate_queue_.front();
    candidate_queue_.pop();

    // Check if keyed scans exist
    if (keyed_observability_.find(candidate.key_from) ==
            keyed_observability_.end() ||
        keyed_observability_.find(candidate.key_to) ==
            keyed_observability_.end()) {
      ROS_WARN("Keyed scans not yet received. ");
      if ((ros::Time::now() - candidate.header.stamp).toSec() <
          keyed_scans_max_delay_)
        candidate_queue_.push(candidate);
      continue;
    }

    if (keyed_observability_[candidate.key_from] < min_observability_)
      continue;

    if (keyed_observability_[candidate.key_to] < min_observability_)
      continue;

    if (!choose_best_) {
      priority_queue_mutex_.lock();
      priority_queue_.push_back(candidate);
      priority_queue_mutex_.unlock();
    }
    // Track best candidate
    double score = keyed_observability_[candidate.key_from] +
        keyed_observability_[candidate.key_to];
    if (score > best_score) {
      best_candidate = candidate;
      best_score = score;
    }
  }
  if (choose_best_ && best_score > 0) {
      priority_queue_mutex_.lock();
      priority_queue_.push_back(best_candidate);
      priority_queue_mutex_.unlock();
  }

  return;
}

void GenericLoopPrioritization::PublishBestCandidates() {
  pose_graph_msgs::LoopCandidateArray output_msg = GetBestCandidates();
  loop_candidate_pub_.publish(output_msg);
}

pose_graph_msgs::LoopCandidateArray
GenericLoopPrioritization::GetBestCandidates() {
  pose_graph_msgs::LoopCandidateArray output_msg;
  output_msg.originator = 0;

  priority_queue_mutex_.lock();
  size_t n = priority_queue_.size();
  for (size_t i = 0; i < n; i++) {
    output_msg.candidates.push_back(priority_queue_.front());
    priority_queue_.pop_front();
    priority_queue_mutex_.unlock();
  }

  priority_queue_mutex_.unlock();
  return output_msg;
}

void GenericLoopPrioritization::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_observability_.find(key) != keyed_observability_.end()) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already has a scan. Not adding.");
    return;
  }

  pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  Eigen::Matrix<double, 3, 1> obs_eigenv;
  lamp_utils::ComputeIcpObservability(scan, &obs_eigenv);
  double min_obs = obs_eigenv.minCoeff();
  // Add the key and observability
  keyed_observability_.insert(std::pair<gtsam::Key, double>(key, min_obs));
}

} // namespace lamp_loop_closure