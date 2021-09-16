/**
 * @file   ObservabilityLoopPrioritization.cc
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */

#include "utils/PointCloudUtils.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <numeric>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>

#include "loop_closure/ObservabilityLoopPrioritization.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

ObservabilityLoopPrioritization::ObservabilityLoopPrioritization() {}
ObservabilityLoopPrioritization::~ObservabilityLoopPrioritization() {}

bool ObservabilityLoopPrioritization::Initialize(const ros::NodeHandle& n) {
  std::string name =
      ros::names::append(n.getNamespace(), "ObservabilityLoopPrioritization");
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

  ROS_INFO_STREAM("Initialized ObservabilityLoopPrioritization."
                  << "\npublish_n_best: " << publish_n_best_
                  << "\nmin_observability: " << min_observability_);

  return true;
}

bool ObservabilityLoopPrioritization::LoadParameters(const ros::NodeHandle& n) {
  if (!LoopPrioritization::LoadParameters(n))
    return false;

  if (!pu::Get(param_ns_ + "/keyed_scans_max_delay", keyed_scans_max_delay_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/publish_n_best",
               publish_n_best_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/min_observability",
               min_observability_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/horizon", horizon_))
    return false;

  return true;
}

bool ObservabilityLoopPrioritization::CreatePublishers(
    const ros::NodeHandle& n) {
  if (!LoopPrioritization::CreatePublishers(n))
    return false;

  return true;
}

bool ObservabilityLoopPrioritization::RegisterCallbacks(
    const ros::NodeHandle& n) {
  if (!LoopPrioritization::RegisterCallbacks(n))
    return false;

  ros::NodeHandle nl(n);
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans",
      100000,
      &ObservabilityLoopPrioritization::KeyedScanCallback,
      this);

  update_timer_ =
      nl.createTimer(ros::Duration(1.0),
                     &ObservabilityLoopPrioritization::ProcessTimerCallback,
                     this);

  return true;
}

void ObservabilityLoopPrioritization::ProcessTimerCallback(
    const ros::TimerEvent& ev) {
  //ROS_INFO_STREAM("Priority Queue Size:" << priority_queue_.size());
  if (priority_queue_.size() > 0 &&
      loop_candidate_pub_.getNumSubscribers() > 0) {
    PrunePriorityQueue();
    PublishBestCandidates();
  }
}

void ObservabilityLoopPrioritization::PopulatePriorityQueue() {
  if (keyed_observability_.size() == 0) {
    ROS_WARN("No keyed scans received yet. Not populating priority queue.");
    return;
  }
  size_t n = candidate_queue_.size();
  if (n > 0) {
    ROS_INFO("ObservabilityLoopPrioritization: Reveived %d loop candidates", n);
  }
  size_t added = 0;
  for (size_t i = 0; i < n; i++) {
    auto candidate = candidate_queue_.front();

    // Check if keyed scans exist
    if (keyed_observability_.count(candidate.key_from) == 0 ||
        keyed_observability_.count(candidate.key_to) == 0) {
      ROS_DEBUG("Keyed scans do not exist and observability score not yet "
                "calculated. ");
      if ((ros::Time::now() - candidate.header.stamp).toSec() <
          keyed_scans_max_delay_) {
        candidate_queue_.push(candidate);
        candidate_queue_.pop();
      }
      continue;
    }

    candidate_queue_.pop();
    double min_obs_from = keyed_observability_[candidate.key_from];
    if (min_obs_from < min_observability_)
      continue;

    double min_obs_to = keyed_observability_[candidate.key_to];
    if (min_obs_to < min_observability_)
      continue;

    double score = min_obs_from + min_obs_to;

    candidate.value = score;
    priority_queue_mutex_.lock();
    std::deque<double>::iterator score_it = observability_score_.begin();
    std::deque<pose_graph_msgs::LoopCandidate>::iterator candidate_it =
        priority_queue_.begin();
    for (size_t i = 0; i < priority_queue_.size(); i++) {
      if (score < observability_score_[i]) {
        ++score_it;
        ++candidate_it;
      } else {
        break;
      }
    }
    observability_score_.insert(score_it, score);
    priority_queue_.insert(candidate_it, candidate);
    added++;
    priority_queue_mutex_.unlock();
  }
  if (added > 0) {
    ROS_INFO(
        "ObservabilityLoopPrioritization: Added %d loop candidates to queue",
        added);
  }
  return;
}

void ObservabilityLoopPrioritization::PrunePriorityQueue() {
  static int old_size = 0;
  if (priority_queue_.size() != old_size) {
    ROS_DEBUG_STREAM(
        "Prune priority queue... size: " << priority_queue_.size());
    old_size = priority_queue_.size();
  }
  priority_queue_mutex_.lock();
  std::deque<double> temp_observability_score;
  std::deque<pose_graph_msgs::LoopCandidate> temp_priority_queue;
  for (size_t i = 0; i < priority_queue_.size(); i++) {
    auto c = priority_queue_[i];
    if (c.header.stamp.toSec() + horizon_ > ros::Time::now().toSec()) {
      temp_observability_score.push_back(observability_score_[i]);
      temp_priority_queue.push_back(priority_queue_[i]);
    }
  }
  observability_score_ = temp_observability_score;
  priority_queue_ = temp_priority_queue;
  priority_queue_mutex_.unlock();
  ROS_DEBUG_STREAM(
      "Discarded old measurements. size: " << priority_queue_.size());
  return;
}

void ObservabilityLoopPrioritization::PublishBestCandidates() {
  pose_graph_msgs::LoopCandidateArray output_msg = GetBestCandidates();
  ROS_DEBUG("Published %d prioritized candidates. ",
            output_msg.candidates.size());
  loop_candidate_pub_.publish(output_msg);
}

pose_graph_msgs::LoopCandidateArray
ObservabilityLoopPrioritization::GetBestCandidates() {
  pose_graph_msgs::LoopCandidateArray output_msg;
  output_msg.originator = 2;
  priority_queue_mutex_.lock();
  size_t n = priority_queue_.size();
  for (size_t i = 0; i < n; i++) {
    if (i == publish_n_best_)
      break;
    output_msg.candidates.push_back(priority_queue_.front());
    priority_queue_.pop_front();
  }
  priority_queue_mutex_.unlock();
  return output_msg;
}

void ObservabilityLoopPrioritization::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_observability_.count(key) > 0) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already processed. Not adding.");
    return;
  }

  pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  char prefix = gtsam::Symbol(key).chr();
  Eigen::Matrix<double, 3, 1> obs_eigenv;
  utils::ComputeIcpObservability(scan, &obs_eigenv);
  double obs_normalized =
      obs_eigenv.minCoeff() / static_cast<double>(scan->size());
  if (max_observability_.count(prefix) == 0 ||
      max_observability_[prefix] < obs_normalized)
    max_observability_[prefix] = obs_normalized;
  double observability = obs_normalized / max_observability_[prefix];

  keyed_observability_.insert(
      std::pair<gtsam::Key, double>(key, observability));
}

} // namespace lamp_loop_closure