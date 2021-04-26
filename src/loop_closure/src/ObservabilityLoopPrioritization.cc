/**
 * @file   ObservabilityLoopPrioritization.cc
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */

#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <numeric>
#include "loop_closure/PointCloudUtils.h"

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

  return true;
}

bool ObservabilityLoopPrioritization::LoadParameters(const ros::NodeHandle& n) {
  if (!LoopPrioritization::LoadParameters(n)) return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/publish_n_best",
               publish_n_best_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/min_observability",
               min_observability_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/normals_search_radius",
               normals_radius_))
    return false;

  return true;
}

bool ObservabilityLoopPrioritization::CreatePublishers(
    const ros::NodeHandle& n) {
  if (!LoopPrioritization::CreatePublishers(n)) return false;

  return true;
}

bool ObservabilityLoopPrioritization::RegisterCallbacks(
    const ros::NodeHandle& n) {
  if (!LoopPrioritization::RegisterCallbacks(n)) return false;

  ros::NodeHandle nl(n);
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans",
      100,
      &ObservabilityLoopPrioritization::KeyedScanCallback,
      this);

  update_timer_ = nl.createTimer(
      2.0, &ObservabilityLoopPrioritization::ProcessTimerCallback, this);

  return true;
}

void ObservabilityLoopPrioritization::ProcessTimerCallback(
    const ros::TimerEvent& ev) {
  PopulatePriorityQueue();
  if (priority_queue_.size() > 0 &&
      loop_candidate_pub_.getNumSubscribers() > 0) {
    PublishBestCandidates();
  }
}

void ObservabilityLoopPrioritization::PopulatePriorityQueue() {
  size_t n = candidate_queue_.size();
  for (size_t i = 0; i < n; i++) {
    auto candidate = candidate_queue_.front();
    candidate_queue_.pop();

    // Check if keyed scans exist
    if (keyed_scans_.find(candidate.key_from) == keyed_scans_.end() ||
        keyed_scans_.find(candidate.key_to) == keyed_scans_.end()) {
      continue;
    }

    Eigen::Matrix<double, 3, 1> obs_eigenv_from;
    ComputeIcpObservability(keyed_scans_[candidate.key_from], &obs_eigenv_from);
    double min_obs_from = obs_eigenv_from.minCoeff();
    if (min_obs_from < min_observability_) continue;

    Eigen::Matrix<double, 3, 1> obs_eigenv_to;
    ComputeIcpObservability(keyed_scans_[candidate.key_to], &obs_eigenv_to);
    double min_obs_to = obs_eigenv_to.minCoeff();
    if (min_obs_to < min_observability_) continue;

    double score = min_obs_from + min_obs_to;
    candidate.value = score;
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
  }
  return;
}

void ObservabilityLoopPrioritization::PublishBestCandidates() {
  pose_graph_msgs::LoopCandidateArray output_msg;
  size_t n = priority_queue_.size();
  for (size_t i = 0; i < n; i++) {
    if (i == publish_n_best_) break;
    output_msg.candidates.push_back(priority_queue_.front());
    priority_queue_.pop_front();
  }
  loop_candidate_pub_.publish(output_msg);
}

void ObservabilityLoopPrioritization::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already has a scan. Not adding.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}

void ObservabilityLoopPrioritization::ComputeIcpObservability(
    PointCloud::ConstPtr cloud,
    Eigen::Matrix<double, 3, 1>* eigenvalues) const {
  // Get normals
  Normals::Ptr normals(new Normals);           // pc with normals
  PointCloud::Ptr normalized(new PointCloud);  // pc whose points have been
                                               // rearranged.
  utils::ComputeNormals(cloud, normals_radius_, 1, normals);
  utils::NormalizePCloud(cloud, normalized);

  // Correspondence with itself (not really used anyways)
  std::vector<size_t> c(cloud->size());
  std::iota(std::begin(c), std::end(c), 0);  // Fill with 0, 1, ...

  Eigen::Matrix4f T_unsued = Eigen::Matrix4f::Zero();  // Unused

  Eigen::Matrix<double, 6, 6> Ap;
  // Compute Ap and its eigenvalues
  utils::ComputeAp_ForPoint2PlaneICP(normalized, normals, c, T_unsued, Ap);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(
      Ap.block(3, 3, 3, 3));
  if (eigensolver.info() == Eigen::Success) {
    *eigenvalues = eigensolver.eigenvalues();
  } else {
    ROS_WARN("Failed to decompose observability matrix. ");
  }
}

}  // namespace lamp_loop_closure