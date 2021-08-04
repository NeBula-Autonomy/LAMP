//
// Created by chris on 6/2/21.
//
#include "loop_closure/ObservabilityQueue.h"
#include <parameter_utils/ParameterUtils.h>
#include <math.h>
#include <limits>

namespace pu = parameter_utils;
namespace lamp_loop_closure {

ObservabilityQueue::ObservabilityQueue() : LoopCandidateQueue() {}
ObservabilityQueue::~ObservabilityQueue() {}

bool ObservabilityQueue::RegisterCallbacks(const ros::NodeHandle& n) {
  if (!LoopCandidateQueue::RegisterCallbacks(n)) { return false; }

  ros::NodeHandle nl(n);
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans",
      100,
      &ObservabilityQueue::KeyedScanCallback,
      this);
  return true;
}


bool ObservabilityQueue::Initialize(const ros::NodeHandle &n) {
  if(!LoopCandidateQueue::Initialize(n)) { return false;}

  key_ = -1;
  return true;
}
bool ObservabilityQueue::LoadParameters(const ros::NodeHandle &n) {
  if(!LoopCandidateQueue::LoadParameters(n)) {return false;}
  if (!pu::Get(param_ns_ + "/obs_prioritization/normals_search_radius",
               normals_radius_))
    return false;
  if (!pu::Get(param_ns_ + "/queue/amount_per_round",
               amount_per_round_)) {return false;}
  if (!pu::Get(param_ns_ + "/obs_prioritization/min_observability",
               min_observability_))
    return false;

  if (!pu::Get(param_ns_ + "/obs_prioritization/threads", num_threads_))
    return false;

    return true;
}

void ObservabilityQueue::FindNextSet() {
  if (!observability_queue_.empty()) {
    pose_graph_msgs::LoopCandidateArray out_array;
    for (int i = 0; i < amount_per_round_; ++i) {
      out_array.candidates.push_back(observability_queue_.top().second);
      //ROS_INFO_STREAM("Queue Popped Scan with observability:" << observability_queue_.top().first);
      observability_queue_.pop();
      if (observability_queue_.empty()) break;
    }
    LoopCandidateQueue::PublishLoopCandidate(out_array);
  }
}
double ObservabilityQueue::ComputeObservability(const pose_graph_msgs::LoopCandidate& candidate){
  // Check if keyed scans exist
  if (keyed_scans_.find(candidate.key_from) == keyed_scans_.end() ||
      keyed_scans_.find(candidate.key_to) == keyed_scans_.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  Eigen::Matrix<double, 3, 1> obs_eigenv_from;
  utils::ComputeIcpObservability(
      keyed_scans_[candidate.key_from], normals_radius_, num_threads_, &obs_eigenv_from);
  double min_obs_from = obs_eigenv_from.minCoeff();

  Eigen::Matrix<double, 3, 1> obs_eigenv_to;
  utils::ComputeIcpObservability(
      keyed_scans_[candidate.key_to], normals_radius_, num_threads_, &obs_eigenv_to);
  double min_obs_to = obs_eigenv_to.minCoeff();

  double score = min_obs_from + min_obs_to;

  return score;
}


void ObservabilityQueue::OnNewLoopClosure() {
  for (auto& cur_queue : queues) {
    while(!cur_queue.second.empty()){
      auto candidate = cur_queue.second.back();
      double score = ComputeObservability(candidate);
      if (isnan(score)) continue; //Retry, this happens when keyed scan isn't found
      cur_queue.second.pop_back();
      if (score >= min_observability_) {
        auto pair = std::make_pair(score, candidate);
        observability_queue_.push(pair);
      } else {
        //ROS_INFO_STREAM("Dropped closure with Observability " << score);
      }
    }
  }
}

void ObservabilityQueue::OnLoopComputationCompleted() {
  FindNextSet();
}

void ObservabilityQueue::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                         << gtsam::DefaultKeyFormatter(key)
                         << " already has a scan. Not adding.");
    return;
  }

  pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}

}
