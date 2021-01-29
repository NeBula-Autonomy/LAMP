#include "utils/PoseGraph.h"
#include "utils/PrefixHandling.h"

double PoseGraph::time_threshold = 1.0;

gtsam::Symbol PoseGraph::GetKeyAtTime(const ros::Time& stamp) const {
  if (stamp_to_odom_key.find(stamp.toSec()) == stamp_to_odom_key.end()) {
    ROS_ERROR("No key exists at given time");
    return utils::GTSAM_ERROR_SYMBOL;
  }
  return stamp_to_odom_key.at(stamp.toSec());
}

gtsam::Symbol PoseGraph::GetClosestKeyAtTime(const ros::Time& stamp,
                                             bool check_threshold) const {
  // If there are no keys, throw an error
  if (stamp_to_odom_key.empty()) {
    ROS_ERROR("Cannot get closest key - no keys are stored");
    return utils::GTSAM_ERROR_SYMBOL;
  }

  // Output key
  gtsam::Symbol key_out;

  // Iterators pointing immediately before and after the target time
  auto iterAfter = stamp_to_odom_key.lower_bound(stamp.toSec());
  auto iterBefore = std::prev(iterAfter);
  double t1 = iterBefore->first;
  double t2 = iterAfter->first;
  double t_closest;

  bool b_is_end_case = false;

  // If time is before the start or after the end, return first/last key
  if (iterAfter == stamp_to_odom_key.begin()) {
    ROS_ERROR("Time stamp before start of range (GetClosestKeyAtTime)");
    key_out = iterAfter->second;
    t_closest = t2;
    b_is_end_case = true;
  } else if (iterAfter == stamp_to_odom_key.end()) {
    ROS_ERROR("Time past end of the range (GetClosestKeyAtTime)");
    key_out = iterBefore->second;
    t_closest = t1;
    b_is_end_case = true;
  }

  if (!b_is_end_case) {
    // Otherwise return the closer key
    if (stamp.toSec() - t1 < t2 - stamp.toSec()) {
      key_out = iterBefore->second;
      t_closest = t1;
    } else {
      key_out = iterAfter->second;
      t_closest = t2;
    }
  }

  // Check threshold
  if (check_threshold && std::abs(t_closest - stamp.toSec()) > time_threshold) {
    ROS_ERROR("Delta between queried time and closest time in graph too large");
    ROS_INFO_STREAM("Time queried is: " << stamp.toSec()
                                        << ", closest time is: " << t_closest);
    ROS_INFO_STREAM("Difference is "
                    << std::abs(stamp.toSec() - t_closest)
                    << ", allowable max is: " << time_threshold);
    key_out = utils::GTSAM_ERROR_SYMBOL;
  } else if (std::abs(t_closest - stamp.toSec()) > time_threshold) {
    ROS_WARN_STREAM("Delta between queried time and closest time in graph too large\n" <<
                    "Time queried is: " << stamp.toSec()
                     << ", closest time is: " << t_closest << "\n Difference is "
                    << std::abs(stamp.toSec() - t_closest)
                    << ", allowable max is: " << time_threshold);
  }

  return key_out;
}

gtsam::Pose3 PoseGraph::LastPose(char c) const {
    gtsam::Key latest = utils::GTSAM_ERROR_SYMBOL; 
    
    // Get the most recent pose from the given robot
    for (auto v : values_) {
      if (gtsam::Symbol(v.key).chr() != c) {
        continue;
      }
      if (latest == utils::GTSAM_ERROR_SYMBOL) {
        latest = v.key;
      }
      else {
        latest = std::max(latest, v.key);
      }
    }

    if (latest == utils::GTSAM_ERROR_SYMBOL) {
      ROS_WARN_STREAM("Could not get latest pose for robot with prefix " << c);
      return gtsam::Pose3();
    }

    return values_.at<gtsam::Pose3>(latest);;
}

const NodeMessage* PoseGraph::FindNode(const gtsam::Key& key) const {
  for (const auto& node : nodes_) {
    if (node.key == key) {
      return &node;
    }
  }
  return nullptr;
}

const EdgeMessage* PoseGraph::FindEdge(const gtsam::Key& key_from,
                                       const gtsam::Key& key_to) const {
  for (const auto& edge : edges_) {
    if (edge.key_from == key_from && edge.key_to == key_to) {
      return &edge;
    }
  }
  return nullptr;
}

const EdgeMessage* PoseGraph::FindPrior(const gtsam::Key& key) const {
  for (const auto& edge : priors_) {
    if (edge.key_from == key) {
      return &edge;
    }
  }
  return nullptr;
}
