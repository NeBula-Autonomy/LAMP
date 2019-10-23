#include "utils/CommonStructs.h"

double PoseGraph::time_threshold = 1.0;

gtsam::Symbol PoseGraph::GetKeyAtTime(const ros::Time& stamp) const {
  if (stamp_to_odom_key.find(stamp.toSec()) == stamp_to_odom_key.end()) {
    ROS_ERROR("No key exists at given time");
    return gtsam::Symbol();
  }

  return stamp_to_odom_key.at(stamp.toSec());
}

gtsam::Symbol PoseGraph::GetClosestKeyAtTime(const ros::Time& stamp,
                                             bool check_threshold) const {
  // If there are no keys, throw an error
  if (stamp_to_odom_key.size() == 0) {
    ROS_ERROR("Cannot get closest key - no keys are stored");
    return gtsam::Symbol();
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
    key_out = gtsam::Symbol();
  }

  return key_out;
}
