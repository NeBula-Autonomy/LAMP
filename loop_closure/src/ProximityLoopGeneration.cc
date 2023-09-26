/**
 * @file   ProximityLoopGeneration.cc
 * @brief  Find potentital loop closures based on proximity
 * @author Yun Chang
 */

#include <parameter_utils/ParameterUtils.h>
#include <string>
#include <lamp_utils/CommonFunctions.h>

#include "loop_closure/ProximityLoopGeneration.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

ProximityLoopGeneration::ProximityLoopGeneration() : LoopGeneration() {}
ProximityLoopGeneration::~ProximityLoopGeneration() {}

bool ProximityLoopGeneration::Initialize(const ros::NodeHandle& n) {
  std::string name =
      ros::names::append(n.getNamespace(), "ProximityLoopGeneration");
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

bool ProximityLoopGeneration::LoadParameters(const ros::NodeHandle& n) {
  if (!LoopGeneration::LoadParameters(n))
    return false;

  if (!pu::Get(param_ns_ + "/proximity_threshold_max",
               proximity_threshold_max_))
    return false;
  if (!pu::Get(param_ns_ + "/proximity_threshold_min",
               proximity_threshold_min_))
    return false;
  if (!pu::Get(param_ns_ + "/increase_rate", increase_rate_))
    return false;

  if (!pu::Get(param_ns_ + "/n_closest", n_closest_))
    return false;

  bool b_take_n_closest;
  if (!pu::Get(param_ns_ + "/b_take_n_closest", b_take_n_closest))
    return false;
  if (!b_take_n_closest)
    n_closest_ = std::numeric_limits<int>::max();

  double distance_to_skip_recent_poses, translation_threshold_nodes;
  if (!pu::Get(param_ns_ + "/translation_threshold_nodes",
               translation_threshold_nodes))
    return false;
  if (!pu::Get(param_ns_ + "/distance_to_skip_recent_poses",
               distance_to_skip_recent_poses))
    return false;

  skip_recent_poses_ =
      (int)(distance_to_skip_recent_poses / translation_threshold_nodes);
  return true;
}

bool ProximityLoopGeneration::CreatePublishers(const ros::NodeHandle& n) {
  if (!LoopGeneration::CreatePublishers(n))
    return false;
  return true;
}

bool ProximityLoopGeneration::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph", 100000, &ProximityLoopGeneration::KeyedPoseCallback, this);
  return true;
}

double
ProximityLoopGeneration::DistanceBetweenKeys(const gtsam::Symbol& key1,
                                             const gtsam::Symbol& key2) const {
  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);
  const gtsam::Pose3 delta = pose1.between(pose2);

  return delta.translation().norm();
}

void ProximityLoopGeneration::GenerateLoops(const gtsam::Key& new_key) {
  // Loop closure off. No candidates generated
  if (!b_check_for_loop_closures_)
    return;

  const gtsam::Symbol key = gtsam::Symbol(new_key);
  std::vector<pose_graph_msgs::LoopCandidate> potential_candidates;
  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    const gtsam::Symbol other_key = it->first;

    // Don't self-check.
    if (key == other_key)
      continue;

    // Don't compare against poses that were recently collected.
    if (lamp_utils::IsKeyFromSameRobot(key, other_key) &&
        std::llabs(key.index() - other_key.index()) < skip_recent_poses_)
      continue;

    double distance = DistanceBetweenKeys(key, other_key);
    double radius;
    if (lamp_utils::IsKeyFromSameRobot(key, other_key)) {
      radius = std::max(
          0.0,
          std::min(proximity_threshold_max_,
                   (key.index() - other_key.index()) * increase_rate_));
    } else {
      radius = std::max(
          proximity_threshold_min_,
          std::min(proximity_threshold_max_, key.index() * increase_rate_));
    }

    if (distance > radius) {
      continue;
    }

    // If all the checks pass create candidate and add to queue
    pose_graph_msgs::LoopCandidate candidate;
    candidate.header.stamp = ros::Time::now();
    candidate.key_from = new_key;
    candidate.key_to = other_key;
    candidate.pose_from = lamp_utils::GtsamToRosMsg(keyed_poses_[new_key]);
    candidate.pose_to = lamp_utils::GtsamToRosMsg(keyed_poses_[other_key]);
    candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
    candidate.value = distance;

    potential_candidates.push_back(candidate);
  }
  if (potential_candidates.size() < n_closest_) {
    candidates_.insert(candidates_.end(),
                       potential_candidates.begin(),
                       potential_candidates.end());
  } else {
    sort(potential_candidates.begin(),
         potential_candidates.end(),
         [](const pose_graph_msgs::LoopCandidate& lhs,
            const pose_graph_msgs::LoopCandidate& rhs) {
           return lhs.value < rhs.value;
         });
    candidates_.insert(candidates_.end(),
                       potential_candidates.begin(),
                       potential_candidates.begin() + n_closest_);
  }
  return;
}

void ProximityLoopGeneration::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  pose_graph_msgs::PoseGraphNode node_msg;
  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Symbol node_key = gtsam::Symbol(node_msg.key); // extract new key
    ros::Time timestamp = node_msg.header.stamp; // extract new timestamp

    if (!lamp_utils::IsRobotPrefix(node_key.chr())) {
      continue;
    }

    gtsam::Pose3 node_pose;
    gtsam::Point3 pose_translation(node_msg.pose.position.x,
                                   node_msg.pose.position.y,
                                   node_msg.pose.position.z);
    gtsam::Rot3 pose_orientation(node_msg.pose.orientation.w,
                                 node_msg.pose.orientation.x,
                                 node_msg.pose.orientation.y,
                                 node_msg.pose.orientation.z);
    node_pose = gtsam::Pose3(pose_orientation, pose_translation);

    // Check if the node is new
    if (keyed_poses_.count(node_key) > 0) {
      // add new key and pose to keyed_poses_
      keyed_poses_[node_key] = node_pose;
      continue; // Not a new node
    }

    keyed_poses_.insert({node_key, node_pose});
    GenerateLoops(node_key);
  }

  if (loop_candidate_pub_.getNumSubscribers() > 0 && candidates_.size() > 0) {
    PublishLoops();
    ClearLoops();
  }
  return;
}

} // namespace lamp_loop_closure