/**
 * @file   ProximityLoopCandidateGeneration.cc
 * @brief  Find potentital loop closures based on proximity
 * @author Yun Chang
 */

#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>
#include <string>

#include "loop_closure/ProximityLoopCandidateGeneration.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

ProximityLoopCandidateGeneration::ProximityLoopCandidateGeneration() {}
ProximityLoopCandidateGeneration::~ProximityLoopCandidateGeneration() {}

bool ProximityLoopCandidateGeneration::Initialize(const ros::NodeHandle& n) {
  std::string name =
      ros::names::append(n.getNamespace(), "ProximityLoopCandidateGeneration");
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

bool ProximityLoopCandidateGeneration::LoadParameters(
    const ros::NodeHandle& n) {
  if (!LoopCandidateGeneration::LoadParameters(n)) return false;

  double distance_to_skip_recent_poses, translation_threshold_nodes;
  if (!pu::Get(param_ns_ + "/translation_threshold_nodes",
               translation_threshold_nodes))
    return false;
  if (!pu::Get(param_ns_ + "/proximity_threshold", proximity_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/distance_to_skip_recent_poses",
               distance_to_skip_recent_poses))
    return false;

  skip_recent_poses_ =
      (int)(distance_to_skip_recent_poses / translation_threshold_nodes);
  return true;
}

bool ProximityLoopCandidateGeneration::CreatePublishers(
    const ros::NodeHandle& n) {
  if (!LoopCandidateGeneration::CreatePublishers(n)) return false;
  return true;
}

bool ProximityLoopCandidateGeneration::RegisterCallbacks(
    const ros::NodeHandle& n) {
  if (!LoopCandidateGeneration::RegisterCallbacks(n)) return false;
  return true;
}

double ProximityLoopCandidateGeneration::DistanceBetweenKeys(
    const gtsam::Symbol& key1,
    const gtsam::Symbol& key2) const {
  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);
  const gtsam::Pose3 delta = pose1.between(pose2);

  return delta.translation().norm();
}

void ProximityLoopCandidateGeneration::GenerateLoopCandidates(
    const gtsam::Key& new_key) {
  const gtsam::Symbol key = gtsam::Symbol(new_key);
  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    const gtsam::Symbol other_key = it->first;

    // Don't self-check.
    if (key == other_key) continue;

    // Don't compare against poses that were recently collected.
    if (utils::IsKeyFromSameRobot(key, other_key) &&
        std::llabs(key.index() - other_key.index()) < skip_recent_poses_)
      continue;

    double distance = DistanceBetweenKeys(key, other_key);

    if (distance > proximity_threshold_) {
      continue;
    }

    // If all the checks pass create candidate and add to queue
    pose_graph_msgs::LoopCandidate candidate;
    candidate.key_from = new_key;
    candidate.key_to = other_key;
    candidate.pose_from = utils::GtsamToRosMsg(keyed_poses_[new_key]);
    candidate.pose_to = utils::GtsamToRosMsg(keyed_poses_[other_key]);
    candidate.type = pose_graph_msgs::LoopCandidate::PROXIMITY;
    candidate.value = distance;

    candidates_.push_back(candidate);
  }
  return;
}

}  // namespace lamp_loop_closure