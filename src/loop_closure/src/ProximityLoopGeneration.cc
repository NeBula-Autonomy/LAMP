/**
 * @file   ProximityLoopGeneration.cc
 * @brief  Find potentital loop closures based on proximity
 * @author Yun Chang
 */

#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>
#include <string>

#include "loop_closure/ProximityLoopGeneration.h"

namespace pu = parameter_utils;

namespace lamp_loop_closure {

ProximityLoopGeneration::ProximityLoopGeneration() {}
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
  if (!LoopGeneration::LoadParameters(n)) return false;

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

bool ProximityLoopGeneration::CreatePublishers(const ros::NodeHandle& n) {
  if (!LoopGeneration::CreatePublishers(n)) return false;
  return true;
}

bool ProximityLoopGeneration::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental",
      100,
      &ProximityLoopGeneration::KeyedPoseCallback,
      this);
  return true;
}

double ProximityLoopGeneration::DistanceBetweenKeys(
    const gtsam::Symbol& key1,
    const gtsam::Symbol& key2) const {
  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);
  const gtsam::Pose3 delta = pose1.between(pose2);

  return delta.translation().norm();
}

void ProximityLoopGeneration::GenerateLoops(const gtsam::Key& new_key) {
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

void ProximityLoopGeneration::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  pose_graph_msgs::PoseGraphNode node_msg;
  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Key new_key = node_msg.key;            // extract new key
    ros::Time timestamp = node_msg.header.stamp;  // extract new timestamp

    // Check if the node is new
    if (keyed_poses_.count(new_key) > 0) {
      continue;  // Not a new node
    }

    // also extract poses (NOTE(Yun) this pose will not be updated...)
    gtsam::Pose3 new_pose;
    gtsam::Point3 pose_translation(node_msg.pose.position.x,
                                   node_msg.pose.position.y,
                                   node_msg.pose.position.z);
    gtsam::Rot3 pose_orientation(node_msg.pose.orientation.w,
                                 node_msg.pose.orientation.x,
                                 node_msg.pose.orientation.y,
                                 node_msg.pose.orientation.z);
    new_pose = gtsam::Pose3(pose_orientation, pose_translation);

    // add new key and pose to keyed_poses_
    keyed_poses_[new_key] = new_pose;

    GenerateLoops(new_key);
  }

  if (loop_candidate_pub_.getNumSubscribers() > 0) {
    PublishLoops();
    ClearLoops();
  }
  return;
}

}  // namespace lamp_loop_closure