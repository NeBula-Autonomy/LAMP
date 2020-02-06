/*
LampPgo.cc
Author: Yun Chang
Interface for ROS and KimeraRPGO
*/

#include "lamp_pgo/LampPgo.h"

#include <string>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>

#include "pose_graph_msgs/PoseGraphNode.h"

using gtsam::NonlinearFactorGraph;
using gtsam::Values;

namespace pu = parameter_utils;

LampPgo::LampPgo() {}
LampPgo::~LampPgo() {}

bool LampPgo::Initialize(const ros::NodeHandle& n) {
  // Create subscriber and publisher
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing

  // Publisher
  optimized_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("optimized_values", 10, false);
  // TODO - make names uniform? - "optimized_values"(here) =
  // "back_end_pose_graph"(lamp)

  // Subscriber
  input_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_to_optimize", 1, &LampPgo::InputCallback, this);
  remove_lc_sub_ = nl.subscribe<std_msgs::String>(
      "remove_loop_closure", 1, &LampPgo::RemoveLCCallback, this);
  ignore_robot_sub_ = nl.subscribe<std_msgs::String>(
      "ignore_loop_closures", 1, &LampPgo::IgnoreRobotLoopClosures, this);
  revive_robot_sub_ = nl.subscribe<std_msgs::String>(
      "revive_loop_closures", 1, &LampPgo::ReviveRobotLoopClosures, this);

  // Parse parameters
  // Optimizer backend
  ROS_INFO_STREAM("PGO NODE NAMESPACE: " << n.getNamespace());
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  ROS_INFO_STREAM("Parameter namespace: " << param_ns_);

  bool b_use_outlier_rejection;
  if (!pu::Get(param_ns_ + "/b_use_outlier_rejection", b_use_outlier_rejection))
    return false;
  if (b_use_outlier_rejection) {
    // outlier rejection on: set up PCM params
    double trans_threshold, rot_threshold;
    if (!pu::Get(param_ns_ + "/translation_check_threshold", trans_threshold))
      return false;
    if (!pu::Get(param_ns_ + "/rotation_check_threshold", rot_threshold))
      return false;
    rpgo_params_.setPcmSimple3DParams(
        trans_threshold, rot_threshold, KimeraRPGO::Verbosity::VERBOSE);
  } else {
    rpgo_params_.setNoRejection(
        KimeraRPGO::Verbosity::VERBOSE);  // set no outlier rejection
  }

  // Artifact or UWB keys (l, m, n, ... + u)
  rpgo_params_.specialSymbols = utils::GetAllSpecialSymbols();

  // set solver
  int solver_num;
  if (!pu::Get(param_ns_ + "/solver", solver_num)) return false;

  if (solver_num == 1) {
    // Levenberg-Marquardt
    rpgo_params_.solver = KimeraRPGO::Solver::LM;
  } else if (solver_num == 2) {
    // Gauss Newton
    rpgo_params_.solver = KimeraRPGO::Solver::GN;
  } else {
    ROS_ERROR("Unsupported solver parameter. Use 1 for LM and 2 for GN");
  }

  // Initialize solver
  pgo_solver_.reset(new KimeraRPGO::RobustSolver(rpgo_params_));
  return true;
}

void LampPgo::RemoveLastLoopClosure(char prefix_1, char prefix_2) {
  pgo_solver_->removeLastLoopClosure(prefix_1, prefix_2);
  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();
  
  ROS_INFO_STREAM("Removed last loop closure between prefixes "
                  << prefix_1 << " and " << prefix_2);

  // publish posegraph
  PublishValues();
}

void LampPgo::RemoveLCCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data.length() < 2) {
    return;  // needs two prefixes (two chars)
  }
  RemoveLastLoopClosure(msg->data[0], msg->data[1]);
  return;
}

void LampPgo::InputCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  // Callback for the input posegraph
  NonlinearFactorGraph all_factors, new_factors;
  Values all_values, new_values;

  ROS_INFO_STREAM("PGO received graph of size " << graph_msg->nodes.size());

  // Convert to gtsam type
  utils::PoseGraphMsgToGtsam(graph_msg, &all_factors, &all_values);

  // Track node IDs
  for (auto n : graph_msg->nodes) {
    if (!key_to_id_map_.count(n.key)) {
      key_to_id_map_[n.key] = n.ID;
    }
  }

  // Extract new values
  // TODO - use the merger here? In case the state of the graph here is
  // different from the lamp node Will that ever be the case?

  gtsam::KeyVector key_list = all_values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    if (!values_.exists(key_list[i])) {
      new_values.insert(key_list[i], all_values.at(key_list[i]));
    }
  }

  // Extract the new factors
  for (size_t i = 0; i < all_factors.size(); i++) {
    bool factor_exists = false;
    for (size_t j = 0; j < nfg_.size(); j++) {
      if (nfg_[j]->keys() == all_factors[i]->keys()) {
        factor_exists = true;
        break;
      }
    }
    if (!factor_exists) {
      // this factor does not exist before
      new_factors.add(all_factors[i]);
    }
  }

  ROS_INFO_STREAM("PGO adding new values " << new_values.size());
  for (auto k : new_values) {
    ROS_INFO_STREAM("\t" << gtsam::DefaultKeyFormatter(k.key));
  }
  ROS_INFO_STREAM("PGO adding new factors " << new_factors.size());

  // new_factors.print("new factors");

  ROS_INFO_STREAM("FACTORS BEFORE");
  for (auto f : new_factors) {
    f->printKeys();
  }

  // Run the optimizer
  pgo_solver_->update(new_factors, new_values);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // nfg_.print("nfg");
  ROS_INFO_STREAM("FACTORS AFTER");
  for (auto f : nfg_) {
    f->printKeys();
    ROS_INFO_STREAM("Error: " << f->error(values_));
  }

  ROS_INFO_STREAM("PGO stored values of size " << values_.size());
  ROS_INFO_STREAM("PGO stored nfg of size " << nfg_.size());

  // publish posegraph
  PublishValues();
}

// TODO - check that this is ok including just the positions in the message
void LampPgo::PublishValues() const {
  pose_graph_msgs::PoseGraph pose_graph_msg;

  // Then store the values as nodes
  gtsam::KeyVector key_list = values_.keys();
  for (const auto& key : key_list) {
    pose_graph_msgs::PoseGraphNode node;
    node.key = key;
    if (key_to_id_map_.count(key)) {
      node.ID = key_to_id_map_.at(key);
    } else {
      ROS_ERROR_STREAM("PGO: ID not found for node key");
    }
    // pose - translation
    node.pose.position.x = values_.at<gtsam::Pose3>(key).translation().x();
    node.pose.position.y = values_.at<gtsam::Pose3>(key).translation().y();
    node.pose.position.z = values_.at<gtsam::Pose3>(key).translation().z();
    // pose - rotation (to quaternion)
    node.pose.orientation.x =
        values_.at<gtsam::Pose3>(key).rotation().toQuaternion().x();
    node.pose.orientation.y =
        values_.at<gtsam::Pose3>(key).rotation().toQuaternion().y();
    node.pose.orientation.z =
        values_.at<gtsam::Pose3>(key).rotation().toQuaternion().z();
    node.pose.orientation.w =
        values_.at<gtsam::Pose3>(key).rotation().toQuaternion().w();

    pose_graph_msg.nodes.push_back(node);
  }

  ROS_INFO_STREAM("PGO publishing graph with " << pose_graph_msg.nodes.size()
                                               << " values");
  optimized_pub_.publish(pose_graph_msg);
}

void LampPgo::IgnoreRobotLoopClosures(const std_msgs::String::ConstPtr& msg) {
  // First convert string "huskyn" to char prefix
  char prefix = utils::GetRobotPrefix(msg->data);

  pgo_solver_->ignorePrefix(prefix);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  ROS_INFO_STREAM("Ignoring all loop closures involving prefix " << prefix);

  // publish posegraph
  PublishValues();
}

void LampPgo::ReviveRobotLoopClosures(const std_msgs::String::ConstPtr& msg) {
  // First convert string "huskyn" to char prefix
  char prefix = utils::GetRobotPrefix(msg->data);

  pgo_solver_->revivePrefix(prefix);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  ROS_INFO_STREAM("Reviving all loop closures involving prefix " << prefix);

  // publish posegraph
  PublishValues();
}
