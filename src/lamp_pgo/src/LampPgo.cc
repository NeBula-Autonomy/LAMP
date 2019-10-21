/*
LampPgo.cc
Author: Yun Chang
Interface for ROS and KimeraRPGO
*/

#include "lamp_pgo/LampPgo.h"

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
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing

  // Publisher
  optimized_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("optimized_values", 10, false);
  // TODO - make names uniform? - "optimized_values"(here) =
  // "back_end_pose_graph"(lamp)

  // Subscriber
  input_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_to_optimize", 1, &LampPgo::InputCallback, this);

  // Parse parameters
  // Optimizer backend
  ROS_INFO_STREAM("PGO PROCESS NAMESPACE: " << n.getNamespace());
  std::string full_namespace = n.getNamespace();
  if (full_namespace.find("base_station") != std::string::npos) {
    param_ns_ = "base";
  }
  else {
    param_ns_ = "robot";
  }
  ROS_INFO_STREAM("Chosen namespace: " << param_ns_);

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
        KimeraRPGO::Verbosity::VERBOSE); // set no outlier rejection
  }

  // TODO - have a better way of handling special symbols...
  std::vector<char> special_symbs{
      'l', 'm', 'n', 'o', 'p', 'q', 'u'}; // for artifacts
  rpgo_params_.specialSymbols = special_symbs;

  // set solver
  int solver_num;
  if (!pu::Get(param_ns_ + "/solver", solver_num))
    return false;

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

void LampPgo::InputCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  // Callback for the input posegraph
  NonlinearFactorGraph all_factors, new_factors;
  Values all_values, new_values;

  ROS_INFO_STREAM("PGO received graph of size " << graph_msg->nodes.size());

  // Convert to gtsam type
  utils::PoseGraphMsgToGtsam(graph_msg, &all_factors, &all_values);

  // Extract the new factors
  for (size_t i = 0; i < all_factors.size(); i++) {
    if (std::find(nfg_.begin(), nfg_.end(), all_factors[i]) == nfg_.end()) {
      // this factor does not exist before
      new_factors.add(all_factors[i]);
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

  ROS_INFO_STREAM("PGO adding new values " << new_values.size());
  ROS_INFO_STREAM("PGO adding new factors " << new_factors.size());

  // Run the optimizer
  pgo_solver_->update(new_factors, new_values);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  ROS_INFO_STREAM("PGO stored values of size " << values_.size());

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

  optimized_pub_.publish(pose_graph_msg);
}
