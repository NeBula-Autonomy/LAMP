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
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  optimized_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("optimized_values", 10, false);

  input_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "input_posegraph", 1, &LampPgo::InputCallback, this);

  // Parse parameters
  // Optimizer backend
  bool b_use_outlier_rejection;
  if (!pu::Get("b_use_outlier_rejection", b_use_outlier_rejection))
    return false;
  if (b_use_outlier_rejection) {
    // outlier rejection on: set up PCM params
    double trans_threshold, rot_threshold;
    if (!pu::Get("translation_check_threshold", trans_threshold)) return false;
    if (!pu::Get("rotation_check_threshold", rot_threshold)) return false;
    rpgo_params_.setPcmSimple3DParams(
        trans_threshold, rot_threshold, KimeraRPGO::Verbosity::VERBOSE);
  } else {
    rpgo_params_.setNoRejection(
        KimeraRPGO::Verbosity::VERBOSE);  // set no outlier rejection
  }
  std::vector<char> special_symbs{
      'l', 'm', 'n', 'o', 'p', 'q', 'u'};  // for artifacts
  rpgo_params_.specialSymbols = special_symbs;

  // set solver
  int solver_num;
  if (!pu::Get("solver", solver_num)) return false;
  if (solver_num == 1) {
    rpgo_params_.solver = KimeraRPGO::Solver::LM;
  } else if (solver_num == 2) {
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
  gtsam::KeyVector key_list = all_values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    if (!values_.exists(key_list[i])) {
      new_values.insert(key_list[i], all_values.at(key_list[i]));
    }
  }

  pgo_solver_->updateBatch(
      new_factors, new_values, new_values.keys().front() - 1);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // publish posegraph
  PublishValues();
}

void LampPgo::PublishValues() const {
  pose_graph_msgs::PoseGraph pose_graph_msg;

  // Then store the values as nodes
  gtsam::KeyVector key_list = values_.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    pose_graph_msgs::PoseGraphNode node;
    node.key = key_list[i];
    // pose - translation
    node.pose.position.x = values_.at<gtsam::Pose3>(i).translation().x();
    node.pose.position.y = values_.at<gtsam::Pose3>(i).translation().y();
    node.pose.position.z = values_.at<gtsam::Pose3>(i).translation().z();
    // pose - rotation (to quaternion)
    node.pose.orientation.x =
        values_.at<gtsam::Pose3>(i).rotation().toQuaternion().x();
    node.pose.orientation.y =
        values_.at<gtsam::Pose3>(i).rotation().toQuaternion().y();
    node.pose.orientation.z =
        values_.at<gtsam::Pose3>(i).rotation().toQuaternion().z();
    node.pose.orientation.w =
        values_.at<gtsam::Pose3>(i).rotation().toQuaternion().w();

    pose_graph_msg.nodes.push_back(node);
  }

  optimized_pub_.publish(pose_graph_msg);
}
