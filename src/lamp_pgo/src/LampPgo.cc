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
#include <utils/CommonFunctions.h>>

namespace pu = parameter_utils;

LampPgo::Initialize(const ros::NodeHandle& n) {
  // Create subscriber and publisher
  optimized_pub_ =
      n.advertise<pose_graph_msgs::PoseGraph>("optimized_values", 10, false);

  input_sub_ =
      n.subscribe("input_posegraph", 1, &pose_graph_msgs::PoseGraph, this);

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
        trans_threshold, rot_threshold, RobustPGO::Verbosity::VERBOSE);
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
    rpgo_params_.solver = RobustPGO::Solver::LM;
  } else if (solver_num == 2) {
    rpgo_params_.solver = RobustPGO::Solver::GN;
  } else {
    ROS_ERROR("Unsupported solver parameter. Use 1 for LM and 2 for GN");
  }

  // Initialize solver
  pgo_solver_.reset(new RobustPGO::RobustSolver(rpgo_params_));
}

LampPgo::InputCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  // Callback for the input posegraph
  NonlinearFactorGraph all_factors, new_factors;
  Values all_values, new_values;
  // Convert to gtsam type
  utils::PoseGraphMsgToGtsam(graph_msg, &all_factors, &all_values);

  // Extract the new factors
  NonlinearFactorGraph::iterator it;

  for (it = all_factors.begin(); it != all_factors.end(); it++) {
    if (std::find(nfg_.begin(), nfg_.end(), it) == nfg_.end()) {
      // this factor does not exist before
      new_factors.add(all_factors[i]);
    }
  }

  // Extract new values
  Values::iterator it;
  for (it = all_values.begin(); it != all_values.end(); it++) {
    if (std::find(values_.begin(), values_.end(), it) == values_.end()) {
      new_values.insert(it);
    }
  }

  pgo_solver_->updateBatch(
      new_factors, new_values, new_values.keys().front() - 1);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // publish posegraph
  PublishValues(optimized_values);
}
