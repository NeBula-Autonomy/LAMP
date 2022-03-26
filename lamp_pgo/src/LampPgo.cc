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
#include <lamp_utils/CommonFunctions.h>

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
  ignored_list_pub_ =
      nl.advertise<std_msgs::String>("ignored_robots", 10, true);

  // Subscriber
  input_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_to_optimize", 1, &LampPgo::InputCallback, this);
  remove_lc_sub_ = nl.subscribe<std_msgs::Bool>(
      "remove_loop_closure", 1, &LampPgo::RemoveLCCallback, this);
  remove_lc_by_id_sub_ = nl.subscribe<std_msgs::String>(
      "remove_loop_closure_by_id", 1, &LampPgo::RemoveLCByIdCallback, this);
  ignore_robot_sub_ = nl.subscribe<std_msgs::String>(
      "ignore_loop_closures", 1, &LampPgo::IgnoreRobotLoopClosures, this);
  revive_robot_sub_ = nl.subscribe<std_msgs::String>(
      "revive_loop_closures", 1, &LampPgo::ReviveRobotLoopClosures, this);
  reset_sub_ =
      nl.subscribe<std_msgs::Bool>("reset", 1, &LampPgo::ResetCallback, this);

  // Parse parameters
  // Optimizer backend
  ROS_INFO_STREAM("PGO NODE NAMESPACE: " << n.getNamespace());
  param_ns_ = lamp_utils::GetParamNamespace(n.getNamespace());
  ROS_INFO_STREAM("Parameter namespace: " << param_ns_);

  bool b_use_outlier_rejection;
  if (!pu::Get(param_ns_ + "/b_use_outlier_rejection", b_use_outlier_rejection))
    return false;
  if (b_use_outlier_rejection) {
    // outlier rejection on: set up PCM params
    double trans_threshold, rot_threshold, gnc_alpha;
    if (!pu::Get(param_ns_ + "/translation_check_threshold", trans_threshold))
      return false;
    if (!pu::Get(param_ns_ + "/rotation_check_threshold", rot_threshold))
      return false;
    if (!pu::Get(param_ns_ + "/gnc_alpha", gnc_alpha)) return false;
    bool b_gnc_bias_odom;
    if (!pu::Get(param_ns_ + "/b_gnc_bias_odom", b_gnc_bias_odom))
      return false;
    rpgo_params_.setPcmSimple3DParams(
        trans_threshold, rot_threshold, KimeraRPGO::Verbosity::VERBOSE);
    if (gnc_alpha > 0 && gnc_alpha < 1) {
      rpgo_params_.setGncInlierCostThresholdsAtProbability(gnc_alpha);
      if (b_gnc_bias_odom)
        rpgo_params_.biasOdometryGnc();
    }
  } else {
    rpgo_params_.setNoRejection(
        KimeraRPGO::Verbosity::VERBOSE);  // set no outlier rejection
  }

  // Artifact or UWB keys (l, m, n, ... + u)
  rpgo_params_.specialSymbols = lamp_utils::GetAllSpecialSymbols();

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
  // Use incremental max clique
  rpgo_params_.setIncremental();

  if (!pu::Get(param_ns_ + "/max_lc_error", max_lc_error_))
    return false;

  std::string log_path;
  if (pu::Get("log_path", log_path)) {
    rpgo_params_.logOutput(log_path);
    ROS_INFO("Enabled logging in Kimera-RPGO");
  }
  // Initialize solver
  pgo_solver_.reset(new KimeraRPGO::RobustSolver(rpgo_params_));

  // Publish ignored list once
  PublishIgnoredList();

  return true;
}

void LampPgo::RemoveLastLoopClosure(char prefix_1, char prefix_2) {
  KimeraRPGO::EdgePtr removed_edge =
      pgo_solver_->removeLastLoopClosure(prefix_1, prefix_2);
  if (removed_edge != NULL) {
    // Extract the optimized values
    values_ = pgo_solver_->calculateEstimate();
    nfg_ = pgo_solver_->getFactorsUnsafe();

    ROS_INFO_STREAM("Removed last loop closure between "
                    << gtsam::DefaultKeyFormatter(removed_edge->from_key)
                    << " and "
                    << gtsam::DefaultKeyFormatter(removed_edge->to_key));

    // publish posegraph
    PublishValues();
  } else {
    ROS_WARN("No more loop closure to remove");
  }
}

void LampPgo::RemoveLastLoopClosure() {
  KimeraRPGO::EdgePtr removed_edge = pgo_solver_->removeLastLoopClosure();
  if (removed_edge != NULL) {
    // Extract the optimized values
    values_ = pgo_solver_->calculateEstimate();
    nfg_ = pgo_solver_->getFactorsUnsafe();

    ROS_INFO_STREAM("Removed last loop closure between "
                    << gtsam::DefaultKeyFormatter(removed_edge->from_key)
                    << " and "
                    << gtsam::DefaultKeyFormatter(removed_edge->to_key));

    // publish posegraph
    PublishValues();
  } else {
    ROS_WARN("No more loop closure to remove");
  }
}

void LampPgo::RemoveLCByIdCallback(const std_msgs::String::ConstPtr& msg) {
  if (msg->data.length() < 2) {
    return;  // needs two prefixes (two chars)
  }
  RemoveLastLoopClosure(msg->data[0], msg->data[1]);
  return;
}

void LampPgo::RemoveLCCallback(const std_msgs::Bool::ConstPtr& msg) {
  RemoveLastLoopClosure();
  return;
}

void LampPgo::ResetCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    // Re-initialize solver
    pgo_solver_.reset(new KimeraRPGO::RobustSolver(rpgo_params_));
    values_ = Values();
    nfg_ = NonlinearFactorGraph();
    nfg_all_ = NonlinearFactorGraph();
  }
}

void LampPgo::InputCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  // Callback for the input posegraph
  NonlinearFactorGraph all_factors, new_factors;
  Values all_values, new_values;

  ROS_DEBUG_STREAM("PGO received graph of size " << graph_msg->nodes.size());

  // Convert to gtsam type
  lamp_utils::PoseGraphMsgToGtsam(graph_msg, &all_factors, &all_values);

  // Track node IDs
  for (auto n : graph_msg->nodes) {
    if (!key_to_id_map_.count(n.key)) {
      key_to_id_map_[n.key] = n.ID;
    }
  }

  // Track edge types
  for (auto e : graph_msg->edges) {
    edge_to_type_[std::make_pair(e.key_to, e.key_from)] = e.type;
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

  gtsam::Values temp_values = values_;
  temp_values.insert(new_values);

  // Extract the new factors
  for (size_t i = 0; i < all_factors.size(); i++) {
    bool factor_exists = false;
    for (size_t j = 0; j < nfg_all_.size(); j++) {
      if (nfg_all_[j]->keys() == all_factors[i]->keys()) {
        factor_exists = true;
        break;
      }
    }
    if (!factor_exists) {
      // this factor does not exist before
      bool loop_closure =
          (lamp_utils::IsRobotPrefix(gtsam::Symbol(all_factors[i]->back()).chr()) &&
           lamp_utils::IsRobotPrefix(gtsam::Symbol(all_factors[i]->front()).chr()) &&
           all_factors[i]->back() != all_factors[i]->front() + 1);
      if (!loop_closure) {
        new_factors.add(all_factors[i]);
      } else {
        if (all_factors[i]->error(temp_values) < max_lc_error_)
          new_factors.add(all_factors[i]);
        else {
          ROS_WARN("Loop closure discarded because of large error. ");
        }
      }
    }
  }

  ROS_DEBUG_STREAM("PGO adding new values " << new_values.size());
  for (auto k : new_values) {
    ROS_DEBUG_STREAM("\t" << gtsam::DefaultKeyFormatter(k.key));
  }
  ROS_DEBUG_STREAM("PGO adding new factors " << new_factors.size());

  // new_factors.print("new factors");

  ROS_DEBUG_STREAM("FACTORS BEFORE");

  // Run the optimizer
  pgo_solver_->update(new_factors, new_values);
  // Track all the added factors (including rejected ones)
  nfg_all_.add(new_factors);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  ROS_DEBUG_STREAM("FACTORS AFTER");
  std::vector<double> bad_errors;
  for (auto f : nfg_) {
    // f->printKeys();
    double error = f->error(values_);
    ROS_DEBUG_STREAM("Error: " << error);
    if (error > 10.0){
      bad_errors.push_back(error);
    }
  }

  ROS_DEBUG_STREAM("PGO stored values of size " << values_.size());
  ROS_DEBUG_STREAM("PGO stored nfg of size " << nfg_.size());

  // publish posegraph
  PublishValues();

  if (!bad_errors.empty()) {
    ROS_WARN_STREAM("After optimization, "
                    << bad_errors.size()
                    << " factors have high error. Likely GNC outliers.");
  }
}

// TODO - check that this is ok including just the positions in the message
void LampPgo::PublishValues() const {
  pose_graph_msgs::PoseGraph pose_graph_msg;
  int iter_debug = 0;
  // Then store the values as nodes
  gtsam::KeyVector key_list = values_.keys();
  // Extract the marginal/covariances of the optimized values

  // gtsam::Marginals marginal(nfg_, values_);
  // marginal.print();
  // marginal.bayesTree_.print("Bayes Tree: ");


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



    // ROS_WARN_STREAM("Debug get covariance");

    pose_graph_msg.nodes.push_back(node);
  }
  try {
    gtsam::Marginals marginal(nfg_, values_);
    for (size_t k = 0 ; k < key_list.size(); ++k) {
      auto key = key_list[k];
      auto node = pose_graph_msg.nodes[k];
      // covariance
      try {
        auto cov_matrix = marginal.marginalCovariance(gtsam::Symbol(key));
        int iter = 0;
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            node.covariance[iter] = cov_matrix(i, j);
            iter++;
          }
        }
      }
      catch (std::exception& e) {
        ROS_WARN_STREAM("Key is not found in the clique"
                        << gtsam::DefaultKeyFormatter(key));
      }
    }
  } catch (gtsam::IndeterminantLinearSystemException e) {
    ROS_ERROR_STREAM("LampPgo System is indeterminant, not computing covariance");
      boost::array<float, 36> default_covariance;
      default_covariance.assign(1e-4);
      for (size_t k = 0 ; k < key_list.size(); ++k) {
          auto node = pose_graph_msg.nodes[k];
          node.covariance = default_covariance;
        }
  }
  for (const auto& factor : nfg_) {
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor)) {
      pose_graph_msgs::PoseGraphEdge edge;
      edge.key_from = factor->front();
      edge.key_to = factor->back();
      lamp_utils::UpdateCovariance(
          edge,
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
              factor)
              ->noiseModel());

      // TODO this makes the assumption that any two nodes has at most one edge
      // which may not be true in the case of e.g. multiple loop closure
      // modalities
      auto it1 = edge_to_type_.find(std::make_pair(edge.key_to, edge.key_from));
      auto it2 = edge_to_type_.find(std::make_pair(edge.key_from, edge.key_to));
      if (it1 != edge_to_type_.end()) {
        edge.type = it1->second;
      } else if (it2 != edge_to_type_.end()) {
        edge.type = it2->second;
      } else {
        ROS_ERROR_STREAM("Couldn't find edge type for edge from: "
                         << edge.key_from << ", to: " << edge.key_to);
        edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
      }
      pose_graph_msg.edges.push_back(edge);
    }
  }

  ROS_DEBUG_STREAM("PGO publishing graph with " << pose_graph_msg.nodes.size()
                                                << " values");
  optimized_pub_.publish(pose_graph_msg);
}

void LampPgo::IgnoreRobotLoopClosures(const std_msgs::String::ConstPtr& msg) {
  // First convert string "huskyn" to char prefix
  char prefix = lamp_utils::GetRobotPrefix(msg->data);

  pgo_solver_->ignorePrefix(prefix);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Double check that it is actually ignored
  std::vector<char> ignored_prefixes = pgo_solver_->getIgnoredPrefixes();
  if (std::find(ignored_prefixes.begin(), ignored_prefixes.end(), prefix) ==
      ignored_prefixes.end()) {
    ROS_ERROR_STREAM("Failed to ignore loop closures involving prefix "
                     << prefix);
    return;
  }
  ROS_INFO_STREAM("Ignoring all loop closures involving prefix " << prefix);
  // Add to ignored list
  if (std::find(ignored_list_.begin(), ignored_list_.end(), msg->data) ==
      ignored_list_.end()) {
    ignored_list_.push_back(msg->data);
  }

  // publish posegraph
  PublishValues();
  // publish ignored list
  PublishIgnoredList();
}

void LampPgo::ReviveRobotLoopClosures(const std_msgs::String::ConstPtr& msg) {
  // First convert string "huskyn" to char prefix
  char prefix = lamp_utils::GetRobotPrefix(msg->data);

  pgo_solver_->revivePrefix(prefix);

  // Extract the optimized values
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Double check that it is actually revived
  std::vector<char> ignored_prefixes = pgo_solver_->getIgnoredPrefixes();
  if (std::find(ignored_prefixes.begin(), ignored_prefixes.end(), prefix) !=
      ignored_prefixes.end()) {
    ROS_ERROR_STREAM("Failed to revive loop closures involving prefix "
                     << prefix);
    return;
  }
  ROS_INFO_STREAM("Reviving all loop closures involving prefix " << prefix);
  // Remove from ignored list
  if (std::find(ignored_list_.begin(), ignored_list_.end(), msg->data) !=
      ignored_list_.end()) {
    ignored_list_.erase(
        std::remove(ignored_list_.begin(), ignored_list_.end(), msg->data),
        ignored_list_.end());
  }

  // publish posegraph
  PublishValues();
  // publish ignored list
  PublishIgnoredList();
}

void LampPgo::PublishIgnoredList() const {
  std::string list_str = "";
  for (size_t i = 0; i < ignored_list_.size(); i++) {
    list_str = list_str + ignored_list_[i] + ", ";
  }

  std_msgs::String msg;
  msg.data = list_str;

  ignored_list_pub_.publish(msg);
  return;
}
