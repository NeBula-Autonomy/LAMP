/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampBase.h>

// #include <math.h>
// #include <ctime>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

using gtsam::BetweenFactor;
using gtsam::RangeFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;

// Constructor
LampBase::LampBase()
  : prefix_(""),
    update_rate_(10),
    time_threshold_(0.001),
    b_use_fixed_covariances_(false),
    initial_key_(0) {
  // any other things on construction
    }

// Destructor
LampBase::~LampBase() {}

// Initialization
bool LampBase::Initialize(const ros::NodeHandle& n) {

  LoadParameters(n);
  CreatePublishers(n);
  // InitializeHandlers(n);

}

// Load Parameters
bool LampBase::LoadParameters(const ros::NodeHandle& n) {

}

bool LampBase::SetFactorPrecisions() {
  if (!pu::Get("attitude_sigma", attitude_sigma_))
    return false;
  if (!pu::Get("position_sigma", position_sigma_))
    return false;
  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_))
    return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_))
    return false;
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;
  if (!pu::Get("artifact_rot_precision", artifact_rot_precision_))
    return false;
  if (!pu::Get("artifact_trans_precision", artifact_trans_precision_))
    return false;
  if (!pu::Get("point_estimate_precision", point_estimate_precision_))
    return false;

  if (!pu::Get("fiducial_trans_precision", fiducial_trans_precision_))
    return false;
  if (!pu::Get("fiducial_rot_precision", fiducial_rot_precision_))
    return false;

  // Set as noise models
  gtsam::Vector6 sigmas;
  sigmas.head<3>().setConstant(attitude_sigma_);
  sigmas.tail<3>().setConstant(position_sigma_);
  odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Set as noise models
  sigmas.head<3>().setConstant(laser_lc_rot_sigma_);
  sigmas.tail<3>().setConstant(laser_lc_trans_sigma_);
  laser_lc_noise_ = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  // Artifact
  gtsam::Vector6 precisions;
  sigmas.head<3>().setConstant(artifact_rot_precision_);
  sigmas.tail<3>().setConstant(artifact_trans_precision_);
  artifact_noise_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  return true;
}

// Create Publishers
bool LampBase::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  pose_graph_incremental_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental", 10, false);
}

bool LampBase::InitializeHandlers(const ros::NodeHandle& n){
  return false;
}

bool LampBase::CheckHandlers(){
  return false;
}

gtsam::Symbol LampBase::GetClosestKeyAtTime(const ros::Time& stamp) const {

  // If there are no keys, throw an error
  if (stamp_to_odom_key_.size() == 0){
    ROS_ERROR("Cannot get closest key - no keys are stored");
    return gtsam::Symbol();
  }

  // Iterators pointing immediately before and after the target time
  auto iterAfter = stamp_to_odom_key_.lower_bound(stamp.toSec());
  auto iterBefore = std::prev(iterAfter);
  double t1 = iterBefore->first; 
  double t2 = iterAfter->first;

  // If time is before the start or after the end, return first/last key
  if (iterAfter == stamp_to_odom_key_.begin()) {
    ROS_WARN("Only one stored key");
    return iterAfter->second;
  }
  else if (iterBefore == std::prev(stamp_to_odom_key_.end())) {
    ROS_WARN("Only one stored key");
    return iterBefore->second;
  }

  // Otherwise return the closer key
  else if (stamp.toSec() - t1 < t2 - stamp.toSec()) {
    return iterBefore->second;
  }
  else {
    return iterAfter->second;
  }
}

gtsam::Symbol LampBase::GetKeyAtTime(const ros::Time& stamp) const {

  if (!stamp_to_odom_key_.count(stamp.toSec())) {
    ROS_ERROR("No key exists at given time");
    return gtsam::Symbol();
  }

  return stamp_to_odom_key_.at(stamp.toSec());
}

bool LampBase::IsTimeWithinThreshold(double time, const ros::Time& target) const {
  return abs(time - target.toSec()) <= time_threshold_;
}

void LampBase::OptimizerUpdateCallback(const pose_graph_msgs::PoseGraphConstPtr &msg) {

  ROS_INFO_STREAM("Received new pose graph from optimizer - merging now --------------------------------------------------");
  
  // Process the slow graph update
  merger_.OnSlowGraphMsg(msg);

  // Give merger the current graph (will likely have more nodes that the
  // optimized)
  merger_.OnFastGraphMsg(
      ConvertPoseGraphToMsg(values_, edges_info_, priors_info_));

  gtsam::Values new_values; 
  gtsam::Symbol key;

  // Update the internal LAMP graph using the one stored by the merger
  pose_graph_msgs::PoseGraphConstPtr fused_graph(new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));

  // update the LAMP internal values_ and factors
  utils::PoseGraphMsgToGtsam(fused_graph, &nfg_, &values_);

  // TODO-maybe - make the copy more efficient
}

// Callback from a laser loop closure message
void LampBase::LaserLoopClosureCallback(
    const pose_graph_msgs::PoseGraphConstPtr msg) {
  ROS_INFO_STREAM("Received laser loop closure message "
                  "--------------------------------------------------");

  // Do things particular to loop closures from the laser

  if (b_use_fixed_covariances_) {
    // Change the covariances in the message first
    pose_graph_msgs::PoseGraph graph_msg = *msg;

    ChangeCovarianceInMessage(graph_msg, laser_lc_noise_);

    pose_graph_msgs::PoseGraphConstPtr msg_ptr(
        new pose_graph_msgs::PoseGraph(graph_msg));

    AddLoopClosureToGraph(msg_ptr);
  } else {
    // Add to the graph
    AddLoopClosureToGraph(msg);
  }
}

// Generic addition of loop closure information to the graph
void LampBase::AddLoopClosureToGraph(
    const pose_graph_msgs::PoseGraphConstPtr msg) {
  // Add to nfg
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values blank_values;
  utils::PoseGraphMsgToGtsam(msg, &new_factors, &blank_values);
  nfg_.add(new_factors);

  // Get info for publishing later - in edges_info
  gtsam::Pose3 transform;
  gtsam::SharedNoiseModel covariance;

  // Add the factors to the pose_graph - loop through in case of multiple loop
  // closures
  for (const pose_graph_msgs::PoseGraphEdge& edge : msg->edges) {
    // Transform to gtsam format
    transform = utils::EdgeMessageToPose(edge);
    covariance = utils::EdgeMessageToCovariance(edge);

    // Add to tracked edges
    TrackEdges(edge.key_from, edge.key_to, edge.type, transform, covariance);
  }

  // Set flag to optimize
  b_run_optimization_ = true;
}

pose_graph_msgs::PoseGraph
LampBase::ChangeCovarianceInMessage(pose_graph_msgs::PoseGraph msg,
                                    gtsam::SharedNoiseModel noise) {
  // Loop through each edge
  for (pose_graph_msgs::PoseGraphEdge& edge : msg.edges) {
    // Replace covariance with the noise model
    utils::UpdateEdgeCovariance(edge, noise);
  }

  return msg;
}

//------------------------------------------------------------------------------------------
// Conversion and publish pose graph functions
//------------------------------------------------------------------------------------------

// Convert the internally stored pose_graph to a pose graph message
// Can be used for incremental or full graph
pose_graph_msgs::PoseGraphConstPtr LampBase::ConvertPoseGraphToMsg(
    gtsam::Values values, EdgeMessages edges_info, PriorMessages priors_info) {
  // Uses internally stored info.
  // Returns a pointer to handle passing more efficiently to the merger

  // Create the Pose Graph Message
  pose_graph_msgs::PoseGraph g;
  g.header.frame_id = fixed_frame_id_;
  g.header.stamp = keyed_stamps_[key_ - 1]; // Get timestamp from latest keyed pose

  // Get Values
  ConvertValuesToNodeMsgs(values, g.nodes);

  // Add the factors  // TODO - check integration of this tracking with all
  // handlers
  g.edges = edges_info;
  g.priors = priors_info;

  pose_graph_msgs::PoseGraphConstPtr g_ptr(new pose_graph_msgs::PoseGraph(g));

  return g_ptr;

}


// Function returns a vector of node messages from an input values
bool LampBase::ConvertValuesToNodeMsgs(
    gtsam::Values values, std::vector<pose_graph_msgs::PoseGraphNode>& nodes) {
  // Converts the internal values

  for (const auto& keyed_pose : values) {
    gu::Transform3 t = utils::ToGu(values.at<gtsam::Pose3>(keyed_pose.key));

    gtsam::Symbol sym_key = gtsam::Symbol(keyed_pose.key);

    // Populate the message with the pose's data.
    pose_graph_msgs::PoseGraphNode node;
    node.key = keyed_pose.key;
    node.header.frame_id = fixed_frame_id_;
    node.pose = gr::ToRosPose(t);

    // Get timestamp
    // Note keyed_stamps are for all nodes TODO - check this is followed
    node.header.stamp = keyed_stamps_[keyed_pose.key];

    // Get the IDs
    if (keyed_scans_.count(keyed_pose.key)) {
      // Key frame, note in the ID
      node.ID = "key_frame";
    }else if (sym_key.chr() == prefix_[0]){
      // Odom or key frame
      node.ID = "odom";
    }else if (sym_key.chr() == 'u'){
      // UWB
      // node.ID = uwd_handler_.GetUWBID(keyed_pose.key); // TODO
      node.ID = "UWB"; // TEMPORARY
    } else {
      // Artifact
      // node.ID = artifact_handler_.GetArtifactID(keyed_pose.key);// TODO
      node.ID = "Artifact"; // TEMPORARY
    }

    // TODO - fill covariance

    // Add to the vector of nodes
    nodes.push_back(node);
  }
  return true;
}


bool LampBase::PublishPoseGraph() {

  // Incremental publishing
  if (pose_graph_incremental_pub_.getNumSubscribers() > 0) {
    // Convert new parts of the pose-graph to messages
    pose_graph_msgs::PoseGraphConstPtr g_inc =
        ConvertPoseGraphToMsg(values_new_, edges_info_new_, priors_info_new_);
    // TODO - change interface to just take a flag? Then do the clear in there?
    // - no want to make sure it is published

    // Publish
    pose_graph_incremental_pub_.publish(*g_inc);

    // Reset new tracking
    values_new_.clear();
    edges_info_new_.clear();
    priors_info_new_.clear();
  }

  // Full pose graph publishing
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Convert master pose-graph to messages
    pose_graph_msgs::PoseGraphConstPtr g_full =
        ConvertPoseGraphToMsg(values_, edges_info_, priors_info_);

    // Publish
    pose_graph_pub_.publish(*g_full);
  }

  return true;
}

bool LampBase::PublishPoseGraphForOptimizer() {
  
  // TODO - incremental publishing instead of full graph

  // Convert master pose-graph to messages
  pose_graph_msgs::PoseGraphConstPtr g =
      ConvertPoseGraphToMsg(values_, edges_info_, priors_info_);

  // Publish 
  pose_graph_to_optimize_pub_.publish(*g);

  return true;
}

//------------------------------------------------------------------------------------------
// Tracking
//------------------------------------------------------------------------------------------

// Helper to add to new values tacking variable as well
void LampBase::AddNewValues(gtsam::Values new_values) {
  // Main values variable
  values_.insert(new_values);

  // New values tracking
  values_new_.insert(new_values);
}

void LampBase::TrackEdges(gtsam::Symbol key_from, 
                          gtsam::Symbol key_to, 
                          int type, 
                          gtsam::Pose3 pose, 
                          gtsam::SharedNoiseModel covariance){

  // Populate the message with the pose's data.
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key_from;
  edge.key_to = key_to;
  edge.type = type;
  // edge.header.frame_id = fixed_frame_id_;
  // edge.header.stamp = keyed_stamps_[key_to];
  edge.pose = gr::ToRosPose(utils::ToGu(pose));

  // Update the covariance
  utils::UpdateEdgeCovariance(edge, covariance);

  edges_info_.push_back(edge);
  edges_info_new_.push_back(edge);
}

void LampBase::TrackPriors(ros::Time stamp, gtsam::Symbol key, gtsam::Pose3 pose, gtsam::SharedNoiseModel covariance){

  // Populate the message with the pose's data.
  pose_graph_msgs::PoseGraphNode prior;
  prior.key = key;
  prior.header.frame_id = fixed_frame_id_;
  prior.header.stamp = keyed_stamps_[key];
  prior.pose = gr::ToRosPose(utils::ToGu(pose));

  // TODO - add covariance 

  priors_info_.push_back(prior);
  priors_info_new_.push_back(prior);
}

// Placeholder function to used fixed covariances while proper covariances are
// being developed
gtsam::SharedNoiseModel LampBase::SetFixedNoiseModels(std::string type) {
  gtsam::Vector6 zero;
  zero << 0, 0, 0, 0, 0, 0;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(zero);

  // Switch based on type
  if (type == "odom") {
    // gtsam::Vector6 noise_vec;
    // noise_vec.head<3>().setConstant(attitude_sigma_);
    // noise_vec.tail<3>().setConstant(position_sigma_);
    // noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
    noise = odom_noise_;
  } else if (type == "laser_loop_closure") {
    // gtsam::Vector6 noise_vec;
    // noise_vec.head<3>().setConstant(laser_lc_rot_sigma_);
    // noise_vec.tail<3>().setConstant(laser_lc_trans_sigma_);
    // noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
    noise = laser_lc_noise_;
  } else if (type == "manual_loop_closure") {
    gtsam::Vector6 noise_vec;
    noise_vec.head<3>().setConstant(manual_lc_rot_precision_);
    noise_vec.tail<3>().setConstant(manual_lc_trans_precision_);
    noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
  } else if (type == "artifact") {
    // gtsam::Vector6 precisions;
    // precisions.head<3>().setConstant(artifact_rot_precision_);
    // precisions.tail<3>().setConstant(artifact_trans_precision_);
    // noise = gtsam::noiseModel::Diagonal::Precisions(precisions);
    noise = artifact_noise_;
  } else if (type == "april") {
    gtsam::Vector6 precisions;
    precisions.head<3>().setConstant(fiducial_rot_precision_);
    precisions.tail<3>().setConstant(fiducial_trans_precision_);
    noise = gtsam::noiseModel::Diagonal::Precisions(precisions);
  } else if (type == "total_station") {
  } else {
    ROS_ERROR("Incorrect input into SetFixedNoiseModels - invalid type");
    throw std::invalid_argument("set fixed noise models");
  }
  // TODO - implement others

  return noise;
}
