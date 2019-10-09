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
    b_use_fixed_covariances_(false) {
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

// Create Publishers
bool LampBase::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
}

// bool LampBase::RegisterCallbacks(const ros::NodeHandle& n){
//   ros::NodeHandle nl(n);

//   return false;
// }

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
  merger_.OnFastGraphMsg(ConvertPoseGraphToMsg());

  gtsam::Values new_values; 
  gtsam::Symbol key;

  // Update the internal LAMP graph using the one stored by the merger
  pose_graph_msgs::PoseGraphConstPtr fused_graph(new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));

  // update the LAMP internal values_ and factors
  utils::PoseGraphMsgToGtsam(fused_graph, &nfg_, &values_);

  // TODO-maybe - make the copy more efficient
}

//------------------------------------------------------------------------------------------
// Conversion and publish pose graph functions
//------------------------------------------------------------------------------------------

// Convert the internally stored pose_graph to a pose graph message
pose_graph_msgs::PoseGraphConstPtr LampBase::ConvertPoseGraphToMsg(){
  // Uses internally stored info.
  // Returns a pointer to handle passing more efficiently to the merger

  // Create the Pose Graph Message
  pose_graph_msgs::PoseGraph g;
  g.header.frame_id = fixed_frame_id_;
  g.header.stamp = keyed_stamps_[key_ - 1]; // Get timestamp from latest keyed pose

  // Flag on whether it is incremental or not
  // TODO make incremental Pose Graph publishing
  g.incremental = false;

  // Get Values
  ConvertValuesToNodeMsgs(g.nodes);

  // Add the factors  // TODO - check integration of this tracking with all
  // handlers
  g.edges = edges_info_;
  g.priors = priors_info_;

  pose_graph_msgs::PoseGraphConstPtr g_ptr(new pose_graph_msgs::PoseGraph(g));

  return g_ptr;

}


// Function returns a vector of node messages from an input values
bool LampBase::ConvertValuesToNodeMsgs(std::vector<pose_graph_msgs::PoseGraphNode>& nodes){
  // Converts the internal values 

  for (const auto& keyed_pose : values_) {

    gu::Transform3 t = utils::ToGu(values_.at<gtsam::Pose3>(keyed_pose.key));

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
  // TODO - incremental publishing instead of full graph

  // Convert master pose-graph to messages
  pose_graph_msgs::PoseGraphConstPtr g = ConvertPoseGraphToMsg();

  // Publish 
  pose_graph_pub_.publish(*g);

  return true;
}

bool LampBase::PublishPoseGraphForOptimizer() {
  
  // TODO - incremental publishing instead of full graph

  // Convert master pose-graph to messages
  pose_graph_msgs::PoseGraphConstPtr g = ConvertPoseGraphToMsg();

  // Publish 
  pose_graph_to_optimize_pub_.publish(*g);

  return true;
}

//------------------------------------------------------------------------------------------
// Tracking
//------------------------------------------------------------------------------------------

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

  // TODO - add covariance 

  edges_info_.push_back(edge);
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
}

// Placeholder function to used fixed covariances while proper covariances are
// being developed
gtsam::SharedNoiseModel LampBase::SetFixedNoiseModels(std::string type) {
  gtsam::Vector6 zero;
  zero << 0, 0, 0, 0, 0, 0;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(zero);

  // Switch based on type
  if (type == "odom") {
    gtsam::Vector6 noise_vec;
    noise_vec.head<3>().setConstant(attitude_sigma_);
    noise_vec.tail<3>().setConstant(position_sigma_);
    noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
  } else if (type == "laser_loop_closure") {
    gtsam::Vector6 noise_vec;
    noise_vec.head<3>().setConstant(laser_lc_rot_sigma_);
    noise_vec.tail<3>().setConstant(laser_lc_trans_sigma_);
    noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
  } else if (type == "manual_loop_closure") {
    gtsam::Vector6 noise_vec;
    noise_vec.head<3>().setConstant(manual_lc_rot_precision_);
    noise_vec.tail<3>().setConstant(manual_lc_trans_precision_);
    noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vec);
  } else if (type == "artifact") {
    gtsam::Vector6 precisions;
    precisions.head<3>().setConstant(artifact_rot_precision_);
    precisions.tail<3>().setConstant(artifact_trans_precision_);
    noise = gtsam::noiseModel::Diagonal::Precisions(precisions);

  } else if (type == "april") {
    gtsam::Vector6 precisions;
    precisions.head<3>().setConstant(fiducial_rot_precision_);
    precisions.tail<3>().setConstant(fiducial_trans_precision_);
    noise = gtsam::noiseModel::Diagonal::Precisions(precisions);
  } else if (type == "total_station") {
  } else {
    ROS_ERROR("Incorrect input into SetFixedNoiseModels - invalid type");
  }
  // TODO - implement others

  return noise;
}
