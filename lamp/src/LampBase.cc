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
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::RangeFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;

// Constructor
LampBase::LampBase()
  : update_rate_(10),
    zero_noise_(0.0001),
    b_use_fixed_covariances_(false),
    b_repub_values_after_optimization_(false),
    b_received_optimizer_update_(false) {
  // any other things on construction

  // set up mapping function to get internal ID given gtsam::Symbol
  pose_graph_.symbol_id_map = boost::bind(&LampBase::MapSymbolToId, this, _1);
}

// Destructor
LampBase::~LampBase() {}

bool LampBase::SetFactorPrecisions() {
  if (!pu::Get("attitude_sigma", attitude_sigma_))
    return false;
  if (!pu::Get("position_sigma", position_sigma_))
    return false;
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
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

  // Set as noise models
  gtsam::Vector6 precisions;
  precisions.head<3>().setConstant(1e-12);
  precisions.tail<3>().setConstant(point_estimate_precision_);
  point_estimate_noise_ = gtsam::noiseModel::Diagonal::Precisions(precisions);

  return true;
}

// Create Publishers
bool LampBase::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, true);
  pose_graph_incremental_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental", 10, true);

  // Published keyed scans (for GT processing)
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, true);

  return true;
}

void LampBase::OptimizerUpdateCallback(
    const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_WARN_STREAM("Received new pose graph from optimizer - merging now "
                  "-----------------------------------------------------");
  b_received_optimizer_update_ = true;

  // ROS_INFO_STREAM("New pose graph nodes: ");
  // for (auto n : msg->nodes) {
  //   ROS_INFO_STREAM(gtsam::DefaultKeyFormatter(n.key)
  //                   << "(" << n.pose.position.x << ", " << n.pose.position.y
  //                   << ", " << n.pose.position.z << ")");
  // }

  // Merge the optimizer result into the internal pose graph
  // and also update loop closure edges to reflect inliers
  MergeOptimizedGraph(msg);

  // Publish the pose graph and update the map
  PublishPoseGraph(false);

  // Update the map (also publishes)
  ReGenerateMapPointCloud();
}

void LampBase::MergeOptimizedGraph(
    const pose_graph_msgs::PoseGraphConstPtr& msg) {
  // Process the slow graph update
  merger_.OnSlowGraphMsg(msg);

  // Give merger the current graph (will likely have more nodes that the
  // optimized)
  merger_.OnFastGraphMsg(pose_graph_.ToMsg());

  gtsam::Values new_values;
  gtsam::Symbol key;

  // Update the internal LAMP graph using the one stored by the merger
  pose_graph_msgs::PoseGraphConstPtr fused_graph(
      new pose_graph_msgs::PoseGraph(merger_.GetCurrentGraph()));

  // update the LAMP internal values_ and factors
  pose_graph_.UpdateFromMsg(fused_graph);

  // prune outliers given optimized graph
  pose_graph_.UpdateLoopClosures(msg);

  // ROS_DEBUG_STREAM("Pose graph after update: ");
  // for (auto n : pose_graph_.GetNodes()) {
  //   ROS_INFO_STREAM(gtsam::DefaultKeyFormatter(n.key)
  //                   << "(" << n.pose.position.x << ", " << n.pose.position.y
  //                   << ", " << n.pose.position.z << ")");
  // }

  if (b_repub_values_after_optimization_) {
    // ROS_INFO("Republishing all values on incremental pose graph");
    pose_graph_.AddAllValuesToNew();
  }
}

// Callback from a laser loop closure message
void LampBase::LaserLoopClosureCallback(
    const pose_graph_msgs::PoseGraphConstPtr msg) {
  ROS_DEBUG_STREAM("Received laser loop closure message "
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

  b_has_new_factor_ = true;
}

// Generic addition of loop closure information to the graph
void LampBase::AddLoopClosureToGraph(
    const pose_graph_msgs::PoseGraphConstPtr msg) {
  pose_graph_.UpdateFromMsg(msg);

  // Set flag to optimize
  b_run_optimization_ = true;
}

pose_graph_msgs::PoseGraph
LampBase::ChangeCovarianceInMessage(pose_graph_msgs::PoseGraph msg,
                                    gtsam::SharedNoiseModel noise) {
  // Loop through each edge
  for (pose_graph_msgs::PoseGraphEdge& edge : msg.edges) {
    // Replace covariance with the noise model
    lamp_utils::UpdateCovariance(edge, noise);
  }

  return msg;
}

//------------------------------------------------------------------------------------------
// Map generation functions
//------------------------------------------------------------------------------------------

bool LampBase::ReGenerateMapPointCloud() {
  // Reset the map
  mapper_->Reset();

  // Combine the keyed scans with the latest node values
  PointCloud::Ptr regenerated_map(new PointCloud);
  CombineKeyedScansWorld(regenerated_map.get());

  // Insert points into the map (publishes incremental point clouds)
  PointCloud::Ptr unused(new PointCloud);
  mapper_->InsertPoints(regenerated_map, unused.get());

  // Publish map
  mapper_->PublishMap();
  return true;
}

// For combining all the scans together
bool LampBase::CombineKeyedScansWorld(PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return false;
  }
  points->points.clear();

  // Iterate over poses in the graph, transforming their corresponding laser
  // scans into world frame and appending them to the output.
  for (const auto& keyed_pose : pose_graph_.GetValues()) {
    const gtsam::Symbol key = keyed_pose.key;

    PointCloud::Ptr scan_world(new PointCloud);

    // Transform the body-frame scan into world frame.
    GetTransformedPointCloudWorld(key, scan_world.get());

    // Append the world-frame point cloud to the output.
    *points += *scan_world;
  }
  ROS_DEBUG_STREAM("Points size is: " << points->points.size()
                                      << ", in CombineKeyedScansWorld");
  return true;
}

// Transform the point cloud to world frame
bool LampBase::GetTransformedPointCloudWorld(const gtsam::Symbol key,
                                             PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return false;
  }
  points->points.clear();

  // No key associated with the scan
  if (!pose_graph_.HasScan(key)) {
    ROS_WARN("Could not find scan associated with key in "
             "GetTransformedPointCloudWorld");
    return false;
  }

  // Check that the key exists
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %s does not exist in values in GetTransformedPointCloudWorld",
             gtsam::DefaultKeyFormatter(key).c_str());
    return false;
  }

  const gu::Transform3 pose = lamp_utils::ToGu(pose_graph_.GetPose(key));
  Eigen::Matrix4d b2w;
  b2w.setZero();
  b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
  b2w.block(0, 3, 3, 1) = pose.translation.Eigen();
  b2w(3, 3) = 1;

  Eigen::Quaterniond quat(pose.rotation.Eigen());
  quat.normalize();
  b2w.block(0, 0, 3, 3) = quat.matrix();

  // ROS_INFO_STREAM("TRANSFORMATION MATRIX (rotation det: " <<
  // pose.rotation.Eigen().determinant() << ")"); Eigen::IOFormat CleanFmt(4, 0,
  // ", ", "\n", "[", "]"); ROS_INFO_STREAM("\n" << b2w.format(CleanFmt));

  // Transform the body-frame scan into world frame.
  pcl::transformPointCloud(*pose_graph_.keyed_scans[key], *points, b2w);

  // ROS_INFO_STREAM("Points size is: " << points->points.size()
  //                                    << ", in
  //                                    GetTransformedPointCloudWorld");
  return true;
}

// For adding one scan to the map
bool LampBase::AddTransformedPointCloudToMap(const gtsam::Symbol key) {
  PointCloud::Ptr points(new PointCloud);

  GetTransformedPointCloudWorld(key, points.get());

  ROS_DEBUG_STREAM("Points size is: " << points->points.size()
                                     << ", in AddTransformedPointCloudToMap");

  // Add to the map
  PointCloud::Ptr unused(new PointCloud);
  mapper_->InsertPoints(points, unused.get());

  return true;
}

//------------------------------------------------------------------------------------------
// Conversion and publish pose graph functions
//------------------------------------------------------------------------------------------

bool LampBase::PublishPoseGraph(bool b_publish_incremental) {
  // Incremental publishing
  if (b_publish_incremental) {
    // Convert new parts of the pose-graph to messages
    pose_graph_msgs::PoseGraphConstPtr g_inc;

    if (b_have_received_first_pg_) {
      g_inc = pose_graph_.ToIncrementalMsg();
    } else {
      g_inc = pose_graph_.ToMsg();
    }
    // TODO - change interface to just take a flag? Then do the clear in there?
    // - no want to make sure it is published

    if (g_inc->nodes.size() > 0 || g_inc->edges.size() > 0) {
      ROS_DEBUG_STREAM("Publishing incremental graph with "
                       << g_inc->nodes.size() << " nodes and "
                       << g_inc->edges.size() << " edges");

      // Publish
      pose_graph_incremental_pub_.publish(*g_inc);

      // Reset new tracking
      pose_graph_.ClearIncrementalMessages();
    } else {
      ROS_DEBUG("No information for incremental publishing");
    }
  }

  // Full pose graph publishing
  // Convert master pose-graph to messages
  pose_graph_msgs::PoseGraphConstPtr g_full = pose_graph_.ToMsg();

  // Publish
  pose_graph_pub_.publish(*g_full);
  ROS_DEBUG_STREAM("Publishing full graph with "
                   << g_full->nodes.size() << " nodes and "
                   << g_full->edges.size() << " edges");

  return true;
}

bool LampBase::PublishPoseGraphForOptimizer() {
  // TODO incremental publishing instead of full graph?

  // Convert master pose-graph to messages
  pose_graph_msgs::PoseGraphConstPtr g = pose_graph_.ToMsg();

  // ROS_DEBUG_STREAM("Publishing pose graph for optimizer with "
  //                 << g->nodes.size() << " nodes and " << g->edges.size()
  //                 << " edges");
  for (auto v : g->nodes) {
    ROS_DEBUG_STREAM(
        "PublishedPGForOptimizer Key : " << gtsam::DefaultKeyFormatter(v.key));
  }

  // Publish
  pose_graph_to_optimize_pub_.publish(*g);

  return true;
}

// Placeholder function to used fixed covariances while proper covariances are
// being developed
gtsam::SharedNoiseModel LampBase::SetFixedNoiseModels(std::string type) {
  gtsam::Vector6 zero;
  zero << 0, 0, 0, 0, 0, 0;
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(zero);

  // Switch based on type
  if (type == "odom") {
    // gtsam::Vector6 sigmas;
    // sigmas.head<3>().setConstant(attitude_sigma_);
    // sigmas.tail<3>().setConstant(position_sigma_);
    // noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    noise = odom_noise_;
  } else if (type == "laser_loop_closure") {
    // gtsam::Vector6 sigmas;
    // sigmas.head<3>().setConstant(laser_lc_rot_sigma_);
    // sigmas.tail<3>().setConstant(laser_lc_trans_sigma_);
    // noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    noise = laser_lc_noise_;
  } else if (type == "total_station") {
  } else {
    ROS_ERROR("Incorrect input into SetFixedNoiseModels - invalid type");
    throw std::invalid_argument("set fixed noise models");
  }
  // TODO - implement others

  return noise;
}

std::string LampBase::MapSymbolToId(gtsam::Symbol key) const {
  if (pose_graph_.HasScan(key)) {
    // Key frame, note in the ID
    return "key_frame";
  }

  else if (lamp_utils::IsRobotPrefix(key.chr())) {
    // Odom or key frame
    return "odom_node";
  }

  else {
    ROS_ERROR("Unknown ID");
    return "";
  }
}

void LampBase::PublishAllKeyedScans() {
  if (pose_graph_.keyed_scans.size() == 0) {
    ROS_WARN("No keyed scans and you are trying to publish all keyed scans");
    return;
  }

  // ROS_INFO("Publishing All Keyed Scans");
  pose_graph_msgs::KeyedScan keyed_scan_msg;

  for (auto it = pose_graph_.keyed_scans.begin();
       it != pose_graph_.keyed_scans.end();
       ++it) {
    ROS_INFO_ONCE("Publishing Keyed Scans... WAIT UNTIL DONE");
    keyed_scan_msg.key = it->first;
    pcl::toROSMsg(*it->second, keyed_scan_msg.scan);
    keyed_scan_pub_.publish(keyed_scan_msg);

    ros::Duration(0.01).sleep();
  }
}
