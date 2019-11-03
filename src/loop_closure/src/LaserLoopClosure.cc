/*
LaserLoopClosure.cc
Author: Yun Chang
Lidar pointcloud based loop closure
*/
#include "loop_closure/LaserLoopClosure.h"

#include <boost/range/as_array.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

LaserLoopClosure::LaserLoopClosure(const ros::NodeHandle& n)
  : LoopClosure(n), last_closure_key_(0) {}

LaserLoopClosure::~LaserLoopClosure() {}

bool LaserLoopClosure::Initialize(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());

  // If laser loop closures are off, exit without setting up publishers or subscribers
  if (!pu::Get(param_ns_ + "/b_find_laser_loop_closures", b_check_for_loop_closures_))
    return false;
  if (!b_check_for_loop_closures_)
    return true;

  // Subscribers
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans", 10, &LaserLoopClosure::KeyedScanCallback, this);

  // Publishers
  loop_closure_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "laser_loop_closures", 10, false);

  // Parameters
  double distance_to_skip_recent_poses;
  // Load loop closing parameters.
  if (!pu::Get(param_ns_ + "/translation_threshold_nodes", translation_threshold_nodes_))
    return false;
  if (!pu::Get(param_ns_ + "/proximity_threshold", proximity_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/max_tolerable_fitness", max_tolerable_fitness_))
    return false;
  if (!pu::Get(param_ns_ + "/distance_to_skip_recent_poses", distance_to_skip_recent_poses))
    return false;
  if (!pu::Get(param_ns_ + "/distance_before_reclosing", distance_before_reclosing_))
    return false;

  // Load ICP parameters (from point_cloud localization)
  if (!pu::Get(param_ns_ + "/icp_lc/tf_epsilon", icp_tf_epsilon_)) return false;
  if (!pu::Get(param_ns_ + "/icp_lc/corr_dist", icp_corr_dist_)) return false;
  if (!pu::Get(param_ns_ + "/icp_lc/iterations", icp_iterations_)) return false;

  // Hard coded covariances
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;

  int icp_init_method;
  if (!pu::Get(param_ns_ + "/icp_initialization_method", icp_init_method))
    return false;
  icp_init_method_ = IcpInitMethod(icp_init_method);

  skip_recent_poses_ =
      (int)(distance_to_skip_recent_poses / translation_threshold_nodes_);

  // Initialize point cloud filter
  if (!filter_.Initialize(n)) {
    ROS_ERROR("LaserLoopClosure: Failed to initialize point cloud filter.");
    return false;
  }

  return true;
}

bool LaserLoopClosure::FindLoopClosures(
    gtsam::Key new_key,
    std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {
  // Look for loop closures for the latest received key
  // Don't check for loop closures against poses that are missing scans.
  if (!keyed_scans_.count(new_key)) {
    ROS_WARN_STREAM("Key " << gtsam::DefaultKeyFormatter(new_key)
                           << " does not have a scan");
    return false;
  }

  // If a loop has already been closed recently, don't try to close a new one.
  if (std::llabs(new_key - last_closure_key_) * translation_threshold_nodes_ <
      distance_before_reclosing_)
    return false;

  // Get pose and scan for the provided key.
  const gtsam::Pose3 pose1 = keyed_poses_.at(new_key);
  const PointCloud::ConstPtr scan1 = keyed_scans_.at(new_key);

  bool closed_loop = false;

  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    const gtsam::Symbol other_key = it->first;

    // Don't self-check.
    if (other_key == new_key)
      continue;

    // Don't compare against poses that were recently collected.
    if (std::llabs(new_key - other_key) < skip_recent_poses_)
      continue;

    // Get pose for the other key.
    const gtsam::Pose3 pose2 = keyed_poses_.at(other_key);
    const gtsam::Pose3 difference = pose1.between(pose2);
    
    if (difference.translation().norm() < proximity_threshold_) {
      const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];

      gu::Transform3 delta;  // (Using BetweenFactor)
      gtsam::Matrix66 covariance = Eigen::MatrixXd::Zero(6,6);

      double fitness_score; // retrieve ICP fitness score if matched
      ROS_INFO_STREAM("Performing alignment between "
                      << gtsam::DefaultKeyFormatter(new_key) << " and "
                      << gtsam::DefaultKeyFormatter(other_key));
      if (PerformAlignment(
              scan1, scan2, pose1, pose2, &delta, &covariance, fitness_score)) {
        ROS_INFO_STREAM("Closed loop between "
                        << gtsam::DefaultKeyFormatter(new_key) << " and "
                        << gtsam::DefaultKeyFormatter(other_key));

        closed_loop = true;
        pose_graph_msgs::PoseGraphEdge edge = CreateLoopClosureEdge(new_key, other_key, delta, covariance);

        loop_closure_edges->push_back(edge);
      }
    }
  } 

  return closed_loop;
}

pose_graph_msgs::PoseGraphEdge LaserLoopClosure::CreateLoopClosureEdge(
        gtsam::Symbol key1, 
        gtsam::Symbol key2,
        geometry_utils::Transform3& delta, 
        gtsam::Matrix66& covariance) {
  last_closure_key_ = key1;
  // Send an message notifying any subscribers that we found a loop
  // closure and having the keys of the loop edge.
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key1;
  edge.key_to = key2;
  edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
  edge.pose = gr::ToRosPose(delta);
  // convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      edge.covariance[6 * i + j] = covariance(i, j);
    }
  }
  // push back to vector
  return edge;
}

bool LaserLoopClosure::PerformAlignment(const PointCloud::ConstPtr& scan1,
                                        const PointCloud::ConstPtr& scan2,
                                        const gtsam::Pose3& pose1,
                                        const gtsam::Pose3& pose2,
                                        gu::Transform3* delta,
                                        gtsam::Matrix66* covariance,
                                        double& fitness_score) {
  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("PerformAlignment: Output pointers are null.");
    return false;
  }

  if (scan1 == NULL || scan2 == NULL) {
    ROS_ERROR("PerformAlignment: Null point clouds.");
    return false;
  }
  // Set up ICP.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // setVerbosityLevel(pcl::console::L_DEBUG);
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);

  // Filter the two scans. They are stored in the pose graph as dense scans for
  // visualization. Filter the first scan only when it is not filtered already.
  // Can be extended to the other scan if all the key scan pairs store the
  // filtered results.
  PointCloud::Ptr scan1_filtered(new PointCloud);
  filter_.Filter(scan1, scan1_filtered);

  PointCloud::Ptr scan2_filtered(new PointCloud);
  filter_.Filter(scan2, scan2_filtered);

  icp.setInputSource(scan1_filtered);

  icp.setInputTarget(scan2_filtered);

  ///// ICP initialization scheme
  // Default is to initialize by identity. Other options include
  // initializing with odom measurement
  // or initialize with 0 translation byt rotation from odom
  Eigen::Matrix4f initial_guess;

  switch (icp_init_method_) {
  case IcpInitMethod::IDENTITY: // initialize with idientity
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  } break;
  case IcpInitMethod::ODOMETRY: // initialize with odometry
  {
    gtsam::Pose3 pose_21 = pose2.between(pose1);
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
    initial_guess.block(0, 3, 3, 1) =
        pose_21.translation().vector().cast<float>();
  } break;
  case IcpInitMethod::ODOM_ROTATION: // initialize with zero translation but
                                     // rot from odom
  {
    gtsam::Pose3 pose_21 = pose2.between(pose1);
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
  } break;
  default: // identity as default (default in ICP anyways)
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  }
  }

  // Perform ICP.
  PointCloud unused_result;
  icp.align(unused_result, initial_guess);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  // gu::Transform3 delta_icp;
  delta->translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  delta->rotation = gu::Rot3(T(0, 0),
                             T(0, 1),
                             T(0, 2),
                             T(1, 0),
                             T(1, 1),
                             T(1, 2),
                             T(2, 0),
                             T(2, 1),
                             T(2, 2));

  // Is the transform good?
  if (!icp.hasConverged()) {
    std::cout << "No converged, score is: " << icp.getFitnessScore()
              << std::endl;
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) {
    std::cout << "Converged, score is: " << icp.getFitnessScore() << std::endl;
    return false;
  }

  fitness_score = icp.getFitnessScore();

  // Find transform from pose2 to pose1 from output of ICP.
  *delta = gu::PoseInverse(*delta); // NOTE: gtsam need 2_Transform_1 while
                                    // ICP output 1_Transform_2

  // TODO: Use real ICP covariance.
  for (int i = 0; i < 3; ++i)
    (*covariance)(i, i) = laser_lc_rot_sigma_ * laser_lc_rot_sigma_;
  for (int i = 3; i < 6; ++i)
    (*covariance)(i, i) = laser_lc_trans_sigma_ * laser_lc_trans_sigma_;

  return true;
}

void LaserLoopClosure::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_ERROR_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already has a scan");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}
