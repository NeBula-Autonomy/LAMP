/*
LaserLoopClosure.cc
Author: Yun Chang
Lidar pointcloud based loop closure
*/
#include "loop_closure/LaserLoopClosure.h"

#include <boost/range/as_array.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
// #include <multithreaded_gicp/gicp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_representation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

LaserLoopClosure::LaserLoopClosure(const ros::NodeHandle& n)
  : LoopClosure(n) {}

LaserLoopClosure::~LaserLoopClosure() {
}

bool LaserLoopClosure::Initialize(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());

  // If laser loop closures are off, exit without setting up publishers or subscribers
  if (!pu::Get(param_ns_ + "/b_find_laser_loop_closures", b_check_for_loop_closures_))
    return false;

  // Subscribers
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans", 100000, &LaserLoopClosure::KeyedScanCallback, this);
  loop_closure_seed_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "seed_loop_closure", 100000, &LaserLoopClosure::SeedCallback, this);
  pc_gt_trigger_sub_ = nl.subscribe<std_msgs::String>(
      "trigger_pc_gt", 1, &LaserLoopClosure::TriggerGTCallback, this);

  // Publishers
  loop_closure_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "laser_loop_closures", 100000, false);
  gt_pub_ = nl.advertise<sensor_msgs::PointCloud2>(
      "ground_truth", 100000, false);
  current_scan_pub_ = nl.advertise<sensor_msgs::PointCloud2>(
      "current_scan", 100000, false);
  aligned_scan_pub_ = nl.advertise<sensor_msgs::PointCloud2>(
      "aligned_scan", 100000, false);
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
  if (!pu::Get(param_ns_ + "/max_rotation_deg", max_rotation_deg_))
    return false;
  max_rotation_rad_ = max_rotation_deg_ * M_PI/180;

  // Load ICP parameters (from point_cloud localization)
  if (!pu::Get(param_ns_ + "/icp_lc/tf_epsilon", icp_tf_epsilon_)) return false;
  if (!pu::Get(param_ns_ + "/icp_lc/corr_dist", icp_corr_dist_)) return false;
  if (!pu::Get(param_ns_ + "/icp_lc/iterations", icp_iterations_)) return false;
  if (!pu::Get(param_ns_ + "/icp_lc/threads", icp_threads_))
    return false;

  // Load SAC parameters
  if (!pu::Get(param_ns_ + "/sac_ia/iterations", sac_iterations_)) return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_prev_scans", sac_num_prev_scans_)) return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_next_scans", sac_num_next_scans_)) return false;
  if (!pu::Get(param_ns_ + "/sac_ia/normals_radius", sac_normals_radius_)) return false;
  if (!pu::Get(param_ns_ + "/sac_ia/features_radius", sac_features_radius_)) return false;
  if (!pu::Get(param_ns_ + "/sac_ia/fitness_score_threshold", sac_fitness_score_threshold_)) return false;

  // Load Harris parameters
  if (!pu::Get(param_ns_ + "/harris3D/harris_threshold_", harris_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_suppression", harris_suppression_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_radius", harris_radius_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_refine", harris_refine_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_response", harris_response_))
    return false;

  // Hard coded covariances
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;
  if (!pu::Get("gt_rot_sigma", gt_rot_sigma_))
    return false;
  if (!pu::Get("gt_trans_sigma", gt_trans_sigma_))
    return false;
  if (!pu::Get("gt_prior_covar", gt_prior_covar_))
    return false;

  int icp_init_method;
  if (!pu::Get(param_ns_ + "/icp_initialization_method", icp_init_method))
    return false;
  icp_init_method_ = IcpInitMethod(icp_init_method);

  skip_recent_poses_ =
      (int)(distance_to_skip_recent_poses / translation_threshold_nodes_);

  return true;
}

void LaserLoopClosure::AccumulateScans(
    gtsam::Key key,
    PointCloud::Ptr scan_out) {
  for (int i = 0; i < sac_num_prev_scans_; i++) {
    gtsam::Key prev_key = key - i - 1;
    // If scan doesn't exist, just skip it
    if (!keyed_poses_.count(prev_key) || !keyed_scans_.count(prev_key)) {
      continue;
    }
    const PointCloud::ConstPtr prev_scan = keyed_scans_[prev_key];
    
    // Transform and Accumulate
    const gtsam::Pose3 new_pose = keyed_poses_.at(key);
    const gtsam::Pose3 old_pose = keyed_poses_.at(prev_key);
    const gtsam::Pose3 tf = new_pose.between(old_pose);
    
    PointCloud::Ptr transformed(new PointCloud);
    pcl::transformPointCloud(*prev_scan, *transformed, tf.matrix());
    *scan_out += *transformed;
  }
  
  for (int i = 0; i < sac_num_next_scans_; i++) {
    gtsam::Key next_key = key + i + 1;
    // If scan doesn't exist, just skip it
    if (!keyed_poses_.count(next_key) || !keyed_scans_.count(next_key)) {
      continue;
    }
    const PointCloud::ConstPtr next_scan = keyed_scans_[next_key];
    
    // Transform and Accumulate
    const gtsam::Pose3 new_pose = keyed_poses_.at(key);
    const gtsam::Pose3 old_pose = keyed_poses_.at(next_key);
    const gtsam::Pose3 tf = new_pose.between(old_pose);
    
    PointCloud::Ptr transformed(new PointCloud);
    pcl::transformPointCloud(*next_scan, *transformed, tf.matrix());
    *scan_out += *transformed;
  }


  // Filter
  /*
  const float voxel_grid_size = 0.2f;
  pcl::VoxelGrid<pcl::PointXYZI> vox_grid;
  vox_grid.setInputCloud(scan_out);
  vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
  vox_grid.filter(*scan_out);
  */
}

void LaserLoopClosure::ComputeNormals(
    PointCloud::ConstPtr input,
    Normals::Ptr normals) {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> norm_est;
  norm_est.setInputCloud(input);
  norm_est.setSearchMethod(search_method);
  norm_est.setRadiusSearch(sac_normals_radius_);
  norm_est.setNumberOfThreads(icp_threads_);
  norm_est.compute(*normals);
}

void LaserLoopClosure::ComputeKeypoints(PointCloud::ConstPtr source,
                                        Normals::Ptr source_normals,
                                        PointCloud::Ptr source_keypoints) {
  pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> harris_detector;

  harris_detector.setNonMaxSupression(harris_suppression_);
  harris_detector.setRefine(harris_refine_);
  harris_detector.setInputCloud(source);
  harris_detector.setNormals(source_normals);
  harris_detector.setNumberOfThreads(icp_threads_);
  harris_detector.setRadius(harris_radius_);
  harris_detector.setThreshold(harris_threshold_);
  harris_detector.setMethod(
      static_cast<pcl::HarrisKeypoint3D<pcl::PointXYZI,
                                        pcl::PointXYZI>::ResponseMethod>(
          harris_response_));
  harris_detector.compute(*source_keypoints);
}

void LaserLoopClosure::ComputeFeatures(PointCloud::ConstPtr keypoints,
                                       PointCloud::ConstPtr input,
                                       Normals::Ptr normals,
                                       Features::Ptr features) {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZI>);
  pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>
      fpfh_est;
  fpfh_est.setInputCloud(keypoints);
  fpfh_est.setSearchSurface(input);
  fpfh_est.setInputNormals(normals);
  fpfh_est.setSearchMethod(search_method);
  fpfh_est.setRadiusSearch(sac_features_radius_);
  fpfh_est.setNumberOfThreads(icp_threads_);
  fpfh_est.compute(*features);
}

void LaserLoopClosure::GetInitialAlignment(
    PointCloud::ConstPtr source,
    PointCloud::ConstPtr target,
    Eigen::Matrix4f* tf_out,
    double& sac_fitness_score) {
  // Get Normals
  Normals::Ptr source_normals(new Normals);
  Normals::Ptr target_normals(new Normals);
  ComputeNormals(source, source_normals);
  ComputeNormals(target, target_normals);

  // Get Harris keypoints for source and target
  PointCloud::Ptr source_keypoints(new PointCloud);

  PointCloud::Ptr target_keypoints(new PointCloud);

  ComputeKeypoints(source, source_normals, source_keypoints);
  ComputeKeypoints(target, target_normals, target_keypoints);

  Features::Ptr source_features(new Features);
  Features::Ptr target_features(new Features);
  ComputeFeatures(source_keypoints, source, source_normals, source_features);
  ComputeFeatures(target_keypoints, target, target_normals, target_features);

  // Align
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia;
  sac_ia.setMaximumIterations(sac_iterations_);
  sac_ia.setInputSource(source_keypoints);
  sac_ia.setSourceFeatures(source_features);
  sac_ia.setInputTarget(target_keypoints);
  sac_ia.setTargetFeatures(target_features);
  PointCloud::Ptr aligned_output(new PointCloud);
  sac_ia.align(*aligned_output);

  sac_fitness_score = sac_ia.getFitnessScore();
  ROS_INFO_STREAM("SAC fitness score" << sac_fitness_score);

  *tf_out = sac_ia.getFinalTransformation();
}

bool LaserLoopClosure::FindLoopClosures(
    gtsam::Key new_key,
    std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {

  // If laser loop closures are off, exit immediately
  if (!b_check_for_loop_closures_)
    return false;


  // Look for loop closures for the latest received key
  // Don't check for loop closures against poses that are missing scans.
  if (!keyed_scans_.count(new_key)) {
    ROS_WARN_STREAM("Key " << gtsam::DefaultKeyFormatter(new_key)
                           << " does not have a scan");
    return false;
  }

  // Get pose and scan for the provided key.
  const gtsam::Pose3 pose1 = keyed_poses_.at(new_key);
  const PointCloud::ConstPtr scan1 = keyed_scans_.at(new_key);

  // Create a temporary copy of last_closure_key_map so that updates in this iteration are not used
  std::map<std::pair<char,char>, gtsam::Key> last_closure_key_copy_(last_closure_key_);

  // Set to true if we find a loop closure (single or inter robot)
  bool closed_loop = false;

  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    const gtsam::Symbol other_key = it->first;

    // If a loop has already been closed recently, don't try to close a new one.
    char c1 = gtsam::Symbol(new_key).chr(), c2 = other_key.chr();
    gtsam::Key last_closure_key_new = last_closure_key_copy_[{c1, c2}];

    // If a loop has already been closed recently, don't try to close a new one.
    if (std::llabs(new_key - last_closure_key_new) * translation_threshold_nodes_ <
        distance_before_reclosing_)
      continue;

    // Skip poses with no keyed scans.
    if (!keyed_scans_.count(other_key)) {
      continue;
    }

    // Check for single robot loop closures
    if (utils::IsKeyFromSameRobot(new_key, other_key)) {
      closed_loop |= CheckForLoopClosure(new_key, other_key, loop_closure_edges);
    }

    // Check for inter robot loop closures
    else {
      closed_loop |= CheckForInterRobotLoopClosure(new_key, other_key, loop_closure_edges);
    }
  }

  return closed_loop;
}

double LaserLoopClosure::DistanceBetweenKeys(gtsam::Symbol key1, gtsam::Symbol key2) {

  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);
  const gtsam::Pose3 delta = pose1.between(pose2);

  return delta.translation().norm();
}


bool LaserLoopClosure::CheckForLoopClosure(
        gtsam::Symbol key1,
        gtsam::Symbol key2,
        std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {

  if (!utils::IsKeyFromSameRobot(key1, key2)) {
    ROS_ERROR_STREAM("Checking for single robot loop closures on different robots");
    return false;
  }

  // Don't self-check.
  if (key1 == key2)
    return false;

  // Don't compare against poses that were recently collected.
  if (std::llabs(key1.index() - key2.index()) < skip_recent_poses_)
    return false;
  
  if (DistanceBetweenKeys(key1, key2) > proximity_threshold_) {
    return false;
  }

  // Perform loop closure without a provided prior transform
  return PerformLoopClosure(key1, key2, false, gtsam::Pose3(), loop_closure_edges);
}

bool LaserLoopClosure::CheckForInterRobotLoopClosure(
        gtsam::Symbol key1,
        gtsam::Symbol key2,
        std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {

  if (utils::IsKeyFromSameRobot(key1, key2)) {
    ROS_ERROR_STREAM("Checking for inter robot loop closures on same robot");
    return false;
  }

  // Don't self-check.
  if (key1 == key2)
    return false;
  
  if (DistanceBetweenKeys(key1, key2) > proximity_threshold_) {
    return false;
  }

  // Perform loop closure without a provided prior transform
  return PerformLoopClosure(key1, key2, false, gtsam::Pose3(), loop_closure_edges);
}

bool LaserLoopClosure::PerformLoopClosure(
        gtsam::Symbol key1,
        gtsam::Symbol key2,
        bool b_use_prior,
        gtsam::Pose3 prior,
        std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {

  if (key1 == key2) return false; // Don't perform loop closure on same node

  gu::Transform3 delta = utils::ToGu(prior);  // (Using BetweenFactor)
  gtsam::Matrix66 covariance = Eigen::MatrixXd::Zero(6,6);
  double fitness_score; // retrieve ICP fitness score if matched

  // Perform ICP
  if (PerformAlignment(
          key1, key2, b_use_prior, &delta, &covariance, fitness_score)) {
    
    ROS_INFO_STREAM("Closed loop between "
                    << gtsam::DefaultKeyFormatter(key1) << " and "
                    << gtsam::DefaultKeyFormatter(key2));
    ROS_INFO_STREAM("Translation (x,y,z): " 
                    << delta.translation.X() << ", " << delta.translation.Y() << ", " 
                    << delta.translation.Z()
                    << ", rotation (w,x,y,z): " << utils::ToGtsam(delta).rotation().quaternion().w()
                    << ", " << utils::ToGtsam(delta).rotation().quaternion().x()
                    << ", " << utils::ToGtsam(delta).rotation().quaternion().y()
                    << ", " << utils::ToGtsam(delta).rotation().quaternion().z());
    

    // Add the edge
    pose_graph_msgs::PoseGraphEdge edge = CreateLoopClosureEdge(key1, key2, delta, covariance);
    loop_closure_edges->push_back(edge);
    return true;
  }
  
  return false;

}

pose_graph_msgs::PoseGraphEdge LaserLoopClosure::CreateLoopClosureEdge(
        gtsam::Symbol key1, 
        gtsam::Symbol key2,
        geometry_utils::Transform3& delta, 
        gtsam::Matrix66& covariance) {

  // Store last time a new loop closure was added
  if (key1 > last_closure_key_[{key1.chr(), key2.chr()}]) {
    last_closure_key_[{key1.chr(), key2.chr()}] = key1;
  }
  if (key2 > last_closure_key_[{key2.chr(), key1.chr()}]) {
    last_closure_key_[{key2.chr(), key1.chr()}] = key2;
  }
  
  // Create the new loop closure edge
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key1;
  edge.key_to = key2;
  edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
  edge.pose = gr::ToRosPose(delta);

  // Convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      edge.covariance[6 * i + j] = covariance(i, j);
    }
  }

  return edge;
}

bool LaserLoopClosure::PerformAlignment(const gtsam::Symbol key1, 
                                        const gtsam::Symbol key2,
                                        bool b_use_delta_as_prior,
                                        gu::Transform3* delta,
                                        gtsam::Matrix66* covariance,
                                        double& fitness_score) {

  ROS_INFO_STREAM("Performing alignment between "
                  << gtsam::DefaultKeyFormatter(key1) << " and "
                  << gtsam::DefaultKeyFormatter(key2));

  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("PerformAlignment: Output pointers are null.");
    return false;
  }

  // Check for available information
  if (!keyed_poses_.count(key1) || !keyed_poses_.count(key2) || !keyed_scans_.count(key1) || !keyed_scans_.count(key2)) {
    ROS_WARN("Incomplete keyed poses/scans");
    return false;
  }

  // Get poses and keys
  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);  
  const PointCloud::ConstPtr scan1 = keyed_scans_.at(key1.key());
  const PointCloud::ConstPtr scan2 = keyed_scans_.at(key2.key());

  if (scan1 == NULL || scan2 == NULL) {
    ROS_ERROR("PerformAlignment: Null point clouds.");
    return false;
  }
  if (scan1->size() == 0 || scan2->size() == 0) {
    ROS_ERROR("PerformAlignment: zero points in point clouds.");
    return false;
  }
  if (scan1->points.empty() || scan2->points.empty()) {
    ROS_ERROR("PerformAlignment: empty point clouds.");
    return false;
  }
  // Set up ICP.
  pcl::MultithreadedGeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                     pcl::PointXYZI>
      icp;
  // setVerbosityLevel(pcl::console::L_DEBUG);
  SetupICP(icp);

  PointCloud::Ptr accumulated_target(new PointCloud);
  *accumulated_target = *scan2;
  AccumulateScans(key2, accumulated_target);

  icp.setInputSource(scan1);

  icp.setInputTarget(accumulated_target);

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
    ROS_INFO_STREAM("INITIAL GUESS:");
    pose_21.print();
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
  case IcpInitMethod::FEATURES:
  {
    double sac_fitness_score = sac_fitness_score_threshold_;
    GetInitialAlignment(scan1, accumulated_target, &initial_guess, sac_fitness_score);
    if (sac_fitness_score >= sac_fitness_score_threshold_) {
      ROS_INFO("SAC fitness score is too high");
      return false;
    }
  } break;
  
  default: // identity as default (default in ICP anyways)
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  }
  }

  // If an initial guess was provided, override the initialization
  if (b_use_delta_as_prior) {
    ROS_INFO_STREAM("Using initial guess provided by delta");
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    const gu::Transform3 guess = *delta; //gu::PoseInverse(*delta);

    ROS_INFO_STREAM("\tTranslation: " << guess.translation.X() << ", " << guess.translation.Y() << ", " << guess.translation.Z());
    ROS_INFO_STREAM("DELTA INITIAL GUESS:");
    utils::ToGtsam(guess).print();

    // Normalize the initial guess rotation
    Eigen::Quaternionf quat(guess.rotation.Eigen().cast<float>());
    quat.normalize();
    initial_guess.block(0, 0, 3, 3) = quat.matrix();    
    initial_guess.block(0, 3, 3, 1) = guess.translation.Eigen().cast<float>();
  }

  // Perform ICP.
  PointCloud unused_result;
  icp.align(unused_result, initial_guess);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp.getFinalTransformation();

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
    ROS_INFO_STREAM("ICP: No converged, score is: " << icp.getFitnessScore());
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) {
    ROS_INFO_STREAM("ICP: Coverged but score is: " << icp.getFitnessScore());
    return false;
  }

  // reject if the rotation is too big
  gu::Transform3 odom_delta = utils::ToGu(pose2.between(pose1));
  gtsam::Pose3 correction = utils::ToGtsam(gu::PoseDelta(*delta, odom_delta));
  if (fabs(2*acos(correction.rotation().toQuaternion().w()))  > max_rotation_rad_) {
    ROS_INFO_STREAM("Rejected loop closure - total rotation too large");
    return false;
  }  

  ROS_INFO_STREAM("ICP: Found loop with score: " << icp.getFitnessScore());
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

void LaserLoopClosure::SeedCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {

  // Edges to publish
  std::vector<pose_graph_msgs::PoseGraphEdge> loop_closure_edges;

  for (auto e : msg->edges) {
    ROS_INFO_STREAM("Received seeded loop closure between " << gtsam::DefaultKeyFormatter(e.key_from) << " and " << gtsam::DefaultKeyFormatter(e.key_to));

    gtsam::Symbol key1 = e.key_from;
    gtsam::Symbol key2 = e.key_to;

    // Check that scans exist 
    if (!keyed_scans_.count(key1) || !keyed_scans_.count(key2)) {
      ROS_WARN_STREAM("Could not seed loop closure - keys do not have scans");
      continue;
    }

    // If edge type is PRIOR, use the edge transform as a prior in ICP 
    bool b_use_prior = false;
    gtsam::Pose3 prior;
    ROS_INFO_STREAM("Edge type: " << e.type);
    if (e.type == pose_graph_msgs::PoseGraphEdge::PRIOR) {
      b_use_prior = true;
      prior = utils::ToGtsam(e.pose);
    }

    if (!PerformLoopClosure(key1, key2, b_use_prior, prior, &loop_closure_edges)){
      continue;
    }
  }

  if (msg->edges[0].type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN) {
    for (auto& edge : loop_closure_edges) {
      edge.type = pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN;
    }
  } 

  // Publish the successful edges if there are any
  if (loop_closure_edges.size() > 0) {
    PublishLoopClosures(loop_closure_edges);
  }
}

void LaserLoopClosure::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already has a scan. Not adding.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}

void LaserLoopClosure::TriggerGTCallback(const std_msgs::String::ConstPtr& msg){
  std::string filename = msg->data;

  ROS_INFO_STREAM("Generating point cloud ground truth using point cloud from " << filename);

  GenerateGTFromPC(filename);
}

void LaserLoopClosure::GenerateGTFromPC(std::string gt_pc_filename) {
  ROS_INFO("Triggering ground truth.\n");

  // Read ground truth from file
  pcl::PCDReader pcd_reader;
  PointCloud gt_point_cloud;
  pcd_reader.read(gt_pc_filename, gt_point_cloud);
  PointCloudConstPtr gt_pc_ptr(new PointCloud(gt_point_cloud));

  // Init pose-graph output
  std::vector<pose_graph_msgs::PoseGraphEdge> gt_edges;

  // Initialize variables
  PointCloud::Ptr keyed_scan_world(new PointCloud);
  gu::Transform3 delta;
  gtsam::Matrix66 covariance;
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = gt_rot_sigma_ * gt_rot_sigma_;
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = gt_trans_sigma_ * gt_trans_sigma_;

  // Set up ICP.
  pcl::MultithreadedGeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                     pcl::PointXYZI>
      icp;
  SetupICP(icp);
  icp.setInputTarget(gt_pc_ptr);

  // ---------------------------------------------------------
  // Loop through keyed poses
  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    ROS_INFO_STREAM("Processing key " << gtsam::DefaultKeyFormatter(it->first) << "\n");

    // Check if the keyed scan exists
    if (!keyed_scans_.count(it->first)){
      ROS_WARN_STREAM("No keyed scan for key " << gtsam::DefaultKeyFormatter(it->first));
      continue;
    }

    // Get scan and transform to the world frame
    gu::Transform3 transform = utils::ToGu(it->second);
    const Eigen::Matrix<double, 3, 3> Rot = transform.rotation.Eigen();
    const Eigen::Matrix<double, 3, 1> Trans = transform.translation.Eigen();
    Eigen::Matrix4d tf;
    tf.block(0, 0, 3, 3) = Rot;
    tf.block(0, 3, 3, 1) = Trans;

    // Transform point cloud to world frame
    pcl::transformPointCloud(*keyed_scans_[it->first], *keyed_scan_world, tf);
    
    // Publish current point cloud
    if (current_scan_pub_.getNumSubscribers() > 0) {
      PublishPointCloud(current_scan_pub_, *keyed_scan_world);
    }

    // Publish ground truth point cloud
    if (gt_pub_.getNumSubscribers() > 0) {
      PublishPointCloud(gt_pub_, gt_point_cloud);
    }

    // Set source
    icp.setInputSource(keyed_scan_world);

    // Perform ICP.
    PointCloud unused_result;
    icp.align(unused_result);

    // Get resulting transform.
    const Eigen::Matrix4f T = icp.getFinalTransformation();

    delta.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    delta.rotation = gu::Rot3(T(0, 0),
                              T(0, 1),
                              T(0, 2),
                              T(1, 0),
                              T(1, 1),
                              T(1, 2),
                              T(2, 0),
                              T(2, 1),
                              T(2, 2));

    // Check it ICP has passed
    if (!icp.hasConverged()) {
      ROS_INFO_STREAM("ICP GT, key " << gtsam::DefaultKeyFormatter(it->first) << " : Not converged, score is: " << icp.getFitnessScore());
      continue;
    }

    // Check our fitness threshold
    if (icp.getFitnessScore() > max_tolerable_fitness_) {
      ROS_INFO_STREAM("ICP GT, key " << gtsam::DefaultKeyFormatter(it->first) << ": Coverged but score is: " << icp.getFitnessScore());
      continue;
    }

    // reject if the rotation is too big
    if (fabs(2*acos(utils::ToGtsam(delta).rotation().toQuaternion().w()))  > max_rotation_rad_) {
      ROS_INFO_STREAM("Rejected GT loop closure - total rotation too large, key " << gtsam::DefaultKeyFormatter(it->first));
      continue;
    }

    // TODO Add translation check as well
    // Check covariances
    const gtsam::Pose3 odom_pose = keyed_poses_.at(it->first); 

    // Compose transform to make factor for optimization
    gtsam::Pose3 gc_factor = utils::ToGtsam(delta).compose(odom_pose);
    gu::Transform3 gc_factor_gu = utils::ToGu(gc_factor);

    // Publish aligned scan
    if (aligned_scan_pub_.getNumSubscribers() > 0) {
      Eigen::Matrix4d tf_align;
      const Eigen::Matrix<double, 3, 3> Rot_gu = gc_factor_gu.rotation.Eigen();
      const Eigen::Matrix<double, 3, 1> Trans_gu =
          gc_factor_gu.translation.Eigen();

      tf_align.block(0, 0, 3, 3) = Rot_gu;
      tf_align.block(0, 3, 3, 1) = Trans_gu;

      PointCloud aligned_cloud;
      pcl::transformPointCloud(
          *keyed_scans_[it->first], aligned_cloud, tf_align);
      PublishPointCloud(aligned_scan_pub_, aligned_cloud);
    }

    // Make prior here
    pose_graph_msgs::PoseGraphEdge edge = CreatePriorEdge(it->first, gc_factor_gu, covariance);
    ROS_INFO_STREAM("The added edge is " << gtsam::DefaultKeyFormatter(edge.key_from));
    // Push to gt_prior
    gt_edges.push_back(edge);
  }

  // Publish the new edges and a node with prior for the origin
  if (gt_edges.size() > 0) {
    // Publish Loop Closures
    ROS_INFO_STREAM("Publishing " << gt_edges.size() << " edges.\n");
    pose_graph_msgs::PoseGraph graph;
    graph.edges = gt_edges; //gt_prior
    loop_closure_pub_.publish(graph);
  }
}

pose_graph_msgs::PoseGraphEdge LaserLoopClosure::CreatePriorEdge(
                gtsam::Symbol key,
                geometry_utils::Transform3& delta, 
                gtsam::Matrix66& covariance) {
  pose_graph_msgs::PoseGraphEdge prior;
  prior.key_from = key;
  prior.key_to = key;
  prior.type = pose_graph_msgs::PoseGraphEdge::PRIOR;
  prior.pose.position.x = delta.translation.X();
  prior.pose.position.y = delta.translation.Y();
  prior.pose.position.z = delta.translation.Z();
  prior.pose.orientation.w = utils::ToGtsam(delta).rotation().quaternion()[0];
  prior.pose.orientation.x = utils::ToGtsam(delta).rotation().quaternion()[1];
  prior.pose.orientation.y = utils::ToGtsam(delta).rotation().quaternion()[2];
  prior.pose.orientation.z = utils::ToGtsam(delta).rotation().quaternion()[3];

  // Convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      if (i == j){
        prior.covariance[6 * i + j] = covariance(i,j);
      }
    }
  }
  return prior;
}

bool LaserLoopClosure::SetupICP(
    pcl::MultithreadedGeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                       pcl::PointXYZI>& icp) {
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);
  icp.setMaximumOptimizerIterations(50);
  icp.setNumThreads(icp_threads_);
  icp.enableTimingOutput(true);
  return true;
}

void LaserLoopClosure::PublishPointCloud(ros::Publisher& pub,
                                         PointCloud& cloud) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  pub.publish(msg);
}