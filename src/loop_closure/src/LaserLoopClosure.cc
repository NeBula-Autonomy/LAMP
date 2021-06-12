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
#include <pcl/common/io.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>

#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <teaser/ply_io.h>

#include <Eigen/Core>


namespace pu = parameter_utils;
namespace gu = geometry_utils;

LaserLoopClosure::LaserLoopClosure(const ros::NodeHandle& n) : LoopClosure(n) {}

LaserLoopClosure::~LaserLoopClosure() {}

bool LaserLoopClosure::Initialize(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());

  // If laser loop closures are off, exit without setting up publishers or
  // subscribers
  if (!pu::Get(param_ns_ + "/b_find_laser_loop_closures",
               b_check_for_loop_closures_))
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
  lc_computation_time_pub_ = nl.advertise<std_msgs::Float64>(
      "loop_closure_computation_time", 100, false);
  // Parameters
  double distance_to_skip_recent_poses;
  // Load loop closing parameters.
  if (!pu::Get(param_ns_ + "/b_check_observability", b_check_observability_))
    return false;
  if (!pu::Get(param_ns_ + "/min_observability_ratio",
               min_observability_ratio_))
    return false;
  if (!pu::Get(param_ns_ + "/translation_threshold_nodes",
               translation_threshold_nodes_))
    return false;
  if (!pu::Get(param_ns_ + "/proximity_threshold", proximity_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/max_tolerable_fitness", max_tolerable_fitness_))
    return false;
  if (!pu::Get(param_ns_ + "/distance_to_skip_recent_poses",
               distance_to_skip_recent_poses))
    return false;
  if (!pu::Get(param_ns_ + "/distance_before_reclosing",
               distance_before_reclosing_))
    return false;
  if (!pu::Get(param_ns_ + "/max_rotation_deg", max_rotation_deg_))
    return false;
  max_rotation_rad_ = max_rotation_deg_ * M_PI / 180;

  // Load ICP parameters (from point_cloud localization)
  if (!pu::Get(param_ns_ + "/icp_lc/tf_epsilon", icp_tf_epsilon_))
    return false;
  if (!pu::Get(param_ns_ + "/icp_lc/corr_dist", icp_corr_dist_))
    return false;
  if (!pu::Get(param_ns_ + "/icp_lc/iterations", icp_iterations_))
    return false;
  if (!pu::Get(param_ns_ + "/icp_lc/threads", icp_threads_))
    return false;

  // Load SAC parameters
  if (!pu::Get(param_ns_ + "/sac_ia/iterations", sac_iterations_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_prev_scans", sac_num_prev_scans_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_next_scans", sac_num_next_scans_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/normals_radius", sac_normals_radius_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/features_radius", sac_features_radius_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/fitness_score_threshold",
               sac_fitness_score_threshold_))
    return false;

  // Load TEASER parameters
  if (!pu::Get(param_ns_ + "/TEASERPP/num_inlier_threshold", teaser_inlier_threshold_)) return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/rotation_cost_threshold", rotation_cost_threshold_)) return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/rotation_max_iterations", rotation_max_iterations_)) return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/noise_bound", noise_bound_)) return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/TEASER_FPFH_normals_radius", TEASER_FPFH_normals_radius_)) return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/TEASER_FPFH_features_radius", TEASER_FPFH_features_radius_)) return false;

  // Load Harris parameters
  if (!pu::Get(param_ns_ + "/harris3D/harris_threshold",
               harris_params_.harris_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_suppression",
               harris_params_.harris_suppression_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_radius",
               harris_params_.harris_radius_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_refine",
               harris_params_.harris_refine_))
    return false;
  if (!pu::Get(param_ns_ + "/harris3D/harris_response",
               harris_params_.harris_response_))
    return false;

  // Hard coded covariances
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;
  if (!pu::Get("b_use_fixed_covariances", b_use_fixed_covariances_))
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

  int icp_covar_method;
  if (!pu::Get(param_ns_ + "/icp_covariance_calculation", icp_covar_method))
    return false;
  icp_covariance_method_ = IcpCovarianceMethod(icp_covar_method);

  skip_recent_poses_ =
      (int)(distance_to_skip_recent_poses / translation_threshold_nodes_);

  SetupICP();
  return true;
}

void LaserLoopClosure::PublishLCComputationTime(
    const double& lc_computation_time,
    const ros::Publisher& pub) {
  // Convert slipage value value to ROS format and publish.
  std_msgs::Float64 computation_time;
  computation_time.data = lc_computation_time;
  pub.publish(computation_time);
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
}

void LaserLoopClosure::GetInitialAlignment(PointCloud::ConstPtr source,
                                           PointCloud::ConstPtr target,
                                           Eigen::Matrix4f* tf_out,
                                           double& sac_fitness_score) {
  // Get Normals
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  Normals::Ptr source_normals(new Normals);
  Normals::Ptr target_normals(new Normals);
  //  utils::ComputeNormals(
  //      source, sac_normals_radius_, icp_threads_, source_normals);
  //  utils::ComputeNormals(
  //      target, sac_normals_radius_, icp_threads_, target_normals);
  utils::ComputeNormals(source, icp_threads_, source_normals);
  utils::ComputeNormals(target, icp_threads_, target_normals);

  // Get Harris keypoints for source and target
  PointCloud::Ptr source_keypoints(new PointCloud);
  PointCloud::Ptr target_keypoints(new PointCloud);
  utils::ComputeKeypoints(
      source, harris_params_, icp_threads_, source_normals, source_keypoints);
  utils::ComputeKeypoints(
      target, harris_params_, icp_threads_, target_normals, target_keypoints);

  Features::Ptr source_features(new Features);
  Features::Ptr target_features(new Features);
  utils::ComputeFeatures(source_keypoints,
                         source,
                         sac_features_radius_,
                         icp_threads_,
                         source_normals,
                         source_features);
  utils::ComputeFeatures(target_keypoints,
                         target,
                         sac_features_radius_,
                         icp_threads_,
                         target_normals,
                         target_features);


  // Align
  pcl::SampleConsensusInitialAlignment<Point, Point, pcl::FPFHSignature33>
      sac_ia;
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
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;
  PublishLCComputationTime(duration, lc_computation_time_pub_);

}

void LaserLoopClosure::GetTEASERInitialAlignment(
    PointCloud::ConstPtr source,
    PointCloud::ConstPtr target,
    Eigen::Matrix4f* tf_out,
    double& n_inliers) {
  
  // Convert to teaser point cloud
  teaser::PointCloud src_cloud;
  for (pcl::PointCloud<Point>::const_iterator it = source->points.begin();
       it != source->points.end();
       it++) {
    src_cloud.push_back({it->x, it->y, it->z});
       }

  teaser::PointCloud tgt_cloud;
  for (pcl::PointCloud<Point>::const_iterator it = target->points.begin();
       it != target->points.end();
       it++) {
    tgt_cloud.push_back({it->x, it->y, it->z});
       }

  
  // Get Normals
  // Normals::Ptr source_normals(new Normals);
  // Normals::Ptr target_normals(new Normals);
  // ComputeNormals(source, source_normals);
  // ComputeNormals(target, target_normals);

  // // Get Harris keypoints for source and target
  // PointCloud::Ptr source_keypoints(new PointCloud);
  // PointCloud::Ptr target_keypoints(new PointCloud);
  // ComputeKeypoints(source, source_normals, source_keypoints);
  // ComputeKeypoints(target, target_normals, target_keypoints);

  // Features::Ptr source_features(new Features);
  // Features::Ptr target_features(new Features);
  // ComputeFeatures(source_keypoints, source, source_normals, source_features);
  // ComputeFeatures(target_keypoints, target, target_normals, target_features);
  // teaser::FPFHCloud src_descriptors;
  // teaser::FPFHCloud target_descriptors;
  // teaser::FPFHCloudPtr src_descriptors(new Features);
  // teaser::FPFHCloudPtr target_descriptors(new Features);
 
  // pcl::copyPointCloud(*source_features, *src_descriptors);
  // pcl::copyPointCloud(*target_features, *target_descriptors);
  

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  auto src_descriptors = fpfh.computeFPFHFeatures(src_cloud, TEASER_FPFH_normals_radius_, TEASER_FPFH_features_radius_);
  auto target_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, TEASER_FPFH_normals_radius_, TEASER_FPFH_features_radius_);
  
  // Align 
  teaser::Matcher matcher;
  ROS_INFO("Before TEASER matcher");
  auto correspondences = matcher.calculateCorrespondences(
      src_cloud, tgt_cloud, *src_descriptors, *target_descriptors, false, true, false, 0.95);
  ROS_INFO("After TEASER matcher");
  int corres_size = correspondences.size();
  // ROS_INFO_STREAM("Correspondence Size is: "<< corres_size);
  
  // Retrive the corresponding points from src and tgt point clouds into two 3-by-N Eigen matrices
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_corres_points(3, corres_size);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_corres_points(3, corres_size);
  for (size_t i = 0; i < corres_size; ++i) {
    src_corres_points.col(i) << src_cloud[correspondences[i].first].x, src_cloud[correspondences[i].first].y, src_cloud[correspondences[i].first].z;
    tgt_corres_points.col(i) << tgt_cloud[correspondences[i].second].x, tgt_cloud[correspondences[i].second].y, tgt_cloud[correspondences[i].second].z;
  }
  // ROS_INFO_STREAM("Matrix src is: "<< src_corres_points);
  // ROS_INFO("Completed TEASER Correspondences!");
  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = noise_bound_;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = rotation_max_iterations_;
  params.rotation_gnc_factor = 1.4;
  // ROS_INFO("Finding TEASER Rigid Transform!");
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = rotation_cost_threshold_; // 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  solver.solve(src_corres_points, tgt_corres_points);
  // ROS_INFO("Solved TEASER Rigid Transform!");
  auto solution = solver.getSolution();
  Eigen::Matrix4d T;
  T.topLeftCorner(3, 3) = solution.rotation;
  T.topRightCorner(3, 1) = solution.translation;
  *tf_out = T.cast <float> ();
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  auto final_inliers = solver.getInlierMaxClique();
  n_inliers = final_inliers.size();
  double duration = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000000.0;
  PublishLCComputationTime(duration, lc_computation_time_pub_);
  // ROS_INFO_STREAM("TEASER++ number of inliers" << n_inliers);
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

  if (b_check_observability_) {
    Eigen::Matrix<double, 3, 1> obs_eigenvals;
    ComputeIcpObservability(scan1, &obs_eigenvals);
    double obs_ratio = obs_eigenvals.minCoeff() / obs_eigenvals.maxCoeff();
    if (obs_ratio < min_observability_ratio_)
      return false;
  }

  // Create a temporary copy of last_closure_key_map so that updates in this
  // iteration are not used
  std::map<std::pair<char, char>, gtsam::Key> last_closure_key_copy_(
      last_closure_key_);

  // Set to true if we find a loop closure (single or inter robot)
  bool closed_loop = false;

  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    const gtsam::Symbol other_key = it->first;

    // If a loop has already been closed recently, don't try to close a new one.
    char c1 = gtsam::Symbol(new_key).chr(), c2 = other_key.chr();
    gtsam::Key last_closure_key_new = last_closure_key_copy_[{c1, c2}];

    // If a loop has already been closed recently, don't try to close a new one.
    if (std::llabs(new_key - last_closure_key_new) *
            translation_threshold_nodes_ <
        distance_before_reclosing_)
      continue;

    // Skip poses with no keyed scans.
    if (!keyed_scans_.count(other_key)) {
      continue;
    }

    // Check for single robot loop closures
    if (utils::IsKeyFromSameRobot(new_key, other_key)) {
      closed_loop |=
          CheckForLoopClosure(new_key, other_key, false, loop_closure_edges);
    }

    // Check for inter robot loop closures
    else {
      closed_loop |=
          CheckForLoopClosure(new_key, other_key, true, loop_closure_edges);
    }
  }

  return closed_loop;
}

double LaserLoopClosure::DistanceBetweenKeys(gtsam::Symbol key1,
                                             gtsam::Symbol key2) {
  const gtsam::Pose3 pose1 = keyed_poses_.at(key1);
  const gtsam::Pose3 pose2 = keyed_poses_.at(key2);
  const gtsam::Pose3 delta = pose1.between(pose2);

  return delta.translation().norm();
}

bool LaserLoopClosure::CheckForLoopClosure(
    gtsam::Symbol key1,
    gtsam::Symbol key2,
    bool b_inter_robot,
    std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {
  if (!b_inter_robot && !utils::IsKeyFromSameRobot(key1, key2)) {
    ROS_ERROR_STREAM(
        "Checking for single robot loop closures on different robots");
    return false;
  }

  // Don't self-check.
  if (key1 == key2)
    return false;

  // Don't compare against poses that were recently collected.
  if (!b_inter_robot &&
      std::llabs(key1.index() - key2.index()) < skip_recent_poses_)
    return false;

  if (DistanceBetweenKeys(key1, key2) > proximity_threshold_) {
    return false;
  }

  // Perform loop closure without a provided prior transform
  return PerformLoopClosure(
      key1, key2, false, gtsam::Pose3(), loop_closure_edges);
}

bool LaserLoopClosure::PerformLoopClosure(
    gtsam::Symbol key1,
    gtsam::Symbol key2,
    bool b_use_prior,
    gtsam::Pose3 prior,
    std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) {
  if (key1 == key2)
    return false; // Don't perform loop closure on same node

  gu::Transform3 delta = utils::ToGu(prior); // (Using BetweenFactor)
  gtsam::Matrix66 covariance = Eigen::MatrixXd::Zero(6, 6);
  double fitness_score; // retrieve ICP fitness score if matched

  // Perform ICP
  if (PerformAlignment(
          key1, key2, b_use_prior, &delta, &covariance, fitness_score)) {
    ROS_INFO_STREAM("Closed loop between " << gtsam::DefaultKeyFormatter(key1)
                                           << " and "
                                           << gtsam::DefaultKeyFormatter(key2));
    ROS_INFO_STREAM("Translation (x,y,z): "
                    << delta.translation.X() << ", " << delta.translation.Y()
                    << ", " << delta.translation.Z() << ", rotation (w,x,y,z): "
                    << utils::ToGtsam(delta).rotation().quaternion().w() << ", "
                    << utils::ToGtsam(delta).rotation().quaternion().x() << ", "
                    << utils::ToGtsam(delta).rotation().quaternion().y() << ", "
                    << utils::ToGtsam(delta).rotation().quaternion().z());

    // Add the edge
    pose_graph_msgs::PoseGraphEdge edge =
        CreateLoopClosureEdge(key1, key2, delta, covariance);
    loop_closure_edges->push_back(edge);
    return true;
  }

  return false;
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
  if (!keyed_poses_.count(key1) || !keyed_poses_.count(key2) ||
      !keyed_scans_.count(key1) || !keyed_scans_.count(key2)) {
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
  // setVerbosityLevel(pcl::console::L_DEBUG);

  PointCloud::Ptr accumulated_target(new PointCloud);
  *accumulated_target = *scan2;
  AccumulateScans(key2, accumulated_target);

  icp_.setInputSource(scan1);

  icp_.setInputTarget(accumulated_target);

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
        pose_21.translation().matrix().cast<float>();
  } break;

  case IcpInitMethod::ODOM_ROTATION: // initialize with zero translation but
                                     // rot from odom
  {
    gtsam::Pose3 pose_21 = pose2.between(pose1);
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
  } break;
  case IcpInitMethod::FEATURES: {
    double sac_fitness_score = sac_fitness_score_threshold_;
    GetInitialAlignment(
        scan1, accumulated_target, &initial_guess, sac_fitness_score);
    if (sac_fitness_score >= sac_fitness_score_threshold_) {
      ROS_INFO("SAC fitness score is too high");
      return false;
    }
  } break;

  case IcpInitMethod::TEASERPP:
  {
    ROS_INFO("TEASERPP is selected!");
    double n_inliers = teaser_inlier_threshold_;
    ROS_INFO_STREAM("teaser_inlier_threshold " << teaser_inlier_threshold_);
    GetTEASERInitialAlignment(scan1, accumulated_target, &initial_guess, n_inliers);
    if (n_inliers <= teaser_inlier_threshold_) {
      ROS_INFO_STREAM("Number of TEASER inliers is too low" << n_inliers);
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
    const gu::Transform3 guess = *delta; // gu::PoseInverse(*delta);

    ROS_INFO_STREAM("\tTranslation: " << guess.translation.X() << ", "
                                      << guess.translation.Y() << ", "
                                      << guess.translation.Z());
    ROS_INFO_STREAM("DELTA INITIAL GUESS:");
    utils::ToGtsam(guess).print();

    // Normalize the initial guess rotation
    Eigen::Quaternionf quat(guess.rotation.Eigen().cast<float>());
    quat.normalize();
    initial_guess.block(0, 0, 3, 3) = quat.matrix();
    initial_guess.block(0, 3, 3, 1) = guess.translation.Eigen().cast<float>();
  }

  // Perform ICP_.
  PointCloud::Ptr icp_result(new PointCloud);
  icp_.align(*icp_result, initial_guess);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp_.getFinalTransformation();

  // Get the correspondence indices
  std::vector<size_t> correspondences;
  if (icp_covariance_method_ == IcpCovarianceMethod::POINT2PLANE) {
    // KdTree<PointXY::Ptr search_tree = icp_.getSearchMethodTarget();
    pcl::search::KdTree<Point>::Ptr search_tree = icp_.getSearchMethodTarget();
    for (auto point : icp_result->points) {
      std::vector<int> matched_indices;
      std::vector<float> matched_distances;
      search_tree->nearestKSearch(point, 1, matched_indices, matched_distances);
      correspondences.push_back(matched_indices[0]);
    }
  }

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
  if (!icp_.hasConverged()) {
    ROS_INFO_STREAM("ICP: No converged, score is: " << icp_.getFitnessScore());
    return false;
  }

  if (icp_.getFitnessScore() > max_tolerable_fitness_) {
    ROS_INFO_STREAM("ICP: Coverged but score is: " << icp_.getFitnessScore());
    return false;
  }

  // reject if the rotation is too big
  gu::Transform3 odom_delta = utils::ToGu(pose2.between(pose1));
  gtsam::Pose3 correction = utils::ToGtsam(gu::PoseDelta(*delta, odom_delta));
  if (fabs(2 * acos(correction.rotation().toQuaternion().w())) >
      max_rotation_rad_) {
    ROS_INFO_STREAM("Rejected loop closure - total rotation too large");
    return false;
  }

  ROS_INFO_STREAM("ICP: Found loop with score: " << icp_.getFitnessScore());
  fitness_score = icp_.getFitnessScore();

  // Find transform from pose2 to pose1 from output of ICP_.
  *delta = gu::PoseInverse(*delta); // NOTE: gtsam need 2_Transform_1 while
                                    // ICP output 1_Transform_2

  // TODO: Use real ICP covariance.
  if (b_use_fixed_covariances_) {
    for (int i = 0; i < 3; ++i)
      (*covariance)(i, i) = laser_lc_rot_sigma_ * laser_lc_rot_sigma_;
    for (int i = 3; i < 6; ++i)
      (*covariance)(i, i) = laser_lc_trans_sigma_ * laser_lc_trans_sigma_;
  } else {
    switch (icp_covariance_method_) {
    case (IcpCovarianceMethod::POINT2POINT):
      ComputeICPCovariancePointPoint(icp_result, T, fitness_score, *covariance);
      break;
    case (IcpCovarianceMethod::POINT2PLANE):
      ComputeICPCovariancePointPlane(
          scan1, accumulated_target, correspondences, T, covariance);
      break;
    default:
      ROS_ERROR(
          "Unknown method for ICP covariance calculation for loop closures. "
          "Check config.");
    }
  }

  return true;
}

void LaserLoopClosure::SeedCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  // Edges to publish
  std::vector<pose_graph_msgs::PoseGraphEdge> loop_closure_edges;

  for (auto e : msg->edges) {
    ROS_INFO_STREAM("Received seeded loop closure between "
                    << gtsam::DefaultKeyFormatter(e.key_from) << " and "
                    << gtsam::DefaultKeyFormatter(e.key_to));

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

    if (!PerformLoopClosure(
            key1, key2, b_use_prior, prior, &loop_closure_edges)) {
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

  pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}

void LaserLoopClosure::TriggerGTCallback(
    const std_msgs::String::ConstPtr& msg) {
  std::string filename = msg->data;

  ROS_INFO_STREAM("Generating point cloud ground truth using point cloud from "
                  << filename);

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

  // Set up ICP_.
  pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point> icp;
  icp_.setInputTarget(gt_pc_ptr);

  // ---------------------------------------------------------
  // Loop through keyed poses
  for (auto it = keyed_poses_.begin(); it != keyed_poses_.end(); ++it) {
    ROS_INFO_STREAM("Processing key " << gtsam::DefaultKeyFormatter(it->first)
                                      << "\n");

    // Check if the keyed scan exists
    if (!keyed_scans_.count(it->first)) {
      ROS_WARN_STREAM("No keyed scan for key "
                      << gtsam::DefaultKeyFormatter(it->first));
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
    icp_.setInputSource(keyed_scan_world);

    // Perform ICP_.
    PointCloud unused_result;
    icp_.align(unused_result);

    // Get resulting transform.
    const Eigen::Matrix4f T = icp_.getFinalTransformation();

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
    if (!icp_.hasConverged()) {
      ROS_INFO_STREAM("ICP GT, key " << gtsam::DefaultKeyFormatter(it->first)
                                     << " : Not converged, score is: "
                                     << icp_.getFitnessScore());
      continue;
    }

    // Check our fitness threshold
    if (icp_.getFitnessScore() > max_tolerable_fitness_) {
      ROS_INFO_STREAM("ICP GT, key "
                      << gtsam::DefaultKeyFormatter(it->first)
                      << ": Coverged but score is: " << icp_.getFitnessScore());
      continue;
    }

    // reject if the rotation is too big
    if (fabs(2 * acos(utils::ToGtsam(delta).rotation().toQuaternion().w())) >
        max_rotation_rad_) {
      ROS_INFO_STREAM(
          "Rejected GT loop closure - total rotation too large, key "
          << gtsam::DefaultKeyFormatter(it->first));
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
    pose_graph_msgs::PoseGraphEdge edge =
        CreatePriorEdge(it->first, gc_factor_gu, covariance);
    ROS_INFO_STREAM("The added edge is "
                    << gtsam::DefaultKeyFormatter(edge.key_from));
    // Push to gt_prior
    gt_edges.push_back(edge);
  }

  // Publish the new edges and a node with prior for the origin
  if (gt_edges.size() > 0) {
    // Publish Loop Closures
    ROS_INFO_STREAM("Publishing " << gt_edges.size() << " edges.\n");
    pose_graph_msgs::PoseGraph graph;
    graph.edges = gt_edges; // gt_prior
    loop_closure_pub_.publish(graph);
  }
}

bool LaserLoopClosure::SetupICP() {
  icp_.setTransformationEpsilon(icp_tf_epsilon_);
  icp_.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp_.setMaximumIterations(icp_iterations_);
  icp_.setRANSACIterations(0);
  icp_.setMaximumOptimizerIterations(50);
  icp_.setNumThreads(icp_threads_);
  icp_.enableTimingOutput(true);
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

bool LaserLoopClosure::ComputeICPCovariancePointPoint(
    const PointCloud::ConstPtr& pointCloud,
    const Eigen::Matrix4f& T,
    const double& icp_fitness,
    Eigen::Matrix<double, 6, 6>& covariance) {
  geometry_utils::Transform3 ICP_transformation;

  // Extract translation values from T
  double t_x = T(0, 3);
  double t_y = T(1, 3);
  double t_z = T(2, 3);

  // Extract roll, pitch and yaw from T
  ICP_transformation.rotation = gu::Rot3(T(0, 0),
                                         T(0, 1),
                                         T(0, 2),
                                         T(1, 0),
                                         T(1, 1),
                                         T(1, 2),
                                         T(2, 0),
                                         T(2, 1),
                                         T(2, 2));
  double r = ICP_transformation.rotation.Roll();
  double p = ICP_transformation.rotation.Pitch();
  double y = ICP_transformation.rotation.Yaw();

  // Symbolic expression of the Jacobian matrix
  double J11, J12, J13, J14, J15, J16, J21, J22, J23, J24, J25, J26, J31, J32,
      J33, J34, J35, J36;

  Eigen::Matrix<double, 6, 6> H;
  H = Eigen::MatrixXd::Zero(6, 6);

  // Compute the entries of Jacobian
  // Entries of Jacobian matrix are obtained from MATLAB Symbolic Toolbox
  for (size_t i = 0; i < pointCloud->points.size(); ++i) {
    double p_x = pointCloud->points[i].x;
    double p_y = pointCloud->points[i].y;
    double p_z = pointCloud->points[i].z;

    J11 = 0.0;
    J12 = -2.0 *
        (p_z * sin(p) + p_x * cos(p) * cos(y) - p_y * cos(p) * sin(y)) *
        (t_x - p_x + p_z * cos(p) - p_x * cos(y) * sin(p) +
         p_y * sin(p) * sin(y));
    J13 = 2.0 * (p_y * cos(y) * sin(p) + p_x * sin(p) * sin(y)) *
        (t_x - p_x + p_z * cos(p) - p_x * cos(y) * sin(p) +
         p_y * sin(p) * sin(y));
    J14 = 2.0 * t_x - 2.0 * p_x + 2.0 * p_z * cos(p) -
        2.0 * p_x * cos(y) * sin(p) + 2.0 * p_y * sin(p) * sin(y);
    J15 = 0.0;
    J16 = 0.0;

    J21 = 2.0 *
        (p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
         p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
         p_z * sin(p) * sin(r)) *
        (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
         p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
         p_z * cos(r) * sin(p));
    J22 = -2.0 *
        (p_z * cos(p) * cos(r) - p_x * cos(r) * cos(y) * sin(p) +
         p_y * cos(r) * sin(p) * sin(y)) *
        (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
         p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
         p_z * cos(r) * sin(p));
    J23 = 2.0 *
        (p_x * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
         p_y * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y))) *
        (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
         p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
         p_z * cos(r) * sin(p));
    J24 = 0.0;
    J25 = 2.0 * t_y - 2.0 * p_y -
        2.0 * p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) -
        2.0 * p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) +
        2.0 * p_z * cos(r) * sin(p);
    J26 = 0.0;

    J31 = -2.0 *
        (p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
         p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
         p_z * cos(r) * sin(p)) *
        (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
         p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
         p_z * sin(p) * sin(r));
    J32 = 2.0 *
        (p_z * cos(p) * sin(r) - p_x * cos(y) * sin(p) * sin(r) +
         p_y * sin(p) * sin(r) * sin(y)) *
        (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
         p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
         p_z * sin(p) * sin(r));
    J33 = 2.0 *
        (p_x * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) -
         p_y * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r))) *
        (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
         p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
         p_z * sin(p) * sin(r));
    J34 = 0.0;
    J35 = 0.0;
    J36 = 2.0 * t_z - 2.0 * p_z +
        2.0 * p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
        2.0 * p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
        2.0 * p_z * sin(p) * sin(r);

    // Form the 3X6 Jacobian matrix
    Eigen::Matrix<double, 3, 6> J;
    J << J11, J12, J13, J14, J15, J16, J21, J22, J23, J24, J25, J26, J31, J32,
        J33, J34, J35, J36;
    // Compute J'XJ (6X6) matrix and keep adding for all the points in the point
    // cloud
    H += J.transpose() * J;
  }
  covariance = H.inverse() * icp_fitness;

  // Here bound the covariance using eigen values
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(covariance);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  double lower_bound = 0.001; // Should be positive semidef
  double upper_bound = 1000;
  if (eigen_values.size() < 6) {
    covariance = Eigen::MatrixXd::Identity(6, 6) * upper_bound;
    ROS_ERROR("Failed to find eigen values when computing icp covariance");
    return false;
  }
  for (size_t i = 0; i < 6; i++) {
    if (eigen_values[i] < lower_bound)
      eigen_values[i] = lower_bound;
    if (eigen_values[i] > upper_bound)
      eigen_values[i] = upper_bound;
  }
  // Update covariance matrix after bound
  covariance =
      eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.inverse();

  return true;
}

bool LaserLoopClosure::ComputeICPCovariancePointPlane(
    const PointCloud::ConstPtr& query_cloud,
    const PointCloud::ConstPtr& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* covariance) {
  // Get normals
  Normals::Ptr reference_normals(new Normals);      // pc with normals
  PointCloud::Ptr query_normalized(new PointCloud); // pc whose points have
                                                    // been rearranged.
  Eigen::Matrix<double, 6, 6> Ap;

  // utils::ComputeNormals(
  //    reference_cloud, sac_normals_radius_, icp_threads_, reference_normals);

  utils::ComputeNormals(reference_cloud, icp_threads_, reference_normals);

  utils::NormalizePCloud(query_cloud, query_normalized);

  ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_normals, correspondences, T, Ap);
  // 1 cm covariance for now hard coded
  *covariance = 0.01 * 0.01 * Ap.inverse();

  // Here bound the covariance using eigen values
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(*covariance);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  double lower_bound = 0.001; // Should be positive semidef
  double upper_bound = 1000;
  if (eigen_values.size() < 6) {
    *covariance = Eigen::MatrixXd::Identity(6, 6) * upper_bound;
    ROS_ERROR("Failed to find eigen values when computing icp covariance");
    return false;
  }
  for (size_t i = 0; i < eigen_values.size(); i++) {
    if (eigen_values(i) < lower_bound)
      eigen_values(i) = lower_bound;
    if (eigen_values(i) > upper_bound)
      eigen_values(i) = upper_bound;
  }
  // Update covariance matrix after bound
  *covariance =
      eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.inverse();
  return true;
}

void LaserLoopClosure::ComputeIcpObservability(
    PointCloud::ConstPtr cloud, Eigen::Matrix<double, 3, 1>* eigenvalues) {
  // Get normals
  Normals::Ptr normals(new Normals);          // pc with normals
  PointCloud::Ptr normalized(new PointCloud); // pc whose points have been
                                              // rearranged.
  // utils::ComputeNormals(cloud, sac_normals_radius_, icp_threads_, normals);
  utils::ComputeNormals(cloud, icp_threads_, normals);
  utils::NormalizePCloud(cloud, normalized);

  // Correspondence with itself (not really used anyways)
  std::vector<size_t> c(cloud->size());
  std::iota(std::begin(c), std::end(c), 0); // Fill with 0, 1, ...

  Eigen::Matrix4f T_unsued = Eigen::Matrix4f::Zero(); // Unused

  Eigen::Matrix<double, 6, 6> Ap;
  // Compute Ap and its eigenvalues
  ComputeAp_ForPoint2PlaneICP(normalized, normals, c, T_unsued, Ap);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(
      Ap.block(3, 3, 3, 3));
  if (eigensolver.info() == Eigen::Success) {
    *eigenvalues = eigensolver.eigenvalues();
  } else {
    ROS_WARN("Failed to decompose observability matrix. ");
  }
}

void LaserLoopClosure::ComputeAp_ForPoint2PlaneICP(
    const PointCloud::Ptr query_normalized,
    const Normals::Ptr reference_normals,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>& Ap) {
  Ap = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> A_i = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Vector3d a_i, n_i;
  for (uint32_t i = 0; i < query_normalized->size(); i++) {
    a_i << query_normalized->points[i].x, //////
        query_normalized->points[i].y,    //////
        query_normalized->points[i].z;

    n_i << reference_normals->points[correspondences[i]].normal_x, //////
        reference_normals->points[correspondences[i]].normal_y,    //////
        reference_normals->points[correspondences[i]].normal_z;

    if (a_i.hasNaN() || n_i.hasNaN())
      continue;

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H.block(0, 0, 1, 3) = (a_i.cross(n_i)).transpose();
    H.block(0, 3, 1, 3) = n_i.transpose();
    Ap += H.transpose() * H;
  }
}
