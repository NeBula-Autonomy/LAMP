/**
 * @file   IcpLoopComputation.cc
 * @brief  Find transform of loop closures via ICP
 * @author Yun Chang
 */
#include <Eigen/LU>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/registration/ia_ransac.h>
#include <teaser/matcher.h>
#include <teaser/registration.h>
#include <utils/CommonFunctions.h>

#include "loop_closure/PointCloudUtils.h"

#include "loop_closure/IcpLoopComputation.h"

namespace pu = parameter_utils;
namespace gu = geometry_utils;

namespace lamp_loop_closure {

IcpLoopComputation::IcpLoopComputation() {}
IcpLoopComputation::~IcpLoopComputation() {}

bool IcpLoopComputation::Initialize(const ros::NodeHandle& n) {
  std::string name = ros::names::append(n.getNamespace(), "IcpLoopComputation");
  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name.c_str());
    return false;
  }

  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name.c_str());
    return false;
  }

  return true;
}

bool IcpLoopComputation::LoadParameters(const ros::NodeHandle& n) {
  if (!LoopComputation::LoadParameters(n))
    return false;

  if (!pu::Get(param_ns_ + "/keyed_scans_max_delay", keyed_scans_max_delay_))
    return false;

  if (!pu::Get(param_ns_ + "/max_tolerable_fitness", max_tolerable_fitness_))
    return false;
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
  if (!pu::Get(param_ns_ + "/TEASERPP/num_inlier_threshold",
               teaser_inlier_threshold_))
    return false;

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

  int icp_init_method;
  if (!pu::Get(param_ns_ + "/icp_initialization_method", icp_init_method))
    return false;
  icp_init_method_ = IcpInitMethod(icp_init_method);

  int icp_covar_method;
  if (!pu::Get(param_ns_ + "/icp_covariance_calculation", icp_covar_method))
    return false;
  icp_covariance_method_ = IcpCovarianceMethod(icp_covar_method);

  SetupICP();

  // Hard coded covariances
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;
  if (!pu::Get("b_use_fixed_covariances", b_use_fixed_covariances_))
    return false;

  return true;
}

bool IcpLoopComputation::CreatePublishers(const ros::NodeHandle& n) {
  if (!LoopComputation::CreatePublishers(n))
    return false;
  return true;
}

bool IcpLoopComputation::RegisterCallbacks(const ros::NodeHandle& n) {
  if (!LoopComputation::RegisterCallbacks(n))
    return false;

  ros::NodeHandle nl(n);
  keyed_scans_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "keyed_scans", 100, &IcpLoopComputation::KeyedScanCallback, this);

  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental",
      100,
      &IcpLoopComputation::KeyedPoseCallback,
      this);

  update_timer_ =
      nl.createTimer(2.0, &IcpLoopComputation::ProcessTimerCallback, this);
  return true;
}

bool IcpLoopComputation::SetupICP() {
  icp_.setTransformationEpsilon(icp_tf_epsilon_);
  icp_.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp_.setMaximumIterations(icp_iterations_);
  icp_.setRANSACIterations(0);
  icp_.setMaximumOptimizerIterations(50);
  icp_.setNumThreads(icp_threads_);
  icp_.enableTimingOutput(true);
  return true;
}

// Compute transform and populate output queue
void IcpLoopComputation::ComputeTransforms() {
  // First make copy of input queue
  size_t n = input_queue_.size();

  // Iterate and compute transforms
  for (size_t i = 0; i < n; i++) {
    auto candidate = input_queue_.front();
    input_queue_.pop();

    // Keyed scans do not exist
    if (keyed_scans_.find(candidate.key_from) == keyed_scans_.end() ||
        keyed_scans_.find(candidate.key_to) == keyed_scans_.end()) {
      if ((ros::Time::now() - candidate.header.stamp).toSec() <
          keyed_scans_max_delay_)
        input_queue_.push(candidate);
      continue;
    }

    gtsam::Key key_from = candidate.key_from;
    gtsam::Key key_to = candidate.key_to;
    gtsam::Pose3 pose_from = utils::ToGtsam(candidate.pose_from);
    gtsam::Pose3 pose_to = utils::ToGtsam(candidate.pose_to);

    gu::Transform3 transform;
    gtsam::Matrix66 covariance;
    if (!PerformAlignment(
            key_from, key_to, pose_from, pose_to, &transform, &covariance))
      continue;

    // If aligned create PoseGraphEdge msg
    pose_graph_msgs::PoseGraphEdge loop_closure =
        CreateLoopClosureEdge(key_from, key_to, transform, covariance);
    output_queue_.push_back(loop_closure);
  }
  return;
}

void IcpLoopComputation::ProcessTimerCallback(const ros::TimerEvent& ev) {
  ComputeTransforms();

  if (loop_closure_pub_.getNumSubscribers() > 0 && output_queue_.size() > 0) {
    PublishLoopClosures();
  }
}

void IcpLoopComputation::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {
  const gtsam::Key key = scan_msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_DEBUG_STREAM("KeyedScanCallback: Key "
                     << gtsam::DefaultKeyFormatter(key)
                     << " already has a scan. Not adding.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloud::ConstPtr>(key, scan));
}

void IcpLoopComputation::KeyedPoseCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) {
  pose_graph_msgs::PoseGraphNode node_msg;
  for (const auto& node_msg : graph_msg->nodes) {
    gtsam::Key new_key = node_msg.key; // extract new key
    // Check if the node is new
    if (keyed_poses_.count(new_key) > 0) {
      continue; // Not a new node
    }

    // also extract poses (NOTE(Yun) this pose will not be updated...)
    gtsam::Pose3 new_pose;
    gtsam::Point3 pose_translation(node_msg.pose.position.x,
                                   node_msg.pose.position.y,
                                   node_msg.pose.position.z);
    gtsam::Rot3 pose_orientation(node_msg.pose.orientation.w,
                                 node_msg.pose.orientation.x,
                                 node_msg.pose.orientation.y,
                                 node_msg.pose.orientation.z);
    new_pose = gtsam::Pose3(pose_orientation, pose_translation);

    // add new key and pose to keyed_poses_
    keyed_poses_[new_key] = new_pose;
  }
}

bool IcpLoopComputation::PerformAlignment(const gtsam::Symbol& key1,
                                          const gtsam::Symbol& key2,
                                          const gtsam::Pose3& pose1,
                                          const gtsam::Pose3& pose2,
                                          gu::Transform3* delta,
                                          gtsam::Matrix66* covariance) {
  ROS_DEBUG_STREAM("Performing alignment between "
                   << gtsam::DefaultKeyFormatter(key1) << " and "
                   << gtsam::DefaultKeyFormatter(key2));

  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("PerformAlignment: Output pointers are null.");
    return false;
  }

  // Check for available information
  if (!keyed_scans_.count(key1) || !keyed_scans_.count(key2)) {
    ROS_WARN(
        "PerformAlignment: Missing keyed-scans when performing alignment. ");
    return false;
  }

  if (!keyed_poses_.count(key1) || !keyed_poses_.count(key2)) {
    ROS_WARN(
        "PerformAlignment: Missing keyed-poses when performing alignment. ");
    return false;
  }

  // Get poses and keys
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
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
    initial_guess.block(0, 3, 3, 1) = pose_21.translation().cast<float>();
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
    GetSacInitialAlignment(scan1, scan2, &initial_guess, sac_fitness_score);
    if (sac_fitness_score >= sac_fitness_score_threshold_) {
      ROS_INFO("SAC fitness score is too high");
      return false;
    }
  } break;
  case IcpInitMethod::TEASERPP: {
    int n_inliers = teaser_inlier_threshold_;
    GetTeaserInitialAlignment(
        scan1, accumulated_target, &initial_guess, n_inliers);
    if (n_inliers <= teaser_inlier_threshold_) {
      ROS_INFO("Number of TEASER inliers is too low: %d <= %d",
               n_inliers,
               teaser_inlier_threshold_);
      return false;
    }
  } break;
  default: // identity as default (default in ICP anyways)
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  }
  }

  // Perform ICP_.
  PointCloud::Ptr icp_result(new PointCloud);
  icp_.align(*icp_result, initial_guess);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp_.getFinalTransformation();

  // Get the correspondence indices
  std::vector<size_t> correspondences;
  if (icp_covariance_method_ == IcpCovarianceMethod::POINT2PLANE) {
    KdTree::Ptr search_tree = icp_.getSearchMethodTarget();
    for (auto point : icp_result->points) {
      // Catch nan of infs in icp result
      if (!pcl::isFinite(point))
        return false;

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
    ROS_INFO_STREAM("ICP: Not converged, score is: " << icp_.getFitnessScore());
    return false;
  }

  if (icp_.getFitnessScore() > max_tolerable_fitness_) {
    ROS_INFO_STREAM("ICP: Coverged but score is: " << icp_.getFitnessScore());
    return false;
  }

  double fitness_score = icp_.getFitnessScore();

  // Find transform from pose2 to pose1 from output of ICP_.
  *delta = gu::PoseInverse(*delta); // NOTE: gtsam need 2_Transform_1 while
                                    // ICP output 1_Transform_2

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
          scan1, scan2, correspondences, T, covariance);
      break;
    default:
      ROS_ERROR(
          "Unknown method for ICP covariance calculation for loop closures. "
          "Check config.");
    }
  }

  return true;
}

void IcpLoopComputation::GetSacInitialAlignment(PointCloud::ConstPtr source,
                                                PointCloud::ConstPtr target,
                                                Eigen::Matrix4f* tf_out,
                                                double& sac_fitness_score) {
  // Get Normals
  Normals::Ptr source_normals(new Normals);
  Normals::Ptr target_normals(new Normals);
  utils::ComputeNormals(
      source, sac_normals_radius_, icp_threads_, source_normals);
  utils::ComputeNormals(
      target, sac_normals_radius_, icp_threads_, target_normals);

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
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZI,
                                       pcl::PointXYZI,
                                       pcl::FPFHSignature33>
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
}

bool IcpLoopComputation::ComputeICPCovariancePointPlane(
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

  utils::ComputeNormals(
      reference_cloud, sac_normals_radius_, icp_threads_, reference_normals);
  utils::NormalizePCloud(query_cloud, query_normalized);

  utils::ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_normals, correspondences, T, Ap);
  // If matrix not invertible, use fixed
  if (covariance->determinant() == 0) {
    for (int i = 0; i < 3; ++i)
      (*covariance)(i, i) = laser_lc_rot_sigma_ * laser_lc_rot_sigma_;
    for (int i = 3; i < 6; ++i)
      (*covariance)(i, i) = laser_lc_trans_sigma_ * laser_lc_trans_sigma_;
    return true;
  }

  *covariance = 0.01 * 0.01 * Ap.inverse();

  // Here bound the covariance using eigen values
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(*covariance);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  double lower_bound = 1e-6; // Should be positive semidef
  double upper_bound = 1e6;
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

  if (covariance->array().hasNaN()) { // Prevent NaNs in covariance
    *covariance = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i)
      (*covariance)(i, i) = laser_lc_rot_sigma_ * laser_lc_rot_sigma_;
    for (int i = 3; i < 6; ++i)
      (*covariance)(i, i) = laser_lc_trans_sigma_ * laser_lc_trans_sigma_;
  }

  return true;
}

bool IcpLoopComputation::ComputeICPCovariancePointPoint(
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

void IcpLoopComputation::AccumulateScans(const gtsam::Key& key,
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
}

void IcpLoopComputation::GetTeaserInitialAlignment(PointCloud::ConstPtr source,
                                                   PointCloud::ConstPtr target,
                                                   Eigen::Matrix4f* tf_out,
                                                   int& n_inliers) {
  // Convert to teaser point cloud
  teaser::PointCloud src_cloud;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it =
           source->points.begin();
       it != source->points.end();
       it++) {
    src_cloud.push_back({it->x, it->y, it->z});
  }

  teaser::PointCloud tgt_cloud;
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator it =
           target->points.begin();
       it != target->points.end();
       it++) {
    tgt_cloud.push_back({it->x, it->y, it->z});
  }

  // Compute FPFH
  teaser::FPFHEstimation fpfh;
  auto src_descriptors = fpfh.computeFPFHFeatures(src_cloud, 1.5, 2.5);
  auto target_descriptors = fpfh.computeFPFHFeatures(tgt_cloud, 1.5, 2.5);

  // Align
  ROS_DEBUG("Finding TEASER Correspondences!");
  teaser::Matcher matcher;
  auto correspondences = matcher.calculateCorrespondences(src_cloud,
                                                          tgt_cloud,
                                                          *src_descriptors,
                                                          *target_descriptors,
                                                          false,
                                                          true,
                                                          false,
                                                          0.95);
  int corres_size = correspondences.size();
  ROS_DEBUG("Found %d correspondences.", corres_size);

  // Retrive the corresponding points from src and tgt point clouds into two
  // 3-by-N Eigen matrices
  Eigen::Matrix<double, 3, Eigen::Dynamic> src_corres_points(3, corres_size);
  Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_corres_points(3, corres_size);
  for (size_t i = 0; i < corres_size; ++i) {
    src_corres_points.col(i) << src_cloud[correspondences[i].first].x,
        src_cloud[correspondences[i].first].y,
        src_cloud[correspondences[i].first].z;
    tgt_corres_points.col(i) << tgt_cloud[correspondences[i].second].x,
        tgt_cloud[correspondences[i].second].y,
        tgt_cloud[correspondences[i].second].z;
  }
  ROS_DEBUG("Completed TEASER Correspondences!");
  // Run TEASER++ registration
  // Prepare solver parameters
  teaser::RobustRegistrationSolver::Params params;
  params.noise_bound = 0.05;
  params.cbar2 = 1;
  params.estimate_scaling = false;
  params.rotation_max_iterations = 100;
  params.rotation_gnc_factor = 1.4;
  ROS_INFO("Finding TEASER Rigid Transform...");
  params.rotation_estimation_algorithm =
      teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
  params.rotation_cost_threshold = 1.0; // 0.005;

  // Solve with TEASER++
  teaser::RobustRegistrationSolver solver(params);
  solver.solve(src_corres_points, tgt_corres_points);
  ROS_INFO("");
  auto solution = solver.getSolution();
  Eigen::Matrix4d T;
  T.topLeftCorner(3, 3) = solution.rotation;
  T.topRightCorner(3, 1) = solution.translation;
  *tf_out = T.cast<float>();

  auto final_inliers = solver.getInlierMaxClique();
  n_inliers = static_cast<int>(final_inliers.size());
  ROS_INFO("Solved TEASER Rigid Transform with %d inliers", n_inliers);
}

} // namespace lamp_loop_closure