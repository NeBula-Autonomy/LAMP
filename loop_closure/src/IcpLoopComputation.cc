/**
 * @file   IcpLoopComputation.cc
 * @brief  Find transform of loop closures via ICP
 * @author Yun Chang
 */
#include <Eigen/LU>
#include <cmath>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/registration/ia_ransac.h>
#include <teaser/matcher.h>
#include <teaser/evaluation.h>
#include <teaser/registration.h>
#include <lamp_utils/CommonFunctions.h>

#include "lamp_utils/PointCloudUtils.h"

#include "loop_closure/IcpLoopComputation.h"

namespace pu = parameter_utils;
namespace gu = geometry_utils;

namespace lamp_loop_closure {

IcpLoopComputation::IcpLoopComputation()
  : icp_computation_pool_(0), b_accumulate_source_(false) {}
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
  if (number_of_threads_in_icp_computation_pool_ > 1) {
      ROS_INFO_STREAM("Thread Pool Initialized with " << number_of_threads_in_icp_computation_pool_ << " threads");
      icp_computation_pool_.resize(number_of_threads_in_icp_computation_pool_);
  }
  else{
      ROS_INFO_STREAM("Not initializing thread pool");
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
  if (!pu::Get(param_ns_ + "/icp_lc/transform_thresholding",
               icp_transform_thresholding_))
    return false;
  if (!pu::Get(param_ns_ + "/icp_lc/max_translation", icp_max_translation_))
    return false;
  if (!pu::Get(param_ns_ + "/icp_lc/max_rotation", icp_max_rotation_))
    return false;

  // Load SAC parameters
  if (!pu::Get(param_ns_ + "/sac_ia/iterations", sac_iterations_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_prev_scans", sac_num_prev_scans_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/num_next_scans", sac_num_next_scans_))
    return false;
  if (!pu::Get(param_ns_ + "/sac_ia/b_accumulate_source", b_accumulate_source_))
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
  if (!pu::Get(param_ns_ + "/TEASERPP/rotation_cost_threshold",
               rotation_cost_threshold_))
    return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/rotation_max_iterations",
               rotation_max_iterations_))
    return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/noise_bound", noise_bound_))
    return false;
  if (!pu::Get(param_ns_ + "/TEASERPP/TEASER_FPFH_features_radius",
               TEASER_FPFH_features_radius_))
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

  SetupICP(icp_);

  // Hard coded covariances
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_))
    return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_))
    return false;
  if (!pu::Get("b_use_fixed_covariances", b_use_fixed_covariances_))
    return false;

  double icp_computation_thread_pool_size;
  if (!pu::Get(param_ns_ + "/icp_thread_pool_thread_count", icp_computation_thread_pool_size))
        return false;
  if (icp_computation_thread_pool_size >= 1.0){
      number_of_threads_in_icp_computation_pool_ = (size_t) icp_computation_thread_pool_size;
  } else {
      double processor_count = std::thread::hardware_concurrency();
      number_of_threads_in_icp_computation_pool_ = (size_t) (icp_computation_thread_pool_size * processor_count);
  }
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
      "keyed_scans", 100000, &IcpLoopComputation::KeyedScanCallback, this);

  keyed_poses_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "pose_graph_incremental",
      100000,
      &IcpLoopComputation::KeyedPoseCallback,
      this);

  update_timer_ =
      nl.createTimer(1.0, &IcpLoopComputation::ProcessTimerCallback, this);
  return true;
}

bool IcpLoopComputation::SetupICP(pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point>& icp) {
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);
  icp.setMaximumOptimizerIterations(50);
  icp.setNumThreads(icp_threads_);
  icp.enableTimingOutput(true);
  return true;
}

// Compute transform and populate output queue
void IcpLoopComputation::ComputeTransforms() {
  // First make copy of input queue
  size_t n = input_queue_.size();

  if (number_of_threads_in_icp_computation_pool_ == 1){
      //If we have decided to not use the thread pool
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
              if (keyed_scans_.find(candidate.key_from) == keyed_scans_.end()){
                  ROS_INFO_STREAM("Missing Candidate for " << candidate.key_from);
              }

              if (keyed_scans_.find(candidate.key_to) == keyed_scans_.end()) {
                  ROS_INFO_STREAM("Missing Candidate for " << candidate.key_to);
              }
              continue;

          }

          gtsam::Key key_from = candidate.key_from;
          gtsam::Key key_to = candidate.key_to;
          gtsam::Pose3 pose_from = lamp_utils::ToGtsam(candidate.pose_from);
          gtsam::Pose3 pose_to = lamp_utils::ToGtsam(candidate.pose_to);

          gu::Transform3 transform;
          gtsam::Matrix66 covariance;
          double icp_fitness;
          if (!PerformAlignment(key_from,
                                key_to,
                                pose_from,
                                pose_to,
                                &transform,
                                &covariance,
                                &icp_fitness,
                                false))
            continue;

          // If aligned create PoseGraphEdge msg
          pose_graph_msgs::PoseGraphEdge loop_closure =
                  CreateLoopClosureEdge(key_from, key_to, transform, covariance);
          loop_closure.range_error = icp_fitness;
          output_queue_.push_back(loop_closure);
      }
  } else {
    ROS_DEBUG_STREAM("Threaded, Queue Size " << n);
    std::vector<std::future<std::pair<bool, pose_graph_msgs::PoseGraphEdge>>>
        futures;
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
        if (keyed_scans_.find(candidate.key_from) == keyed_scans_.end()) {
          ROS_INFO_STREAM("Missing Candidate for " << candidate.key_from);
        }

        if (keyed_scans_.find(candidate.key_to) == keyed_scans_.end()) {
          ROS_INFO_STREAM("Missing Candidate for " << candidate.key_to);
        }
        continue;
      }

      // pose_graph_msgs::LoopCandidate* candidate_ptr = new
      // pose_graph_msgs::LoopCandidate(candidate);
      futures.emplace_back(icp_computation_pool_.enqueue([&, candidate]() {
        gtsam::Key key_from = candidate.key_from;
        gtsam::Key key_to = candidate.key_to;
        gtsam::Pose3 pose_from = lamp_utils::ToGtsam(candidate.pose_from);
        gtsam::Pose3 pose_to = lamp_utils::ToGtsam(candidate.pose_to);

        gu::Transform3 transform;
        gtsam::Matrix66 covariance;
        double icp_fitness;
        if (!PerformAlignment(key_from,
                              key_to,
                              pose_from,
                              pose_to,
                              &transform,
                              &covariance,
                              &icp_fitness,
                              true))
          return std::make_pair(false, pose_graph_msgs::PoseGraphEdge());
        // If aligned create PoseGraphEdge msg
        pose_graph_msgs::PoseGraphEdge loop_closure =
            CreateLoopClosureEdge(key_from, key_to, transform, covariance);
        loop_closure.range_error = icp_fitness;
        return std::make_pair(true, loop_closure);
      }));
      }
      for (auto &future : futures) {
          future.wait();
          auto result = future.get();
          bool alignment_was_successful = result.first;
          if (alignment_was_successful) {
              output_queue_.push_back(result.second);
          }
      }
  }
}

void IcpLoopComputation::ProcessTimerCallback(const ros::TimerEvent& ev) {
  ComputeTransforms();

  if (loop_closure_pub_.getNumSubscribers() > 0) {
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

  pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
  pcl::fromROSMsg(scan_msg->scan, *scan);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Key, PointCloudConstPtr>(key, scan));
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
                                          gtsam::Matrix66* covariance,
                                          double* fitness_score,
                                          bool re_initialize_icp) {
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
  const PointCloudConstPtr scan1 = keyed_scans_.at(key1.key());
  const PointCloudConstPtr scan2 = keyed_scans_.at(key2.key());

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

  PointCloud::Ptr accumulated_source(new PointCloud);
  *accumulated_source = *scan1;

  if (b_accumulate_source_) {
    AccumulateScans(key1, accumulated_source);
  }

  pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point>* icp;
  if (re_initialize_icp){
    icp = new  pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point>();
    SetupICP(*icp);
  } else {
      icp = &icp_;
  }
  icp->setInputSource(accumulated_source);
  icp->setInputTarget(accumulated_target);
  if (accumulated_source->size() < 20) {
    icp->setCorrespondenceRandomness(accumulated_source->size());
  }

  ///// ICP initialization scheme
  // Default is to initialize by identity. Other options include
  // initializing with odom measurement
  // or initialize with 0 translation byt rotation from odom
  Eigen::Matrix4f initial_guess;
  gtsam::Pose3 pose_21 = keyed_poses_[key2].between(keyed_poses_[key1]);
  initial_guess = Eigen::Matrix4f::Identity(4, 4);
  initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
  initial_guess.block(0, 3, 3, 1) = pose_21.translation().cast<float>();

  switch (icp_init_method_) {
  case IcpInitMethod::IDENTITY: // initialize with idientity
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  } break;

  case IcpInitMethod::ODOMETRY: // initialize with odometry
  {
  } break;

  case IcpInitMethod::ODOM_ROTATION: // initialize with zero translation but
                                     // rot from odom
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
    initial_guess.block(0, 0, 3, 3) = pose_21.rotation().matrix().cast<float>();
  } break;
  case IcpInitMethod::FEATURES: {
    double sac_fitness_score = sac_fitness_score_threshold_;
    GetSacInitialAlignment(accumulated_source,
                           accumulated_target,
                           &initial_guess,
                           sac_fitness_score);
    if (sac_fitness_score >= sac_fitness_score_threshold_) {
      ROS_DEBUG("SAC fitness score is too high");
      return false;
    }
  } break;
  case IcpInitMethod::TEASERPP: {
    GetTeaserInitialAlignment(
        accumulated_source, accumulated_target, &initial_guess);
  } break;
  case IcpInitMethod::CANDIDATE: {
    gtsam::Pose3 candidate_pose21 = pose2.between(pose1);
    initial_guess.block(0, 0, 3, 3) =
        candidate_pose21.rotation().matrix().cast<float>();
    initial_guess.block(0, 3, 3, 1) =
        candidate_pose21.translation().cast<float>();
  } break;
  default: // identity as default (default in ICP anyways)
  {
    initial_guess = Eigen::Matrix4f::Identity(4, 4);
  }
  }

  // Perform ICP_.
  PointCloud::Ptr icp_result(new PointCloud);
  icp->align(*icp_result, initial_guess);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp->getFinalTransformation();

  // Get the correspondence indices
  std::vector<size_t> correspondences;
  if (icp_covariance_method_ == IcpCovarianceMethod::POINT2PLANE) {
    KdTree::Ptr search_tree = icp->getSearchMethodTarget();
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
  if (!icp->hasConverged()) {
    ROS_DEBUG_STREAM(
        "ICP: Not converged, score is: " << icp->getFitnessScore());
    return false;
  }

  *fitness_score = icp->getFitnessScore();

  if (*fitness_score > max_tolerable_fitness_) {
    ROS_INFO_STREAM("ICP: Converged or max iterations reached, but score: "
                    << icp->getFitnessScore()
                    << ", Exceeds threshold: " << max_tolerable_fitness_);
    return false;
  }

  // Check if the rotation exceeds thresholds
  // Get difference between odom and icp estimation
  gtsam::Pose3 diff = (keyed_poses_[key2].between(keyed_poses_[key1]))
                          .between(lamp_utils::ToGtsam(*delta));
  gtsam::Vector diff_log = gtsam::Pose3::Logmap(diff);
  double trans_diff =
      std::sqrt(diff_log.tail(3).transpose() * diff_log.tail(3));
  double rot_diff = std::sqrt(diff_log.head(3).transpose() * diff_log.head(3));
  // Thresholding
  if (icp_transform_thresholding_ &&
      (trans_diff > icp_max_translation_ ||
       rot_diff > icp_max_rotation_ * M_PI / 180.0)) {
    ROS_DEBUG_STREAM(
        "ICP: Convered and passes fitness, but translation or rotation exceeds "
        "threshold.\n\tTranslation: "
        << trans_diff << ", thresh: " << icp_max_translation_
        << "\n\tRotation (deg): " << rot_diff * 180.0 / M_PI << ", "
        << ", thresh: " << icp_max_rotation_);
    return false;
  }

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
      ComputeICPCovariancePointPoint(
          icp_result, T, *fitness_score, *covariance);
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

  ROS_INFO_STREAM("Successfully completed alignment between "
                  << gtsam::DefaultKeyFormatter(key1) << " and "
                  << gtsam::DefaultKeyFormatter(key2)
                  << " with fitness score: " << *fitness_score);

  return true;
}

void IcpLoopComputation::GetSacInitialAlignment(PointCloudConstPtr source,
                                                PointCloudConstPtr target,
                                                Eigen::Matrix4f* tf_out,
                                                double& sac_fitness_score) {
  // Get Normals
  Normals::Ptr source_normals(new Normals);
  Normals::Ptr target_normals(new Normals);
  lamp_utils::ExtractNormals(source, source_normals);
  lamp_utils::ExtractNormals(target, target_normals);

  // Get Harris keypoints for source and target
  PointCloud::Ptr source_keypoints(new PointCloud);
  PointCloud::Ptr target_keypoints(new PointCloud);

  lamp_utils::ComputeKeypoints(
      source, source_normals, harris_params_, icp_threads_, source_keypoints);
  lamp_utils::ComputeKeypoints(
      target, target_normals, harris_params_, icp_threads_, target_keypoints);

  Features::Ptr source_features(new Features);
  Features::Ptr target_features(new Features);
  lamp_utils::ComputeFeatures(source_keypoints,
                         source,
                         source_normals,
                         sac_features_radius_,
                         icp_threads_,
                         source_features);
  lamp_utils::ComputeFeatures(target_keypoints,
                         target,
                         target_normals,
                         sac_features_radius_,
                         icp_threads_,
                         target_features);

  // std::cout << "loop - SAC  src cloud size: " << source_keypoints->size() <<
  // std::endl; std::cout << "loop - SAC target cloud size: " <<
  // target_keypoints->size() << std::endl; std::cout << "loop - SAC src
  // features size: " << source_features->size() << std::endl; std::cout <<
  // "loop - SAC target features size: " << target_features->size() <<
  // std::endl; Align
  pcl::SampleConsensusInitialAlignment<Point, Point, pcl::FPFHSignature33>
      sac_ia;
  sac_ia.setMaximumIterations(sac_iterations_);
  sac_ia.setInputSource(source_keypoints);
  sac_ia.setSourceFeatures(source_features);
  sac_ia.setInputTarget(target_keypoints);
  sac_ia.setTargetFeatures(target_features);
  sac_ia.setCorrespondenceRandomness(5);
  PointCloud::Ptr aligned_output(new PointCloud);
  sac_ia.align(*aligned_output, *tf_out);

  sac_fitness_score = sac_ia.getFitnessScore();
  ROS_DEBUG_STREAM("SAC fitness score: " << sac_fitness_score);

  *tf_out = sac_ia.getFinalTransformation();
}

bool IcpLoopComputation::ComputeICPCovariancePointPlane(
    const PointCloudConstPtr& query_cloud,
    const PointCloudConstPtr& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* covariance) {
  // Get normals
  Normals::Ptr reference_normals(new Normals);      // pc with normals
  PointCloud::Ptr query_normalized(new PointCloud); // pc whose points have
                                                    // been rearranged.
  Eigen::Matrix<double, 6, 6> Ap;

  lamp_utils::ExtractNormals(reference_cloud, reference_normals);
  lamp_utils::NormalizePCloud(query_cloud, query_normalized);

  lamp_utils::ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_normals, correspondences, T, Ap);
  // If matrix not invertible, use fixed
  if (Ap.determinant() == 0) {
    for (int i = 0; i < 3; ++i)
      (*covariance)(i, i) = laser_lc_rot_sigma_ * laser_lc_rot_sigma_;
    for (int i = 3; i < 6; ++i)
      (*covariance)(i, i) = laser_lc_trans_sigma_ * laser_lc_trans_sigma_;
    return true;
  } else {
    *covariance = 0.05 * 0.05 * Ap.inverse();
  }

  // Here bound the covariance using eigen values
  //// First find ldlt decomposition
  auto ldlt = covariance->ldlt();
  Eigen::MatrixXd L = ldlt.matrixL();
  Eigen::VectorXd vecD = ldlt.vectorD();

  double lower_bound = 1e-12;
  double upper_bound = 1;

  bool recompute = false;
  for (size_t i = 0; i < vecD.size(); i++) {
    if (vecD(i) <= 0) {
      vecD(i) = lower_bound;
      recompute = true;
    }
    if (vecD(i) > upper_bound) {
      vecD(i) = upper_bound;
      recompute = true;
    }
  }

  if (recompute)
    *covariance = L * vecD.asDiagonal() * L.transpose();

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
    const PointCloudConstPtr& pointCloud,
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
    const PointCloudConstPtr prev_scan = keyed_scans_[prev_key];

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
    const PointCloudConstPtr next_scan = keyed_scans_[next_key];

    // Transform and Accumulate
    const gtsam::Pose3 new_pose = keyed_poses_.at(key);
    const gtsam::Pose3 old_pose = keyed_poses_.at(next_key);
    const gtsam::Pose3 tf = new_pose.between(old_pose);

    PointCloud::Ptr transformed(new PointCloud);
    pcl::transformPointCloud(*next_scan, *transformed, tf.matrix());
    *scan_out += *transformed;
  }
}

void IcpLoopComputation::GetTeaserInitialAlignment(PointCloudConstPtr source,
                                                   PointCloudConstPtr target,
                                                   Eigen::Matrix4f* tf_out) {
  // Get Normals
  Normals::Ptr source_normals(new Normals);
  Normals::Ptr target_normals(new Normals);
  lamp_utils::ExtractNormals(source, source_normals);
  lamp_utils::ExtractNormals(target, target_normals);

  // Get Harris keypoints for source and target
  PointCloud::Ptr source_keypoints(new PointCloud);
  PointCloud::Ptr target_keypoints(new PointCloud);

  lamp_utils::ComputeKeypoints(
      source, source_normals, harris_params_, icp_threads_, source_keypoints);
  lamp_utils::ComputeKeypoints(
      target, target_normals, harris_params_, icp_threads_, target_keypoints);

  Features::Ptr source_features(new Features);
  Features::Ptr target_features(new Features);
  lamp_utils::ComputeFeatures(source_keypoints,
                         source,
                         source_normals,
                         sac_features_radius_,
                         icp_threads_,
                         source_features);
  lamp_utils::ComputeFeatures(target_keypoints,
                         target,
                         target_normals,
                         sac_features_radius_,
                         icp_threads_,
                         target_features);

  // std::cout << "loop - src cloud size: " << source_keypoints->size() <<
  // std::endl; std::cout << "loop - target cloud size: " <<
  // target_keypoints->size() << std::endl; std::cout << "loop - src features
  // size: " << source_features->size() << std::endl; std::cout << "loop -
  // target features size: " << target_features->size() << std::endl; std::cout
  // << "Number of inlier threshold is: " << teaser_inlier_threshold_ <<
  // std::endl;

  if (source_keypoints->size() == 0 || target_keypoints->size() == 0) {
    return;
  }

  // Align
  ROS_DEBUG("Finding TEASER Correspondences!");
  teaser::Matcher matcher;
  auto correspondences = matcher.calculateKCorrespondences(
      source_keypoints, target_keypoints, source_features, target_features, 5);
  int corres_size = correspondences.size();

  // ROS_DEBUG("Found %d correspondences.", corres_size);
  if (corres_size > 10) {
    // Retrive the corresponding points from src and tgt point clouds into two
    // 3-by-N Eigen matrices
    Eigen::Matrix<double, 3, Eigen::Dynamic> src_corres_points(3, corres_size);
    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_corres_points(3, corres_size);
    for (size_t i = 0; i < corres_size; ++i) {
      src_corres_points.col(i)
          << (*source_keypoints)[correspondences[i].first].x,
          (*source_keypoints)[correspondences[i].first].y,
          (*source_keypoints)[correspondences[i].first].z;
      tgt_corres_points.col(i)
          << (*target_keypoints)[correspondences[i].second].x,
          (*target_keypoints)[correspondences[i].second].y,
          (*target_keypoints)[correspondences[i].second].z;
    }

    ROS_DEBUG("Completed TEASER Correspondences!");
    // Run TEASER++ registration
    // Prepare solver parameters
    teaser::RobustRegistrationSolver::Params params;
    params.noise_bound = 0.5;
    params.cbar2 = 1;
    params.estimate_scaling = false;
    params.rotation_max_iterations = 100;
    params.rotation_gnc_factor = 1.4;
    ROS_INFO("Finding TEASER Rigid Transform...");
    params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::
        ROTATION_ESTIMATION_ALGORITHM::GNC_TLS;
    params.rotation_cost_threshold = 1e-6;
    params.inlier_selection_mode =
        teaser::RobustRegistrationSolver::INLIER_SELECTION_MODE::PMC_HEU;
    // Solve with TEASER++
    teaser::RobustRegistrationSolver solver(params);
    solver.solve(src_corres_points, tgt_corres_points);
    ROS_INFO("");
    auto solution = solver.getSolution();
    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = solution.rotation;
    T.topRightCorner(3, 1) = solution.translation;
    T(3, 3) = 1.0;

    Eigen::Matrix<double, 3, 3> odom_rotation;
    odom_rotation = tf_out->block(0, 0, 3, 3).cast<double>();
    Eigen::Matrix<double, 3, 1> odom_translation;
    odom_translation = tf_out->block(0, 3, 3, 1).cast<double>();
    double corr_dist_threshold = 100;
    teaser::SolutionEvaluator evaluator(
        src_corres_points, tgt_corres_points, corr_dist_threshold);

    auto error_teaser =
        evaluator.computeErrorMetric(solution.rotation, solution.translation);
    auto error_odom =
        evaluator.computeErrorMetric(odom_rotation, odom_translation);
    // std::cout << "Odom Rotation: " << odom_rotation << std::endl;
    // std::cout << "Odom Translation: " << odom_translation << std::endl;
    // std::cout << "error_teaser: " << error_teaser << std::endl;
    // std::cout << "error_odom: " << error_odom << std::endl;
    // std::cout << "Estimated T is: " << T << std::endl;
    // *tf_out = T.cast<float>();
    // auto final_inliers = solver.getInlierMaxClique();
    // n_inliers = static_cast<int>(final_inliers.size());
    // ROS_INFO("Solved TEASER Rigid Transform with %d inliers", n_inliers);

    if (error_teaser <= error_odom) {
      // std::cout << "Estimated T is: " << T << std::endl;
      *tf_out = T.cast<float>();
      teaser_count_++;
      // auto final_inliers = solver.getInlierMaxClique();
      // n_inliers = static_cast<int>(final_inliers.size());
      // ROS_INFO("Solved TEASER Rigid Transform with %d inliers", n_inliers);
    } else {
      odom_count_++;
    }
    // std::cout << "teaser_count: " << teaser_count_ << std::endl;
    // std::cout << "odom_count: " << odom_count_ << std::endl;

  } else {
    ROS_INFO("Number of corresponding points too low %d: ", corres_size);
  }
}

} // namespace lamp_loop_closure