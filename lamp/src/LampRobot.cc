/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 * Kamak Ebadi          ()
 * Matteo               ()
 * Nobuhrio
 * Yun
 * Abhishek
 * Eric Hieden
 */

// Includes
#include <lamp/LampRobot.h>
#include <utils/PointCloudUtils.h>

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
using gtsam::Symbol;
using gtsam::Values;
using gtsam::Vector3;

// Constructor
LampRobot::LampRobot()
  : is_artifact_initialized(false), b_init_pg_pub_(false), init_count_(0) {
  b_run_optimization_ = false;
  mapper_ = std::make_shared<PointCloudMapper>();
}

// Destructor
LampRobot::~LampRobot() {}

// Initialization - override for robot specific setup
bool LampRobot::Initialize(const ros::NodeHandle& n) {
  // Get the name of the process
  name_ = ros::names::append(n.getNamespace(), "LampRobot");

  if (!mapper_->Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  //  // Init Handlers
  if (!InitializeHandlers(n)) {
    ROS_ERROR("%s: Failed to initialize handlers.", name_.c_str());
    return false;
  }

  //  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }

  return true;
}

bool LampRobot::LoadParameters(const ros::NodeHandle& n) {
  // Rates
  if (!pu::Get("rate/update_rate", update_rate_))
    return false;
  if (!pu::Get("init_wait_time", init_wait_time_))
    return false;
  if (!pu::Get("repub_first_wait_time", repub_first_wait_time_))
    return false;

  // Settings for precisions
  if (!pu::Get("b_use_fixed_covariances", b_use_fixed_covariances_))
    return false;

  // Switch on/off flag for IMU
  if (!pu::Get("b_add_imu_factors", b_add_imu_factors_))
    return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", pose_graph_.fixed_frame_id))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;

  if (!pu::Get("b_artifacts_in_global", b_artifacts_in_global_))
    return false;

  if (!pu::Get("time_threshold", pose_graph_.time_threshold))
    return false;
  // Load filtering parameters.
  if (!pu::Get("filtering/adaptive_grid_filter",
               filter_params_.adaptive_grid_filter))
    return false;
  if (!pu::Get("filtering/adaptive_grid_target",
               filter_params_.adaptive_grid_target))
    return false;
  if (!pu::Get("filtering/adaptive_max_grid", filter_params_.adaptive_max_grid))
    return false;
  if (!pu::Get("filtering/adaptive_min_grid", filter_params_.adaptive_min_grid))
    return false;
  if (!pu::Get("filtering/observability_check",
               filter_params_.observability_check))
    return false;
  if (!pu::Get("filtering/random_filter", filter_params_.random_filter))
    return false;
  if (!pu::Get("filtering/decimate_percentage",
               filter_params_.decimate_percentage))
    return false;

  // Cap to [0.0, 1.0].
  filter_params_.decimate_percentage =
      std::min(1.0, std::max(0.0, filter_params_.decimate_percentage));

  // Initialize Filter
  filter_ = LampPcldFilter(filter_params_);

  // Set Precisions
  // TODO - eventually remove the need to use this
  if (!SetFactorPrecisions()) {
    ROS_ERROR("SetFactorPrecisions failed");
    return false;
  }

  // Set the initial key - to get the right symbol
  if (!SetInitialKey()) {
    ROS_ERROR("SetInitialKey failed");
    return false;
  }

  // Wait for the first clock message to be received
  ros::Rate r(2);
  while (ros::Time::now().toSec() == 0.0) {
    ROS_INFO_ONCE("Waiting for clock...");
    ROS_DEBUG("Waiting for clock...");
    r.sleep();
  }
  ROS_INFO("Have clock");

  // Timestamp to keys initialization (initilization is particular to the robot
  // version of lamp)
  ros::Time stamp = ros::Time::now();
  pose_graph_.InsertKeyedStamp(pose_graph_.initial_key, stamp);
  pose_graph_.InsertStampedOdomKey(stamp.toSec(), pose_graph_.initial_key);

  // Set initial key
  pose_graph_.key = pose_graph_.initial_key + 1;

  // Set the initial position (from fiducials) - also inits the pose-graph
  if (!SetInitialPosition()) {
    ROS_ERROR("SetInitialPosition failed");
    return false;
  }

  b_has_new_factor_ = false;

  return true;
}

bool LampRobot::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  update_timer_ =
      nl.createTimer(update_rate_, &LampRobot::ProcessTimerCallback, this);

  back_end_pose_graph_sub_ = nl.subscribe("optimized_values",
                                          1,
                                          &LampRobot::OptimizerUpdateCallback,
                                          dynamic_cast<LampBase*>(this));

  laser_loop_closure_sub_ = nl.subscribe("laser_loop_closures",
                                         1,
                                         &LampRobot::LaserLoopClosureCallback,
                                         dynamic_cast<LampBase*>(this));

  return true;
}

bool LampRobot::CreatePublishers(const ros::NodeHandle& n) {
  // Creates pose graph publishers in base class
  LampBase::CreatePublishers(n);

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Pose Graph publishers
  pose_graph_to_optimize_pub_ = nl.advertise<pose_graph_msgs::PoseGraph>(
      "pose_graph_to_optimize", 10, true);
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, true);

  // Publishers
  pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>("lamp_pose", 10, false);

  return true;
}

bool LampRobot::SetInitialKey() {
  // Get the robot prefix from launchfile to set initial key
  // TODO - get this convertor setup to Kyon
  unsigned char prefix_converter[1];

  if (!pu::Get("robot_prefix", pose_graph_.prefix)) {
    ROS_ERROR(
        "Could not find node ID assosiated with robot_namespace [LampRobot]");
    pose_graph_.initial_key = 0;
    return false;
  } else {
    std::copy(
        pose_graph_.prefix.begin(), pose_graph_.prefix.end(), prefix_converter);
    pose_graph_.initial_key = gtsam::Symbol(prefix_converter[0], 0);
    return true;
  }
}

bool LampRobot::SetInitialPosition() {
  // Load initial position and orientation.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
  // if (pose_graph_.prefix.at(0) != 'a') { // offset for debugging
  //   init_x = 0.0, init_y = 0.0, init_z = 0.0;
  //   init_qx = 0.0, init_qy = 0.0, init_qz = sqrt(0.01), init_qw = sqrt(0.99);
  // }
  bool b_have_fiducial = true;
  if (!pu::Get("fiducial_calibration/position/x", init_x))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/y", init_y))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/z", init_z))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/x", init_qx))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/y", init_qy))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/z", init_qz))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/w", init_qw))
    b_have_fiducial = false;

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  }

  // Load initial position and orientation noise.
  double sigma_x = 0.0, sigma_y = 0.0, sigma_z = 0.0;
  double sigma_roll = 0.0, sigma_pitch = 0.0, sigma_yaw = 0.0;
  if (!pu::Get("init/position_sigma/x", sigma_x))
    return false;
  if (!pu::Get("init/position_sigma/y", sigma_y))
    return false;
  if (!pu::Get("init/position_sigma/z", sigma_z))
    return false;
  if (!pu::Get("init/orientation_sigma/roll", sigma_roll))
    return false;
  if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch))
    return false;
  if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw))
    return false;

  // convert initial quaternion to Roll/Pitch/Yaw
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  gu::Quat q(gu::Quat(init_qw, init_qx, init_qy, init_qz));
  gu::Rot3 m1;
  m1 = gu::QuatToR(q);
  init_roll = m1.Roll();
  init_pitch = m1.Pitch();
  init_yaw = m1.Yaw();

  // Set the initial position.
  Vector3 translation(init_x, init_y, init_z);
  Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
  Pose3 pose(rotation, translation);

  // Set the covariance on initial position.
  initial_noise_ << sigma_roll, sigma_pitch, sigma_yaw, sigma_x, sigma_y,
      sigma_z;

  gtsam::noiseModel::Diagonal::shared_ptr covariance(
      gtsam::noiseModel::Diagonal::Sigmas(initial_noise_));
  ROS_DEBUG_STREAM("covariance is");
  ROS_DEBUG_STREAM(initial_noise_);

  // Initialize graph  with the initial position
  InitializeGraph(pose, covariance);

  return true;
}

bool LampRobot::InitializeGraph(
    gtsam::Pose3& pose, gtsam::noiseModel::Diagonal::shared_ptr& covariance) {
  pose_graph_.Initialize(GetInitialKey(), pose, covariance);

  // // Publish the first pose
  // PublishPoseGraph(true);

  return true;
}

bool LampRobot::InitializeHandlers(const ros::NodeHandle& n) {
  if (!odometry_handler_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize the odometry handler.", name_.c_str());
    return false;
  }

  if (!artifact_handler_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize the artifact handler.", name_.c_str());
    return false;
  }

  if (!stationary_handler_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize the imu handler.", name_.c_str());
    return false;
  }

  return true;
}

// Check for data from all of the handlers
bool LampRobot::CheckHandlers() {
  // b_has_new_factor_ will be set to true if there is a new factor
  // b_run_optimization_ will be set to true if there is a new loop closure

  bool b_have_odom_factors;
  // bool b_have_loop_closure;
  bool b_have_new_artifacts;

  // Check the odom for adding new poses
  b_have_odom_factors = ProcessOdomData(odometry_handler_.GetData());

  // Check all handlers
  // Set the initialized flag in artifacts to start
  // receiving messages.
  if ((pose_graph_.GetValues().size() > 0) && (!is_artifact_initialized)) {
    is_artifact_initialized = true;
    artifact_handler_.SetPgoInitialized(true);
  }
  // Check for artifacts
  b_have_new_artifacts = ProcessArtifactData(artifact_handler_.GetData());

  if (b_add_imu_factors_ && stationary_handler_.has_data_) {
    // Check if we have moved since the last stationary factor
    if (stationary_handler_.CheckKeyRecency(pose_graph_.key)) {
      // Passes, so create a new factor
      // Force new odometry node
      ProcessOdomData(odometry_handler_.GetData(false));
      stationary_handler_.SetKeyForImuAttitude(pose_graph_.key - 1);
      ProcessStationaryData(stationary_handler_.GetData());
    }
  }
  return true;
}

void LampRobot::ProcessTimerCallback(const ros::TimerEvent& ev) {
  // Print some debug messages
  // ROS_INFO_STREAM("Checking for new data");

  // Publish initial node again if we haven't moved in 5s
  if (!b_have_received_first_pg_) {
    init_count_++;
    if (!b_init_pg_pub_) {
      if ((float)init_count_ / update_rate_ > init_wait_time_) {
        // Publish the pose graph
        PublishPoseGraph(true);

        // Get a keyed scan
        PointCloud::Ptr new_scan(new PointCloud);
        // Take away 0.1 from ros::Time::now() so the delay in getting point
        // clouds is accounted for
        if (odometry_handler_.GetKeyedScanAtTime(
                ros::Time::now() - ros::Duration(0.1), new_scan)) {
          AddKeyedScanAndPublish(new_scan, pose_graph_.initial_key);
        } else {
          ROS_WARN("No point clouds from Odom Handler during init in lamp");
        }

        b_init_pg_pub_ = true;
      }
    }
    if (init_count_ % 100 == 0) {
      // Republish every 5 seconds (with 20 hz rate)
      PublishPoseGraph(true);

      // Publish first point cloud
      PointCloud::Ptr new_scan(new PointCloud);
      // Take away 0.1 from ros::Time::now() so the delay in getting point
      // clouds is accounted for
      if (odometry_handler_.GetKeyedScanAtTime(
              ros::Time::now() - ros::Duration(0.1), new_scan)) {
        AddKeyedScanAndPublish(new_scan, pose_graph_.initial_key);
      } else {
        ROS_WARN("No point clouds from Odom Handler during init in lamp");
      }
    }
    if ((float)init_count_ / update_rate_ > repub_first_wait_time_) {
      // Placeholder to move to incremental publishing - TODO - trigger on
      // callback from the base station
      b_have_received_first_pg_ = true;
    }
  }

  // Publish odom
  UpdateAndPublishOdom();

  // Check the handlers
  CheckHandlers();

  // Publish the pose graph
  if (b_has_new_factor_) {
    ROS_DEBUG("Have new factor, publishing pose-graph");
    PublishPoseGraph();

    // Publish the full map (for debug)
    mapper_->PublishMap();

    b_has_new_factor_ = false;
    if (!b_init_pg_pub_) {
      b_init_pg_pub_ = true;
    }
    if (!b_have_received_first_pg_) {
      b_have_received_first_pg_ = true;
    }
  }

  // Start optimize, if needed
  if (b_run_optimization_) {
    ROS_INFO_STREAM(
        "Optimization activated: Publishing pose graph to optimizer");
    PublishPoseGraphForOptimizer();

    b_run_optimization_ = false;
  }

  if (b_received_optimizer_update_) {
    ROS_INFO("LampRobot: received optimizer update");
    pose_graph_.AddLastNodeToNew();
    PublishPoseGraph();

    b_received_optimizer_update_ = false;
  }

  // Publish anything that is needed
}

/*!
  \brief  Calls handler function to update global position of the artifacts
  \author Abhishek Thakur
  \date 09 Oct 2019
*/
void LampRobot::UpdateArtifactPositions() {
  // Get new positions of artifacts from the pose-graph for artifact_key
  std::unordered_map<long unsigned int, ArtifactInfo>& artifact_info_hash =
      artifact_handler_.GetArtifactKey2InfoHash();

  // Result of updating the global pose
  bool result;

  // Loop over to update global pose.
  for (auto const& it : artifact_info_hash) {
    // Get the key
    gtsam::Symbol artifact_key = gtsam::Symbol(it.first);

    // Get the pose from the pose graph
    gtsam::Point3 artifact_position =
        pose_graph_.GetPose(artifact_key).translation();

    // Update global pose just for what has changed. returns bool
    result = result ||
        artifact_handler_.UpdateGlobalPosition(artifact_key, artifact_position);
  }
}

//-------------------------------------------------------------------

// Handler Wrappers
/*!
  \brief  Wrapper for the odom class interactions
  Creates factors from the odom output
  \param   data - the output data struct from the OdometryHandler class
  \warning ...
  \author Benjamin Morrell
  \date 01 Oct 2019
*/
bool LampRobot::ProcessOdomData(std::shared_ptr<FactorData> data) {
  // Extract odom data
  std::shared_ptr<OdomData> odom_data =
      std::dynamic_pointer_cast<OdomData>(data);

  // Check if there are new factors
  if (!odom_data->b_has_data) {
    return false;
  }

  // Record new factor being added - need to publish pose graph(
  ROS_DEBUG("Have Odom Factor");
  b_has_new_factor_ = true;

  // process data for each new factor
  for (auto odom_factor : odom_data->factors) {
    ROS_DEBUG("Adding new odom factor to pose graph");
    // Get the transforms - odom transforms
    Pose3 transform = odom_factor.transform;
    gtsam::SharedNoiseModel covariance =
        odom_factor.covariance; // TODO - check format

    if (b_use_fixed_covariances_) {
      covariance = SetFixedNoiseModels("odom");
    }

    std::pair<ros::Time, ros::Time> times = odom_factor.stamps;

    // Get the previous key - special case for odom that we use key)
    Symbol prev_key = pose_graph_.key - 1;
    // Get the current "new key" value stored in the variable pose_graph_.key
    Symbol current_key = pose_graph_.key;
    // Increment key
    pose_graph_.key = pose_graph_.key + 1;

    // TODO - use this for other handlers: Symbol prev_key =
    // GetKeyAtTime(times.first);

    // Compute the new value with a normalized transform
    Pose3 last_pose = pose_graph_.GetPose(prev_key);
    ROS_DEBUG_STREAM(
        "Last pose det: " << last_pose.rotation().matrix().determinant());
    Eigen::Quaterniond quat(last_pose.rotation().matrix());
    quat = quat.normalized();
    last_pose =
        Pose3(gtsam::Rot3(quat.toRotationMatrix()), last_pose.translation());
    ROS_DEBUG_STREAM(
        "Last pose det after: " << last_pose.rotation().matrix().determinant());

    // Add values to graph so have it for adding map TODO - use unit covariance
    pose_graph_.TrackNode(
        times.second, current_key, last_pose.compose(transform), covariance);

    // add  node/keyframe to keyed stamps
    pose_graph_.InsertKeyedStamp(
        current_key,
        times.second); // TODO - check - can remove as duplicates TrackNode
    pose_graph_.InsertStampedOdomKey(times.second.toSec(), current_key);

    // Track the edges that have been added
    int type = pose_graph_msgs::PoseGraphEdge::ODOM;
    pose_graph_.TrackFactor(prev_key, current_key, type, transform, covariance);

    // Get keyed scan from odom handler
    PointCloud::Ptr new_scan(new PointCloud);

    if (odom_factor.b_has_point_cloud) {
      // Store the keyed scan and add it to the map
      // Copy input scan.
      new_scan = odom_factor.point_cloud;

      if (!new_scan->points.empty() && new_scan != NULL ||
          new_scan->size() != 0) {
        // Add to keyed scans and publish
        AddKeyedScanAndPublish(new_scan, current_key);
      } else {
        ROS_WARN("No valid point cloud with odom factor in Lamp");
      }
    }
  }

  return true;
}

void LampRobot::AddKeyedScanAndPublish(PointCloud::Ptr new_scan,
                                       gtsam::Symbol current_key) {
  // Filter and publish scan
  filter_.Filter(*new_scan, new_scan);

  pose_graph_.InsertKeyedScan(current_key, new_scan);

  AddTransformedPointCloudToMap(current_key);

  // publish keyed scan
  pose_graph_msgs::KeyedScan keyed_scan_msg;
  keyed_scan_msg.key = current_key;
  // Publish the keyed scans without normals
  PointXyziCloud::Ptr pub_scan(new PointXyziCloud);
  utils::ConvertPointCloud(new_scan, pub_scan);
  pcl::toROSMsg(*pub_scan, keyed_scan_msg.scan);
  keyed_scan_pub_.publish(keyed_scan_msg);
}

// Odometry update
void LampRobot::UpdateAndPublishOdom() {
  // Get the pose at the last key
  Pose3 last_pose = pose_graph_.LastPose();

  // Get the delta from the last pose to now
  ros::Time stamp; // = ros::Time::now();
  GtsamPosCov delta_pose_cov;
  // if (!odometry_handler_.GetOdomDelta(stamp, delta_pose_cov)) {
  // Had a bad odom return - try latest time from odometry_handler
  if (!odometry_handler_.GetOdomDeltaLatestTime(stamp, delta_pose_cov)) {
    ROS_WARN_ONCE("No good odom input to LAMP yet");
    return;
  }

  // Compose the delta
  auto delta_pose = delta_pose_cov.pose;
  auto delta_cov = delta_pose_cov.covariance;

  Pose3 new_pose = last_pose.compose(delta_pose);

  // TODO use the covariance when we have it
  // gtsam::Matrix66 covariance;
  // odometry_handler_.GetDeltaCovarianceBetweenTimes(pose_graph_.keyed_stamps[pose_graph_.key-1],
  // stamp, covariance);
  //
  // Compose covariance
  // TODO

  // Convert to ROS to publish
  geometry_msgs::PoseStamped msg;
  msg.pose = utils::GtsamToRosMsg(new_pose);
  msg.header.frame_id = pose_graph_.fixed_frame_id;
  msg.header.stamp = stamp;

  // TODO - use the covariance when we have it
  // geometry_msgs::PoseWithCovarianceStamped msg;
  // msg.pose = utils::GtsamToRosMsg(new_pose, covariance);

  // msg.header.frame_id = pose_graph_.fixed_frame_id;
  // msg.header.stamp = stamp;

  // Publish pose graph
  pose_pub_.publish(msg);
}

/*!
  \brief  Wrapper for the stationary class interactions
  Creates factors from the imu output - called when there the robot stops
  \param   data - the output data struct from the ImuHandler class
  \warning ...time sync
  \author Benjamin Morrell
  \date 22 Nov 2019
*/
bool LampRobot::ProcessStationaryData(std::shared_ptr<FactorData> data) {
  // Extract odom data
  std::shared_ptr<ImuData> imu_data = std::dynamic_pointer_cast<ImuData>(data);

  // Check if there are new factors
  if (!imu_data->b_has_data) {
    return false;
  }

  gtsam::Unit3 ref_unit = imu_data->factors[0].attitude.nZ();
  gtsam::Unit3 meas_unit = imu_data->factors[0].attitude.bRef();
  geometry_msgs::Point meas, ref;
  meas.x = meas_unit.point3().x();
  meas.y = meas_unit.point3().y();
  meas.z = meas_unit.point3().z();
  ref.x = ref_unit.point3().x();
  ref.y = ref_unit.point3().y();
  ref.z = ref_unit.point3().z();

  // gtsam::noiseModel::Isotropic noise =
  // boost::dynamic_pointer_cast<gtsam::noiseModel::Isotropic>(imu_data->factors[0].attitude.noiseModel());
  double noise_sigma =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Isotropic>(
          imu_data->factors[0].attitude.noiseModel())
          ->sigma();

  pose_graph_.TrackIMUFactor(
      imu_data->factors[0].attitude.front(), meas, ref, noise_sigma, true);

  // Do not optimize on the robot
  // Optimize every "imu_factors_per_opt"
  // b_run_optimization_ = false;
  return true;
}

/*!
  \brief  Wrapper for the artifact class interactions
  Creates factors from the artifact output
  \param   data - the output data struct from the ArtifactHandler class
  \warning ...
  \author Abhishek Thakur
  \date 08 Oct 2019
*/
bool LampRobot::ProcessArtifactData(std::shared_ptr<FactorData> data) {
  // Extract artifact data
  std::shared_ptr<ArtifactData> artifact_data =
      std::dynamic_pointer_cast<ArtifactData>(data);

  // Check if there are new factors
  if (!artifact_data->b_has_data) {
    return false;
  }

  ROS_DEBUG("Have Artifact Factor");
  b_has_new_factor_ = true;

  // Necessary variables
  Pose3 transform;
  Pose3 temp_transform;
  Pose3 global_pose;
  gtsam::SharedNoiseModel covariance;
  ros::Time timestamp = ros::Time::now();
  gtsam::Symbol pose_key;
  gtsam::Symbol cur_artifact_key;

  // process data for each new factor
  for (auto artifact : artifact_data->factors) {
    // Get the time
    timestamp = artifact.stamp;

    // Get the artifact key
    cur_artifact_key = artifact.key;

    // Get the pose measurement
    if (b_artifacts_in_global_) {
      // Convert pose to relative frame
      if (!ConvertGlobalToRelative(
              timestamp,
              gtsam::Pose3(gtsam::Rot3(), artifact.position),
              temp_transform)) {
        ROS_ERROR("Can't convert artifact from global to relative");
        b_has_new_factor_ = false;
        // Clean Artifact handler so that these set of
        // factors are removed from history
        artifact_handler_.CleanFailedFactors(false);
        return false;
      }
    } else {
      // Is in relative already
      ROS_DEBUG("Have artifact in relative frame");
      temp_transform = gtsam::Pose3(gtsam::Rot3(), artifact.position);
    }

    // Is a relative tranform, so need to handle linking to the pose-graph
    HandleRelativePoseMeasurement(
        timestamp, temp_transform, transform, global_pose, pose_key);
    ROS_DEBUG("HandleRelativePoseMeasurement");

    if (pose_key == utils::GTSAM_ERROR_SYMBOL) {
      ROS_ERROR("Bad artifact time. Not adding to graph - ERROR THAT NEEDS TO "
                "BE HANDLED OR LOSE ARTIFACTS!!");
      b_has_new_factor_ = false;
      // Clean Artifact handler so that these set of
      // factors are removed from history
      artifact_handler_.CleanFailedFactors(false);
      return false;
    }

    // Can only used fixed covariance for artifact edges TODO: use covariance
    // from artifact msg
    // covariance = SetFixedNoiseModels("artifact");
    covariance = artifact.covariance;

    // Get the covariances (Should be in relative frame as well)
    // TODO - handle this better - need to add covariances from the odom - do in
    // the function above
    // std::cout << msg_d.pose.covariance << '\n';

    // Check if it is a new artifact or not
    if (!pose_graph_.HasKey(cur_artifact_key)) {
      ROS_DEBUG("Have a new artifact in LAMP");

      // Insert into the values TODO - add unit covariance
      std::string id = artifact_handler_.GetArtifactID(cur_artifact_key);
      pose_graph_.TrackNode(
          timestamp, cur_artifact_key, global_pose, covariance, id);

      // Add keyed stamps
      pose_graph_.InsertKeyedStamp(cur_artifact_key, timestamp);

      // Publish the new artifact, with the global pose
      ROS_DEBUG("Calling Publish Artifacts");
      artifact_handler_.PublishArtifacts(cur_artifact_key, global_pose);

      // Add and track the edges that have been added
      int type = pose_graph_msgs::PoseGraphEdge::ARTIFACT;
      pose_graph_.TrackFactor(
          pose_key, cur_artifact_key, type, transform, covariance);
      ROS_DEBUG("Added artifact to pose graph factors in lamp");

    } else {
      // Find artifact edge that is already connected to cur_artifact_key
      auto edge = pose_graph_.FindEdgeKeyTo(cur_artifact_key);

      if (edge == nullptr) {
        ROS_ERROR_STREAM("Edge is not found!!!");
        if (pose_graph_.HasKey(cur_artifact_key)) {
          ROS_DEBUG_STREAM("Posegraph has this key!! "
                          << gtsam::DefaultKeyFormatter(cur_artifact_key));
        }
        return false;
      }

      // Second sighting of an artifact - we have a loop closure
      ROS_INFO_STREAM("\nProcessArtifactData: Artifact re-sighted with key: "
                      << gtsam::DefaultKeyFormatter(cur_artifact_key)
                      << " and from pose key: "
                      << gtsam::DefaultKeyFormatter(pose_key)
                      << ". The artifact is already connected to key: "
                      << gtsam::DefaultKeyFormatter(edge->key_from));

      // Find the transform from the odom node that has been connected to the
      // resighted artifact
      HandleRelativePoseMeasurementWithFixedKey(
          timestamp, temp_transform, edge->key_from, transform, global_pose);

      // Insert into the values TODO - add unit covariance
      std::string id = artifact_handler_.GetArtifactID(cur_artifact_key);

      // TODO Add keyed stamps. If the time stamp changes, .
      pose_graph_.InsertKeyedStamp(cur_artifact_key, timestamp);

      // Publish the new artifact, with the global pose. FYI: We may removed
      // this (check with Kyon)
      // ROS_INFO("Calling Publish Artifacts");
      artifact_handler_.PublishArtifacts(cur_artifact_key, global_pose);

      // Add and track the edges that have been added
      int type = pose_graph_msgs::PoseGraphEdge::ARTIFACT;

      pose_graph_.TrackNode(
          timestamp, cur_artifact_key, global_pose, covariance, id);

      pose_graph_.TrackArtifactFactor(
          edge->key_from, cur_artifact_key, transform, covariance, true, true);

      ROS_INFO("Added resighted artifact to pose graph factors in lamp");
    }
  }

  ROS_DEBUG("Successfully complete ArtifactProcess call with an artifact");

  // Clean up for next iteration
  artifact_handler_.CleanFailedFactors(true);
  return true;
}

// Function gets a relative pose and time, and returns the global pose and the
// transform from the closest node in time, as well as the key of the closest
// node
/*!
  \brief  Function to handle relative measurements and adding them to the
  pose-graph \param   stamp          - The time of the measurement \param
  relative_pose  - The observed relative pose \param   transform      - The
  output transform (for a between factor) \param   global pose    - The output
  global pose estimate \param   key_from       - The output key from which the
  new relative measurement is attached \warning ... \author Benjamin Morrell
  \date 01 Oct 2019
*/
void LampRobot::HandleRelativePoseMeasurement(const ros::Time& stamp,
                                              const gtsam::Pose3& relative_pose,
                                              gtsam::Pose3& transform,
                                              gtsam::Pose3& global_pose,
                                              gtsam::Symbol& key_from) {
  // Get the key from:
  key_from = pose_graph_.GetClosestKeyAtTime(stamp, false);

  if (key_from == utils::GTSAM_ERROR_SYMBOL) {
    ROS_ERROR("Measurement is from a time out of range. Rejecting");
    return;
  }

  // Time from this key - closest time that there is anode
  ros::Time stamp_from = pose_graph_.keyed_stamps[key_from];

  // Get the delta pose from the key_from to the time of the observation
  GtsamPosCov delta_pose_cov;
  delta_pose_cov =
      odometry_handler_.GetFusedOdomDeltaBetweenTimes(stamp_from, stamp);

  if (!delta_pose_cov.b_has_value) {
    ROS_ERROR("---------- [LampRobot::HandleRelativePoseMeasurement] Could not "
              "get delta between times - THIS CASE IS NOT "
              "WELL HANDLED YET-----------");
    key_from = utils::GTSAM_ERROR_SYMBOL;
    return;
  }

  // TODO - do covariances as well

  // Compose the transforms to get the between factor
  gtsam::Pose3 delta_pose = delta_pose_cov.pose;
  gtsam::SharedNoiseModel delta_cov = delta_pose_cov.covariance;
  transform = delta_pose.compose(relative_pose);

  // Compose from the node in the graph to get the global position
  // TODO - maybe do this outside this function
  global_pose = pose_graph_.GetPose(key_from).compose(transform);
}

// Function gets a relative pose and time, and returns the global pose and the
// transform from the closest node in time.
/*!
  \brief  Function to handle relative measurements and adding them to the
  pose-graph \param   stamp          - The time of the measurement \param
  relative_pose  - The observed relative pose \param   transform      - The
  output transform (for a between factor) \param   global pose    - The output
  global pose estimate \param   key_from       - The output key from which the
  new relative measurement is attached \warning ... \author
*/
void LampRobot::HandleRelativePoseMeasurementWithFixedKey(
    const ros::Time& stamp,
    const gtsam::Pose3& relative_pose,
    const gtsam::Symbol& key_from,
    gtsam::Pose3& transform,
    gtsam::Pose3& global_pose) {
  if (key_from == utils::GTSAM_ERROR_SYMBOL) {
    ROS_ERROR("Measurement is from a time out of range. Rejecting");
    return;
  }

  // Time from this key - closest time that there is anode
  ros::Time stamp_from = pose_graph_.keyed_stamps[key_from];

  // Get the delta pose from the key_from to the time of the observation
  GtsamPosCov delta_pose_cov;
  delta_pose_cov =
      odometry_handler_.GetFusedOdomDeltaBetweenTimes(stamp_from, stamp);

  if (!delta_pose_cov.b_has_value) {
    ROS_ERROR("----------[LampRobot::HandleRelativePoseMeasurementWithFixedKey]"
              "Could not get delta between times - THIS CASE IS NOT "
              "WELL HANDLED YET-----------");
    return;
  }

  // TODO - do covariances as well

  // Compose the transforms to get the between factor
  gtsam::Pose3 delta_pose = delta_pose_cov.pose;
  gtsam::SharedNoiseModel delta_cov = delta_pose_cov.covariance;
  transform = delta_pose.compose(relative_pose);

  // Compose from the node in the graph to get the global position
  // TODO - maybe do this outside this function
  global_pose = pose_graph_.GetPose(key_from).compose(transform);
}

// Placeholder function for handling global artifacts (will soon be relative)
bool LampRobot::ConvertGlobalToRelative(const ros::Time stamp,
                                        const gtsam::Pose3 pose_global,
                                        gtsam::Pose3& pose_relative) {
  // Get the closes node in the pose-graph
  gtsam::Symbol key_from = pose_graph_.GetClosestKeyAtTime(stamp, false);

  if (key_from == utils::GTSAM_ERROR_SYMBOL) {
    ROS_ERROR(
        "Key from artifact key_from is outside of range - can't link artifact");
    return false;
  }

  // Pose of closest node
  gtsam::Pose3 node_pose = pose_graph_.GetPose(key_from);

  // Compose to get the relative position (relative pose between node and
  // global)
  pose_relative = node_pose.between(pose_global);

  return true;
}

// TODO Function handler wrappers
// - hopefully a lot of cutting code from others

// TODO
// - Unit tests for these functions
// - How to handle relative measurements not directly at nodes
