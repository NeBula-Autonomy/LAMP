/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       ()
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
LampRobot::LampRobot() {}

//Destructor
LampRobot::~LampRobot() {}

// Initialization - override for robot specific setup
bool LampRobot::Initialize(const ros::NodeHandle& n) {
  // Get the name of the process
  name_ = ros::names::append(n.getNamespace(), "LampRobot");
  
  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  // Add load params etc
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  // Register Callbacks
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Publishers
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }

  // Init Handlers
  if (!InitializeHandlers(n)){
    ROS_ERROR("%s: Failed to initialize handlers.", name_.c_str());
    return false;
  }
  

  return true; 
}

bool LampRobot::LoadParameters(const ros::NodeHandle& n) {
  // Rates
  if (!pu::Get("rate/update_rate", update_rate_))
    return false;

  // Settings for precisions
  if (!pu::Get("b_use_fixed_covariances", b_use_fixed_covariances_))
    return false;

  // TODO - bring in other parameter

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

  // Set the initial position (from fiducials) - also inits the pose-graph
  if (!SetInitialPosition()) {
    ROS_ERROR("SetInitialPosition failed");
    return false;
  }

  // Timestamp to keys initialization (initilization is particular to the robot version of lamp)
  ros::Time stamp = ros::Time::now();
  keyed_stamps_.insert(std::pair<gtsam::Symbol, ros::Time>(initial_key_, stamp));
  stamp_to_odom_key_.insert(std::pair<double, gtsam::Symbol>(stamp.toSec(), initial_key_));

  // Set initial key 
  key_ = initial_key_ + 1;

  return true;
}

bool LampRobot::RegisterCallbacks(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  update_timer_ = nl.createTimer(update_rate_, &LampRobot::ProcessTimerCallback, this);
    
  back_end_pose_graph_sub_ = nl.subscribe("back_end_pose_graph", 1, &LampRobot::OptimizerUpdateCallback, dynamic_cast<LampBase*>(this));

  return true; 
}

bool LampRobot::CreatePublishers(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Pose Graph publishers
  pose_graph_pub_ =
    nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  pose_graph_to_optimize_pub_ =
    nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph_to_optimize", 10, false);
  keyed_scan_pub_ =
    nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);

  // Publishers
  pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>("lamp_pose", 10, false);

  return true; 
}

bool LampRobot::SetInitialKey(){
  //Get the robot prefix from launchfile to set initial key
  // TODO - get this convertor setup to Kyon
  unsigned char prefix_converter[1];
  
  if (!pu::Get("robot_prefix", prefix_)){
     ROS_ERROR("Could not find node ID assosiated with robot_namespace");
     initial_key_ = 0;
     return false;
  } else {
    std::copy( prefix_.begin(), prefix_.end(), prefix_converter);
    initial_key_ = gtsam::Symbol(prefix_converter[0],0);
    return true;
  }
}

bool LampRobot::SetFactorPrecisions() {
  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_)) return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_)) return false;
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_)) return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_)) return false;
  if (!pu::Get("artifact_rot_precision", artifact_rot_precision_)) return false; 
  if (!pu::Get("artifact_trans_precision", artifact_trans_precision_)) return false;
  if (!pu::Get("point_estimate_precision", point_estimate_precision_)) return false;

  if(!pu::Get("fiducial_trans_precision", fiducial_trans_precision_)) return false;
  if(!pu::Get("fiducial_rot_precision", fiducial_rot_precision_)) return false;

  return true;
}

bool LampRobot::SetInitialPosition() {

  // Load initial position and orientation.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
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
  if (!pu::Get("init/position_sigma/x", sigma_x)) return false;
  if (!pu::Get("init/position_sigma/y", sigma_y)) return false;
  if (!pu::Get("init/position_sigma/z", sigma_z)) return false;
  if (!pu::Get("init/orientation_sigma/roll", sigma_roll)) return false;
  if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch)) return false;
  if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw)) return false;

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
  ROS_INFO_STREAM("covariance is");
  ROS_INFO_STREAM(initial_noise_);


  // Initialize graph  with the initial position
  InitializeGraph(pose, covariance);

  return true;
}

bool LampRobot::InitializeHandlers(const ros::NodeHandle& n){

    if (!odometry_handler_.Initialize(n)) {
      ROS_ERROR("%s: Failed to initialize the odometry handler.", name_.c_str());
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
  b_have_new_artifacts = ProcessArtifactData(artifact_handler_.GetData());
  // ProcessAprilData(april_handler_.GetData());

  // TODO - determine what a true and false return means here
  return true;
}

bool LampRobot::InitializeGraph(gtsam::Pose3& pose, gtsam::noiseModel::Diagonal::shared_ptr& covariance) {
  nfg_ = NonlinearFactorGraph();
  values_ = Values();
  nfg_.add(PriorFactor<Pose3>(initial_key_, pose, covariance));
  values_.insert(initial_key_, pose);

  ros::Time stamp = ros::Time::now();
  keyed_stamps_[initial_key_] = stamp;

  // Populate the priors_info vector
  TrackPriors(stamp, initial_key_, pose, covariance);

  return true;
}

void LampRobot::ProcessTimerCallback(const ros::TimerEvent& ev) {

  // Print some debug messages
  ROS_INFO_STREAM("Checking for new data");

  // Check the handlers
  CheckHandlers();

  // Publish odom
  UpdateAndPublishOdom();

  // Publish the pose graph
  if (b_has_new_factor_) {
    PublishPoseGraph();

    b_has_new_factor_ = false;
  }

  // Start optimize, if needed
  if (b_run_optimization_) {
      PublishPoseGraphForOptimizer();

      b_run_optimization_ = false; 
  }

  // Publish anything that is needed 
  UpdateArtifactPositions();

}


/*! 
  \brief  Calls handler function to update global position of the artifacts
  \author Benjamin Morrell
  \date 01 Oct 2019
*/
void LampRobot::UpdateArtifactPositions(){
  //Get new positions of artifacts from the pose-graph for artifact_key 
  
  // Update global pose just for what has changed
  // artifact_handler_.UpdateGlobalPose(artifact_key, global_pose);

  // artifact_handler_.PublishArtifacts(artifact_key, global_pose);
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
bool LampRobot::ProcessOdomData(FactorData data){

  // Check if there are new factors 
  if (!data.b_has_data) {
    return false;
  }

  // Record new factor being added - need to publish pose graph 
  b_has_new_factor_ = true;

  // Number of factors from size of data vector
  int num_factors = data.time_stamps.size();

  // factors to add
  NonlinearFactorGraph new_factors;

  // Values to add
  Values new_values;

  // process data for each new factor 
  for (int i = 0; i < num_factors; i++) {
    // Get the transforms - odom transforms
    Pose3 transform = data.transforms[i];
    gtsam::SharedNoiseModel covariance =
        data.covariances[i]; // TODO - check format

    if (b_use_fixed_covariances_) {
      covariance = SetFixedNoiseModels("odom");
    }

    std::pair<ros::Time, ros::Time> times = data.time_stamps[i];

    // Get the previous key - special case for odom that we use key)
    Symbol prev_key = key_ - 1;
    // Get the current "new key" value stored in the global variable key_
    Symbol current_key = key_;
    // Increment key
    key_ = key_ + 1;

    // TODO - use this for other handlers: Symbol prev_key =
    // GetKeyAtTime(times.first);

    // create the factor
    new_factors.add(BetweenFactor<Pose3>(prev_key, current_key, transform, covariance));

    // Compute the new value
    Pose3 last_pose = values_.at<Pose3>(prev_key);
    new_values.insert(current_key, last_pose.compose(transform));

    // add  node/keyframe to keyed stamps
    keyed_stamps_.insert(
        std::pair<gtsam::Symbol, ros::Time>(current_key, times.second));
    stamp_to_odom_key_.insert(
        std::pair<double, gtsam::Symbol>(times.second.toSec(), current_key));

    // Track the edges that have been added
    int type = pose_graph_msgs::PoseGraphEdge::ODOM;
    TrackEdges(prev_key, current_key, type, transform, covariance);
    
    // Get keyed scan from odom handler
    PointCloud::Ptr new_scan(new PointCloud);

    if (odometry_handler_.GetKeyedScanAtTime(times.second, new_scan)) {
      // add new keyed scan to map

      // TODO : filter before adding
      keyed_scans_.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(current_key, new_scan));

      // publish keyed scan
      pose_graph_msgs::KeyedScan keyed_scan_msg;
      keyed_scan_msg.key = current_key;
      pcl::toROSMsg(*new_scan, keyed_scan_msg.scan);
      keyed_scan_pub_.publish(keyed_scan_msg);
    }
  }

  // Add factors and values to the graph
  nfg_.add(new_factors);
  values_.insert(new_values);

  return true;
}

// Odometry update
void LampRobot::UpdateAndPublishOdom() {
  // Get the pose at the last key
  Pose3 last_pose = values_.at<Pose3>(key_ - 1);

  // Get the delta from the last pose to now
  ros::Time stamp = ros::Time::now();
  Pose3 delta_pose;
  odometry_handler_.GetDeltaBetweenTimes(
      keyed_stamps_[key_ - 1], stamp, delta_pose);

  // Compose the delta
  Pose3 new_pose = last_pose.compose(delta_pose);

  // TODO use the covariance when we have it
  // gtsam::Matrix66 covariance;
  // odometry_handler_.GetDeltaCovarianceBetweenTimes(keyed_stamps_[key_-1],
  // stamp, covariance);
  //
  // Compose covariance
  // TODO

  // Convert to ROS to publish
  geometry_msgs::PoseStamped msg;
  msg.pose = utils::GtsamToRosMsg(new_pose);
  msg.header.frame_id = fixed_frame_id_;
  msg.header.stamp = stamp;

  // TODO - use the covariance when we have it
  // geometry_msgs::PoseWithCovarianceStamped msg;
  // msg.pose = utils::GtsamToRosMsg(new_pose, covariance);
  // msg.header.frame_id = fixed_frame_id_;
  // msg.header.stamp = stamp;

  // Publish pose graph
  pose_pub_.publish(msg);
}

/*! 
  \brief  Wrapper for the artifact class interactions
  Creates factors from the artifact output 
  \param   data - the output data struct from the ArtifactHandler class
  \warning ...
  \author Alex Stephens
  \date 01 Oct 2019
*/
bool LampRobot::ProcessArtifactData(FactorData data){

  // Check if there are new factors 
  if (!data.b_has_data) {
    return false;
  }

  // Necessary variables
  Pose3 transform;
  gtsam::SharedNoiseModel covariance;
  std::pair<ros::Time, ros::Time> times;
  gtsam::Key pose_key;
  gtsam::Key cur_artifact_key;

  // New Factors to be added
  NonlinearFactorGraph new_factors;

  // New Values to be added
  Values new_values;

  // Get number of new measurements
  int num_factors = data.transforms.size();

  // process data for each new factor
  for (int i = 0; i < num_factors; i++) {
    // Get the transforms
    transform = data.transforms[i];
    // Get the covariances (Should be in relative frame as well)
    covariance = data.covariances[i];
    // Get the time
    times = data.time_stamps[i];
    // Get the key where the artifact loop closure has to be added    
    pose_key = GetKeyAtTime(times.first);
    // Get the artifact key
    cur_artifact_key = data.artifact_key[i];

    // Was adding factor successful 
    bool result;

    // create and add the factor
    new_factors.add(BetweenFactor<Pose3>(pose_key, cur_artifact_key, transform, covariance));

    // Compute the value
    Pose3 last_pose = values_.at<Pose3>(pose_key);
    new_values.insert(cur_artifact_key, last_pose.compose(transform));

    // TODO: Not inserting in keyed_stamps as both nodes are already present
    // as this is a loop closure.

    // Track the edges that have been added
    int type = pose_graph_msgs::PoseGraphEdge::ARTIFACT;
    TrackEdges(pose_key, cur_artifact_key, type, transform, covariance);
  }
  // add factor to buffer to send to pgo
  nfg_.add(new_factors);
  values_.insert(new_values);

  // TODO - use the HandleRelativePoseMeasurement function below

  return true;

}

// Function gets a relative pose and time, and returns the global pose and the
// transform from the closest node in time, as well as the key of the closest
// node
void LampRobot::HandleRelativePoseMeasurement(const ros::Time& stamp,
                                              const gtsam::Pose3& relative_pose,
                                              gtsam::Pose3& transform,
                                              gtsam::Pose3& global_pose,
                                              gtsam::Symbol& key_from) {
  // Get the key from:
  key_from = GetKeyAtTime(stamp);

  // Time from this key - closest time that there is anode
  ros::Time stamp_from = keyed_stamps_[key_from];

  // Get the delta pose from the key_from to the time of the observation
  Pose3 delta_pose;
  odometry_handler_.GetDeltaBetweenTimes(stamp_from, stamp, delta_pose);

  // Compose the transforms to get the between factor
  transform = delta_pose.compose(relative_pose);

  // Compose from the node in the graph to get the global position
  // TODO - maybe do this outside this function
  global_pose = values_.at<Pose3>(key_from).compose(transform);
}

// TODO Function handler wrappers
// - hopefully a lot of cutting code from others

// TODO 
// - Unit tests for these functions 
// - How to handle relative measurements not directly at nodes 
