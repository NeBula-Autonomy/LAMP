/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


// Includes
#include <lamp/LampRobot.h>

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

// Constructor (if there is override)
LampRobot::LampRobot() :
  update_rate_(10) {}

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

  if (!pu::Get("update_rate", update_rate_)) return false;

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

  update_timer_ = nl.createTimer(
    update_rate_, &LampRobot::ProcessTimerCallback, this);
    
  back_end_pose_graph_sub_ = nl.subscribe("back_end_pose_graph", 1, &LampRobot::OptimizerUpdateCallback, dynamic_cast<LampBase*>(this));

  return true; 
}

bool LampRobot::CreatePublishers(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  pose_graph_pub_ =
    nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  keyed_scan_pub_ =
    nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);


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
    // artifact_handler_.Initialize(); 

  return true; 
}



// Check for data from all of the handlers
bool LampRobot::CheckHandlers() {

  // Check the odom for adding new poses
  // ProcessOdomData(odometry_handler_.GetData());

  // Check all handlers
  // ProcessArtifactData(artifact_handler_.GetData());
  // ProcessAprilData(april_handler_.GetData());


  // Return true if there is a new node - or have a flag that we are ready to publish 
  return true;
}

bool LampRobot::InitializeGraph(gtsam::Pose3& pose, gtsam::noiseModel::Diagonal::shared_ptr& covariance) {
  nfg_ = NonlinearFactorGraph();
  values_ = Values();
  nfg_.add(PriorFactor<Pose3>(initial_key_, pose, covariance));
  values_.insert(initial_key_, pose);

  ros::Time stamp = ros::Time::now();
  keyed_stamps_[initial_key_] = stamp;

  TrackPriors(stamp, initial_key_, pose, covariance);

  // Populate the priors_info vector 

  return true;
}

void LampRobot::ProcessTimerCallback(const ros::TimerEvent& ev){

  // Print some debug messages
  ROS_INFO_STREAM("Checking for new data");

  // Check the handlers
  CheckHandlers();

  if (b_has_new_factor_) {
    PublishPoseGraph();

    b_has_new_factor_ = false;
  }

  // Start optimize, if needed
  if (b_run_optimization_) {
      // tell LampPgo to optimize
      // TODO Set up publisher for this

      PublishPoseGraph();

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

  //Get new positions of artifacts from the pose-graph 
  
  // Update global pose just for what has changed
  // artifact_handler_.UpdateArtifactPositions(keyed_poses_global);

  // artifact_handler_.PublishArtifacts();
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

  int num_factors = data.time_stamps.size();

  // process data for each new factor 
  for (int i = 0; i < num_factors; i++) {
      

    gtsam::Pose3 transform = data.transforms[i];
    gtsam::SharedNoiseModel covariance = data.covariances[i]; // TODO - fix
    std::pair<ros::Time, ros::Time> times = data.time_stamps[i];


    gtsam::Symbol prev_key = GetKeyAtTime(times.first);
    gtsam::Symbol current_key = GetKeyAtTime(times.second);

    // create the factor
    NonlinearFactorGraph new_factor;
    new_factor.add(BetweenFactor<Pose3>(prev_key, current_key, transform, covariance));

    // TODO develop and chek these
    // TODO - move to another function?

    // add factor to buffer to send to pgo
    
    // add to nfg_ and values_

    // add  node/keyframe to keyed stamps
    keyed_stamps_.insert(
        std::pair<gtsam::Symbol, ros::Time>(current_key, times.second));
    stamp_to_odom_key_.insert(
        std::pair<double, gtsam::Symbol>(times.second.toSec(), current_key));


    TrackEdges(prev_key, current_key, transform, covariance);

    // // check for keyed scans
    // if (odometry_handler_.HasKeyedScanAtTime(times.second)) {

    //   // get keyed scan from odom handler
    //   std::pair<gtsam::Symbol, PointCloud::ConstPtr> new_scan;
    //   new_scan = odometry_handler_.GetKeyedScanAtTime();

    //   // add new keyed scan to map
    //   keyed_scans_.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(new_scan.first, new_scan.second));


    //   // publish keyed scan
    //   pose_graph_msgs::KeyedScan keyed_scan_msg;
    //   keyed_scan_msg.key = current_key_;
    //   pcl::toROSMsg(*new_scan.second, keyed_scan_msg.scan);
    //   keyed_scan_pub_.publish(keyed_scan_msg);

    // }

  }

    
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

  // TODO - fill out this function 

  // process data for each new factor
  int num_factors = data.transforms.size();

  for (int i = 0; i < num_factors; i++) {

    // create the factor

    // add factor to buffer to send to pgo
  }

}

// TODO Function handler wrappers 
// - hopefully a lot of copying code from others


// TODO 
// - Unit tests for these functions 
// - Maps from timestamps to keys - base class
// - How to handle relative measurements not directly at nodes 
// - How to handle keyed scans
// - How to publish the pose-graph (on update, or on timer)
// 


// Pose Graph merger class


