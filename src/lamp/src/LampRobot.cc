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

// Constructor (if there is override)
LampRobot::LampRobot() :
  update_rate_(10) {}

//Destructor
LampRobot::~LampRobot() {}

// Initialization - override for robot specific setup
bool LampRobot::Initialize(const ros::NodeHandle& n) {
  // Add load params etc


  return true; 
}

bool LampRobot::LoadParameters(const ros::NodeHandle& n) {

  if (!pu::Get("update_rate", update_rate_)) return false;

  // TODO initialize keyed_stamps_ and stamps_keyed_


  return true;
}

bool LampRobot::RegisterOnlineCallbacks(const ros::NodeHandle& n) {

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  update_timer_ = nl.createTimer(
    update_rate_, &LampRobot::ProcessTimerCallback, this);
    


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

bool LampRobot::InitializeHandlers(){
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

}



void LampRobot::ProcessTimerCallback(const ros::TimerEvent& ev){

  // Print some debug messages
  ROS_INFO_STREAM("Checking for new data");

  // Check the handlers
  CheckHandlers();

  // Start optimize, if needed
  if (b_run_optimization_) {
      // tell LampPgo to optimize
      // TODO Set up publisher for this

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

  int num_factors = data.time_stamps.size();

  // process data for each new factor 
  for (int i = 0; i < num_factors; i++) {
      /*

        gtsam::Pose3 transform = data.transforms[i];
        Mat1212 covariance = data.covariances[i];
        std::pair<ros::Time, ros::Time> times = time_stamps[i];

        // create the factor
        NonlinearFactorGraph new_factor;
        new_factor.add(BetweenFactor<Pose3>(prev_key_, current_key_, transform, covariance));

        // TODO develop and chek these
        // TODO - move to another function?

        // add factor to buffer to send to pgo
        
        // add to nfg_ and values_

        // add  node/keyframe to keyed stamps
        keyed_stamps_.insert(
            std::pair<gtsam::Symbol, ros::Time>(current_key_, times.second));
        stamps_keyed_.insert(
            std::pair<double, gtsam::Symbol>(times.second, current_key_));


        // check for keyed scans
        if (odometry_handler_.HasKeyedScanAtTime(times.second)) {

          // get keyed scan from odom handler
          std::pair<gtsam::Symbol, PointCloud::ConstPtr> new_scan;
          new_scan = odometry_handler_.GetKeyedScanAtTime();

          // add new keyed scan to map
          keyed_scans_.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(new_scan.first, new_scan.second));


          // publish keyed scan
          pose_graph_msgs::KeyedScan keyed_scan_msg;
          keyed_scan_msg.key = key;
          pcl::toROSMsg(*new_scan.second, keyed_scan_msg.scan);
          keyed_scan_pub_.publish(keyed_scan_msg);

          */

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