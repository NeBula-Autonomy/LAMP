#include "factor_handlers/AprilTagHandler.h"

/**
 * QUESTION: 
 * A little background on fiducials. When are they observed.
 * A overall summary of the code. How they are different from
 * artifacts.
 * What would change in the new messages for april tags
 * What would be the functionality as in what would be in message
 * and what should be in FactorData.
 */ 

/*! \brief Initialize parameters and callbacks. 
 * n - Nodehandle
 * Returns bool
 */
bool AprilTagHandler::Initialize(const ros::NodeHandle& n){
  name_ = ros::names::append(n.getNamespace(), "April Tag");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load April Tag parameters.", name_.c_str());
    return false;
  }
  // Need to change this
  if (!RegisterCallbacks(n, false)) {
    ROS_ERROR("%s: Failed to register April Tag callback.", name_.c_str());
    return false;
  }

  return true;
}

/*! \brief Load April Tag parameters. 
 * n - Nodehandle
 * Returns bool
 */
bool AprilTagHandler::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("b_artifacts_in_global", b_artifacts_in_global_))
    return false;
  if (!pu::Get("use_artifact_loop_closure", use_artifact_loop_closure_)) return false;

  //Get the artifact prefix from launchfile to set initial unique artifact ID
  bool b_initialized_artifact_prefix_from_launchfile = true;
  std::string artifact_prefix;
  unsigned char artifact_prefix_converter[1];
  if (!pu::Get("artifact_prefix", artifact_prefix)){
     b_initialized_artifact_prefix_from_launchfile = false;
     ROS_ERROR("Could not find node ID assosiated with robot_namespace");
  }
  
  if (b_initialized_artifact_prefix_from_launchfile){
    std::copy( artifact_prefix.begin(), artifact_prefix.end(), artifact_prefix_converter);
    artifact_prefix_ = artifact_prefix_converter[0];
  }
  return true; 
}

bool AprilTagHandler::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Create publisher for artifact
  // TODO: Change the message to new April tag message
  artifact_pub_ = nl.advertise<core_msgs::Artifact>("april_tag", 10);

  return true;
}

/*! \brief Register Online callbacks. 
 * n - Nodehandle
 * Returns bool
 */
bool AprilTagHandler::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  artifact_sub_ = nl.subscribe(
    "april_tag_relative", 10, &AprilTagHandler::AprilTagCallback, this);

  return CreatePublishers(n);  
}

/*! \brief  Callback for Artifacts.
  * Returns  Void
  */
void AprilTagHandler::AprilTagCallback(const core_msgs::Artifact& msg) {
  // TODO: Convert April tag message into Artifact message

  // Call Artifact Callback
  ArtifactCallback(msg);
}
