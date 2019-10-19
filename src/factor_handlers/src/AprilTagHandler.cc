#include "factor_handlers/AprilTagHandler.h"

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

  // Load april tag related parameters here.
  // TODO: Change this section to reflect new april tags.
  if (!pu::Get("calibration_left_x", calibration_left_x_)) return false;
  if (!pu::Get("calibration_left_y", calibration_left_y_)) return false;
  if (!pu::Get("calibration_left_z", calibration_left_z_)) return false;
  
  if (!pu::Get("calibration_right_x", calibration_right_x_)) return false;
  if (!pu::Get("calibration_right_y", calibration_right_y_)) return false;
  if (!pu::Get("calibration_right_z", calibration_right_z_)) return false;
  
  if (!pu::Get("distal_x", distal_x_)) return false;
  if (!pu::Get("distal_y", distal_y_)) return false;
  if (!pu::Get("distal_z", distal_z_)) return false;

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

  return true;
}

/*! \brief  Callback for April tags.
  * Returns  Void
  */
void AprilTagHandler::AprilTagCallback(const core_msgs::AprilTag& msg) {
  // Convert April tag message into Artifact message
  core_msgs::Artifact artifact_msg = ConvertAprilTagMsgToArtifactMsg(msg);

  // How many times have we processed this artifact
  int num_updates;
  // Find num_updates
  if (artifact_id2key_hash.find(msg.id) == artifact_id2key_hash.end()){
    num_updates = 0;
  } else {
    num_updates = artifact_key2info_hash_[artifact_id2key_hash[msg.id]].num_updates;
  }

  // Call Artifact Callback
  ArtifactCallback(artifact_msg);

  // If new april tag then update the global position in
  // stored april tag info artifact_key2info_hash_
  // Get the april tag key from artifact_id2key_hash map
  gtsam::Symbol april_tag_key;

  // If callback above succeeded
  if (artifact_key2info_hash_[april_tag_key].num_updates - num_updates == 1){
    // Update type of factor data to april from artifact in base class
    artifact_data_.type = "april";
    // If new measurement and successful callback, update the global position
    if ((num_updates == 0) && (artifact_key2info_hash_[april_tag_key].num_updates == 1)) {
      // Check what kind - curently distal, calibration left and right 
      if (artifact_key2info_hash_[april_tag_key].id == "distal") {
          artifact_key2info_hash_[april_tag_key].global_position = gtsam::Point3(distal_x_, distal_y_, distal_z_);
      } else if (artifact_key2info_hash_[april_tag_key].id == "calibration_left") {
          artifact_key2info_hash_[april_tag_key].global_position = gtsam::Point3(calibration_left_x_, calibration_left_y_, calibration_left_z_);
      } else if (artifact_key2info_hash_[april_tag_key].id == "calibration_right") {
          artifact_key2info_hash_[april_tag_key].global_position = gtsam::Point3(calibration_right_x_, calibration_right_y_, calibration_right_z_);
      }
    }
  } 
}

/*! \brief  Convert April tag message to Artifact message.
  * Returns  Artifacts message
  */
core_msgs::Artifact AprilTagHandler::ConvertAprilTagMsgToArtifactMsg(const core_msgs::AprilTag& msg) {
  core_msgs::Artifact artifact_msg;

  // Fill april tags name
  artifact_msg.name = msg.name;
  // Put empty label
  artifact_msg.label = "";
  // Fill empty sequence
  artifact_msg.seq = 0;
  // Put empty id 
  artifact_msg.id = "";
  // Fill parent id
  artifact_msg.parent_id = msg.id;
  // Fill empty hostspot name
  artifact_msg.hotspot_name = "";
  // Fill pose value
  artifact_msg.point = msg.point;
  // Fill confidence with 1
  artifact_msg.confidence = 1.0;
  // Fill the covariance
  artifact_msg.covariance = msg.covariance;

  // Return message
  return artifact_msg;
}

/*! \brief  Get ground truth data from April tag node key.  
  * Returns  Ground truth information
  */
gtsam::Pose3 AprilTagHandler::GetGroundTruthData(const gtsam::Symbol april_tag_key) {
  if (artifact_key2info_hash_.find(april_tag_key) != artifact_key2info_hash_.end()) {
    return gtsam::Pose3(gtsam::Rot3(), artifact_key2info_hash_[april_tag_key].global_position);
  } else {
    std::cout << "Key not found in the April Tag id to key map.";
    return gtsam::Pose3();
  }
}