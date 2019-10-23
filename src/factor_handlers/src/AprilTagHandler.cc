#include "factor_handlers/AprilTagHandler.h"

/**
 * \brief Constructor
 */ 
AprilTagHandler::AprilTagHandler()
{
  // April tag prefix is set to T
  artifact_prefix_='T';
}

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
  ROS_INFO("%s: Registering online callbacks for April Tags.", name_.c_str());

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

  // Check if a new key arrived
  bool new_april_tag = false;
  if (artifact_id2key_hash.find(msg.id) == artifact_id2key_hash.end()){
    new_april_tag = true;
  }

  // Call Artifact Callback
  ArtifactCallback(artifact_msg);

  // Get the april tag key from artifact_id2key_hash map
  gtsam::Symbol april_tag_key;

  // Is the first callback for the new tag successful
  bool is_successful = false;
  if (artifact_id2key_hash.find(msg.id) != artifact_id2key_hash.end()){
    is_successful = true;
  }

  // If new april tag then update the global position in
  // stored april tag info artifact_key2info_hash_
  if (new_april_tag && is_successful) {
    april_tag_key = artifact_id2key_hash[msg.id];
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

/*! \brief  Convert April tag message to Artifact message.
  * Returns  Artifacts message
  */
core_msgs::Artifact AprilTagHandler::ConvertAprilTagMsgToArtifactMsg(const core_msgs::AprilTag& msg) const {
  core_msgs::Artifact artifact_msg;

  // Fill the header
  artifact_msg.header = msg.header;
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

/*! \brief Gives the factors to be added and clears to start afresh.
 *  \return New factor data
 */
FactorData* AprilTagHandler::GetData() {
  // Create a temporary copy to return
  FactorData* data_ptr = new AprilTagData(artifact_data_);

  // Clear artifact data
  ClearArtifactData();

  // Return artifact data
  return data_ptr;
}

/*! \brief  Add artifact data
 * Returns  Void
 */
void AprilTagHandler::AddArtifactData(const gtsam::Symbol april_tag_key, 
                                      const ros::Time time_stamp, 
                                      const gtsam::Point3 transform, 
                                      const gtsam::SharedNoiseModel noise) {
  // Make new data true
  artifact_data_.b_has_data = true;
  // Fill type
  artifact_data_.type = "april";

  // Create and add the new artifact
  AprilTagFactor new_april_tag;
  new_april_tag.position = transform;
  new_april_tag.covariance = noise;
  new_april_tag.stamp = time_stamp;
  new_april_tag.key = april_tag_key;

  artifact_data_.factors.push_back(new_april_tag);
}
