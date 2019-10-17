// Includes
#include "factor_handlers/ArtifactHandler.h"

/**
 * Constructor             - Done
 * Initialize              - Done
 * LoadParameters          - Done
 * RegisterCallbacks       - Not sure here
 * ComputeTransform        - Done
 * GetData                 - Done
 * PublishArtifacts        - Done
 * RegisterOnlineCallbacks - b_is_basestation_ and b_use_lo_frontend_ and
 * b_is_front_end_ in lamp. else Nearly Done GetArtifactID           - Done
 * ArtifactCallback        - Check if this needs more
 * ArtifactBaseCallback    - Need to check this
 * RegisterLogCallbacks    - Done
 * CreatePublishers        - Done
 * UpdateGlobalPose        - Done
 * Tests                   - Test needed.
 * covariances             - Done
 */

// Constructor
ArtifactHandler::ArtifactHandler()
  : largest_artifact_id_(0), use_artifact_loop_closure_(false) {}

/*! \brief Initialize parameters and callbacks.
 * n - Nodehandle
 * Returns bool
 */
bool ArtifactHandler::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "Artifact");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load artifact parameters.", name_.c_str());
    return false;
  }
  // Need to change this
  if (!RegisterCallbacks(n, false)) {
    ROS_ERROR("%s: Failed to register artifact callback.", name_.c_str());
    return false;
  }

  return true;
}

/*! \brief Load artifact parameters.
 * n - Nodehandle
 * Returns bool
 */
bool ArtifactHandler::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("b_artifacts_in_global", b_artifacts_in_global_))
    return false;
  if (!pu::Get("use_artifact_loop_closure", use_artifact_loop_closure_))
    return false;

  // Get the artifact prefix from launchfile to set initial unique artifact ID
  bool b_initialized_artifact_prefix_from_launchfile = true;
  std::string artifact_prefix;
  unsigned char artifact_prefix_converter[1];
  if (!pu::Get("artifact_prefix", artifact_prefix)) {
    b_initialized_artifact_prefix_from_launchfile = false;
    ROS_ERROR("Could not find node ID assosiated with robot_namespace");
  }

  if (b_initialized_artifact_prefix_from_launchfile) {
    std::copy(artifact_prefix.begin(),
              artifact_prefix.end(),
              artifact_prefix_converter);
    artifact_prefix_ = artifact_prefix_converter[0];
  }
  return true;
}

/*! \brief Register callbacks.
 * n - Nodehandle
 * Returns bool
 */
bool ArtifactHandler::RegisterCallbacks(const ros::NodeHandle& n,
                                        bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

/*! \brief Compute transform from Artifact message.
 * Returns Transform
 */
Eigen::Vector3d
ArtifactHandler::ComputeTransform(const core_msgs::Artifact& msg) {
  // Get artifact position
  Eigen::Vector3d artifact_position;
  artifact_position << msg.point.point.x, msg.point.point.y, msg.point.point.z;

  std::cout << "Artifact position in robot frame is: " << artifact_position[0]
            << ", " << artifact_position[1] << ", " << artifact_position[2]
            << std::endl;
  return artifact_position;
}

/*! \brief  Get artifacts ID from artifact key
 * Returns Artifacts ID
 */
std::string ArtifactHandler::GetArtifactID(gtsam::Symbol artifact_key) {
  std::string artifact_id;
  for (auto it = artifact_id2key_hash.begin(); it != artifact_id2key_hash.end();
       ++it) {
    if (it->second == artifact_key) {
      artifact_id = it->first;
      return artifact_id;
    }
  }
  std::cout << "Artifact ID not found for key"
            << gtsam::DefaultKeyFormatter(artifact_key) << std::endl;
  return "";
}

/*! \brief  Callback for Artifacts.
 * Returns  Void
 */
void ArtifactHandler::ArtifactCallback(const core_msgs::Artifact& msg) {
  // Subscribe to artifact messages, include in pose graph, publish global
  // position Artifact information
  PrintArtifactInputMessage(msg);

  // Check for NaNs and reject
  if (std::isnan(msg.point.point.x) || std::isnan(msg.point.point.y) ||
      std::isnan(msg.point.point.z)) {
    ROS_WARN("NAN positions input from artifact message - ignoring");
    return;
  }

  // Get the transformation
  Eigen::Vector3d R_artifact_position = ComputeTransform(msg);

  // Get the artifact id
  std::string artifact_id =
      msg.parent_id; // Note that we are looking at the parent id here

  // Artifact key
  gtsam::Symbol cur_artifact_key;
  bool b_is_new_artifact = false;

  // get artifact id / key -----------------------------------------------
  // Check if the ID of the object already exists in the object hash
  if (use_artifact_loop_closure_ &&
      artifact_id2key_hash.find(artifact_id) != artifact_id2key_hash.end() &&
      msg.label != "cellphone") {
    // Take the ID for that object - no reconciliation in the pose-graph of a
    // cell phone (for now)
    cur_artifact_key = artifact_id2key_hash[artifact_id];
    std::cout << "artifact previously observed, artifact id " << artifact_id
              << " with key in pose graph "
              << gtsam::DefaultKeyFormatter(cur_artifact_key) << std::endl;
  } else {
    // New artifact - increment the id counters
    b_is_new_artifact = true;
    cur_artifact_key = gtsam::Symbol(artifact_prefix_, largest_artifact_id_);
    ++largest_artifact_id_;
    std::cout << "new artifact observed, artifact id " << artifact_id
              << " with key in pose graph "
              << gtsam::DefaultKeyFormatter(cur_artifact_key) << std::endl;
    // update hash
    artifact_id2key_hash[artifact_id] = cur_artifact_key;
  }
  // Generate gtsam pose
  const gtsam::Pose3 relative_pose =
      gtsam::Pose3(gtsam::Rot3(),
                   gtsam::Point3(R_artifact_position[0],
                                 R_artifact_position[1],
                                 R_artifact_position[2]));

  // Fill ArtifactInfo hash
  StoreArtifactInfo(cur_artifact_key, msg);

  // Extract covariance
  gtsam::SharedNoiseModel noise = ExtractCovariance(msg.covariance);

  // Fill artifact_data_
  AddArtifactData(cur_artifact_key,
                  std::make_pair(msg.header.stamp, ros::Time(0.0)),
                  relative_pose,
                  noise);
}

/*! \brief  Gives the factors to be added and clears to start afresh.
 * Returns  Factors
 */
FactorData ArtifactHandler::GetData() {
  // Create a temporary copy to return
  FactorData temp_artifact_data_ = artifact_data_;

  // Clear artifact data
  ClearArtifactData();

  // Return artifact data
  return temp_artifact_data_;
}

/*! \brief  Create the publishers to log data.
 * Returns  Values
 */
bool ArtifactHandler::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool ArtifactHandler::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Create publisher for artifact
  artifact_pub_ = nl.advertise<core_msgs::Artifact>("artifact", 10);

  return true;
}

/*! \brief Register Online callbacks.
 * n - Nodehandle
 * Returns bool
 */
bool ArtifactHandler::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  artifact_sub_ = nl.subscribe(
      "artifact_relative", 10, &ArtifactHandler::ArtifactCallback, this);

  return CreatePublishers(n);
}

/*! \brief  Updates the global pose of an artifact
 * Returns  Void
 */
bool ArtifactHandler::UpdateGlobalPose(gtsam::Symbol artifact_key,
                                       gtsam::Pose3 global_pose) {
  if (artifact_key2info_hash_.find(artifact_key) !=
      artifact_key2info_hash_.end()) {
    artifact_key2info_hash_[artifact_key].global_pose = global_pose;
    return true;
  } else {
    std::cout << "Key not found in the Artifact id to key map.";
    return false;
  }
}

/*! \brief  Publish Artifact. Need to see if publish_all is still relevant
 * I am considering publishing if we have the key and the pose without
 * any further processing.
 * TODO Resolve the frame and transform between world and map
 * in output message.
 * Returns  Void
 */
void ArtifactHandler::PublishArtifacts(gtsam::Symbol artifact_key,
                                       gtsam::Pose3 global_pose) {
  // Get the artifact pose
  Eigen::Vector3d artifact_position = global_pose.translation().vector();
  std::string artifact_label;

  if (!(artifact_key.chr() == 'l' || artifact_key.chr() == 'm' ||
        artifact_key.chr() == 'n' || artifact_key.chr() == 'o' ||
        artifact_key.chr() == 'p' || artifact_key.chr() == 'q')) {
    ROS_WARN("ERROR - have a non-landmark ID");
    ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(artifact_key));
    return;
  }

  // Using the artifact key to publish that artifact
  ROS_INFO("Publishing the new artifact");
  ROS_INFO_STREAM("Artifact key to publish is "
                  << gtsam::DefaultKeyFormatter(artifact_key));

  // Check that the key exists
  if (artifact_key2info_hash_.count(artifact_key) == 0) {
    ROS_WARN("Artifact key is not in hash, nothing to publish");
    return;
  }

  // Get label
  artifact_label = artifact_key2info_hash_[artifact_key].msg.label;

  // Increment update count
  artifact_key2info_hash_[artifact_key].num_updates++;

  std::cout << "Number of updates of artifact is: "
            << artifact_key2info_hash_[artifact_key].num_updates << std::endl;

  // Fill artifact message
  core_msgs::Artifact new_msg = artifact_key2info_hash_[artifact_key].msg;

  // Update the time
  new_msg.header.stamp = ros::Time::now();

  // Fill the new message positions
  new_msg.point.point.x = artifact_position[0];
  new_msg.point.point.y = artifact_position[1];
  new_msg.point.point.z = artifact_position[2];
  // TODO Need to check the frame id and transform
  // new_msg.point.header.frame_id = fixed_frame_id_;
  // Transform to world frame from map frame
  // new_msg.point = tf_buffer_.transform(
  // new_msg.point, "world", new_msg.point.header.stamp, "world");

  // Print out
  // Transform at time of message
  PrintArtifactInputMessage(new_msg);

  // Publish
  artifact_pub_.publish(new_msg);
}

/*! \brief  Print Artifact input message for debugging
 * Returns  Void
 */
void ArtifactHandler::PrintArtifactInputMessage(
    const core_msgs::Artifact& artifact) {
  std::cout << "Artifact position in world is: " << artifact.point.point.x
            << ", " << artifact.point.point.y << ", " << artifact.point.point.z
            << std::endl;
  std::cout << "Frame ID is: " << artifact.point.header.frame_id << std::endl;

  std::cout << "\t Parent id: " << artifact.parent_id << std::endl;
  std::cout << "\t Confidence: " << artifact.confidence << std::endl;
  std::cout << "\t Position:\n[" << artifact.point.point.x << ", "
            << artifact.point.point.y << ", " << artifact.point.point.z << "]"
            << std::endl;
  std::cout << "\t Label: " << artifact.label << std::endl;
}

/*! \brief  Extracts covariance from artifact message and converts to
 * gtsam::SharedNoiseModel Returns  gtsam::SharedNoiseModel
 */
gtsam::SharedNoiseModel
ArtifactHandler::ExtractCovariance(const boost::array<float, 9> covariance) {
  // Extract covariance information
  gtsam::Matrix33 cov;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      cov(i + 3, j + 3) = covariance[3 * i + j];

  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Covariance(cov);
  return noise;
}

/*! \brief  Clear artifact data
 * Returns  Void
 */
void ArtifactHandler::ClearArtifactData() {
  // Clear artifact data
  artifact_data_.b_has_data = false;
  artifact_data_.artifact_key.clear();
  artifact_data_.covariances.clear();
  artifact_data_.time_stamps.clear();
  artifact_data_.transforms.clear();
}

/*! \brief  Add artifact data
 * Returns  Void
 */
void ArtifactHandler::AddArtifactData(
    const gtsam::Symbol cur_key,
    std::pair<ros::Time, ros::Time> time_stamp,
    const gtsam::Pose3 transform,
    const gtsam::SharedNoiseModel noise) {
  // Make new data true
  artifact_data_.b_has_data = true;
  // Fill type
  artifact_data_.type = "artifact";
  // Append transform
  artifact_data_.transforms.push_back(transform);
  // Append covariance
  artifact_data_.covariances.push_back(noise);
  // Append std::pair<ros::Time, ros::Time(0.0)> for artifact
  artifact_data_.time_stamps.push_back(time_stamp);
  // Append the artifact key
  artifact_data_.artifact_key.push_back(cur_key);
}

/*! \brief  Stores/Updated artifactInfo Hash
 * Returns  Void
 */
void ArtifactHandler::StoreArtifactInfo(const gtsam::Symbol artifact_key,
                                        const core_msgs::Artifact& msg) {
  ArtifactInfo artifactinfo(msg.parent_id);
  artifactinfo.msg = msg;

  // keep track of artifact info: add to hash if not added
  if (artifact_key2info_hash_.find(artifact_key) ==
      artifact_key2info_hash_.end()) {
    ROS_INFO_STREAM("New artifact detected with key "
                    << gtsam::DefaultKeyFormatter(artifact_key));
    artifact_key2info_hash_[artifact_key] = artifactinfo;
  } else {
    ROS_INFO("Existing artifact detected");
    artifact_key2info_hash_[artifact_key] = artifactinfo;
  }
}