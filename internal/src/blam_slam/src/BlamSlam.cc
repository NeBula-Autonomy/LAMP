/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <blam_slam/BlamSlam.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

BlamSlam::BlamSlam()
  : estimate_update_rate_(0.0),
    visualization_update_rate_(0.0),
    uwb_update_rate_(0.0),
    position_sigma_(0.01),
    attitude_sigma_(0.04),
    marker_id_(0),
    largest_artifact_id_(0),
    b_first_pose_scan_revieved_(false),
    use_artifact_loop_closure_(false) {}

BlamSlam::~BlamSlam() {}

bool BlamSlam::Initialize(const ros::NodeHandle& n, bool from_log) {
  name_ = ros::names::append(n.getNamespace(), "BlamSlam");

  initial_key_ = 0;

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!loop_closure_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize laser loop closure.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n, from_log)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool BlamSlam::LoadParameters(const ros::NodeHandle& n) {
  // Load update rates.
  if (!pu::Get("rate/estimate", estimate_update_rate_)) return false;
  if (!pu::Get("rate/visualization", visualization_update_rate_)) return false;
  if (!pu::Get("rate/uwb_update", uwb_update_rate_)) return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;
  if (!pu::Get("frame_id/artifacts_in_global", artifacts_in_global_))
    return false;

  // Covariance for odom factors
  if (!pu::Get("noise/odom_position_sigma", position_sigma_)) return false;
  if (!pu::Get("noise/odom_attitude_sigma", attitude_sigma_)) return false;

  //Load and restart deltas
  if (!pu::Get("restart_x", restart_x_)) return false;
  if (!pu::Get("restart_y", restart_y_)) return false;
  if (!pu::Get("restart_z", restart_z_)) return false;
  if (!pu::Get("restart_roll", restart_roll_)) return false;
  if (!pu::Get("restart_pitch", restart_pitch_)) return false;
  if (!pu::Get("restart_yaw", restart_yaw_)) return false;

  // Load uwb information
  if (!pu::Get("uwb/all", uwb_id_list_all_)) return false;
  if (!pu::Get("uwb/drop", uwb_id_list_drop_)) return false;
  for (auto itr = uwb_id_list_all_.begin(); itr != uwb_id_list_all_.end(); itr++) {
    UwbMeasurementInfo uwb_data;
    uwb_data.id = *itr;
    uwb_data.drop_status = true;  // This should be false. (after the enhancement of UWB firmware)
    uwb_id2data_hash_[*itr] = uwb_data;
    uwb_id2data_hash_[*itr].in_pose_graph = false;
  }
  for (auto itr = uwb_id_list_drop_.begin(); itr != uwb_id_list_drop_.end(); itr++) {
    uwb_id2data_hash_[*itr].holder = name_.c_str();
    uwb_id2data_hash_[*itr].drop_status = false;  // This sentence will be removed.
  }

  if (!pu::Get("use_artifact_loop_closure", use_artifact_loop_closure_)) return false;

  if (!pu::Get("b_use_uwb", b_use_uwb_)) return false;
  if (!pu::Get("uwb_skip_measurement_number", uwb_skip_measurement_number_)) return false;
  if (!pu::Get("uwb_update_key_number", uwb_update_key_number_)) return false;
  if (!pu::Get("uwb_required_key_number_first", uwb_required_key_number_first_)) return false;
  if (!pu::Get("uwb_required_key_number_not_first", uwb_required_key_number_not_first_)) return false;

  std::string graph_filename;
  if (pu::Get("load_graph", graph_filename) && !graph_filename.empty()) {
    if (loop_closure_.Load(graph_filename)) {
      PointCloud::Ptr regenerated_map(new PointCloud);
      loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
      mapper_.Reset();
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(regenerated_map, unused.get());

      // Also reset the robot's estimated position.
      be_current_pose_ = loop_closure_.GetLastPose();

      // Publish updated map
      mapper_.PublishMap();
    } else {
      ROS_ERROR_STREAM("Failed to load graph from " << graph_filename);
    }
  }

  return true;
}

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);
      
  add_factor_srv_ = nl.advertiseService("add_factor", &BlamSlam::AddFactorService, this);
  remove_factor_srv_ = nl.advertiseService("remove_factor", &BlamSlam::RemoveFactorService, this);
  save_graph_srv_ = nl.advertiseService("save_graph", &BlamSlam::SaveGraphService, this);
  restart_srv_ = nl.advertiseService("restart", &BlamSlam::RestartService, this);
  load_graph_srv_ = nl.advertiseService("load_graph", &BlamSlam::LoadGraphService, this);
  batch_loop_closure_srv_ = nl.advertiseService("batch_loop_closure", &BlamSlam::BatchLoopClosureService, this);
  drop_uwb_srv_ = nl.advertiseService("drop_uwb_anchor", &BlamSlam::DropUwbService, this);
  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

bool BlamSlam::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool BlamSlam::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  pose_scan_sub_ = nl.subscribe("pose_and_scan", 100000, &BlamSlam::PoseScanCallback, this);

  uwb_update_timer_ = nl.createTimer(uwb_update_rate_, &BlamSlam::UwbTimerCallback, this);

  pcld_sub_ = nl.subscribe("pcld", 100000, &BlamSlam::PointCloudCallback, this);

  artifact_sub_ = nl.subscribe("artifact_relative", 10, &BlamSlam::ArtifactCallback, this);

  uwb_sub_ =
      nl.subscribe("uwb_signal", 1000, &BlamSlam::UwbSignalCallback, this);

  return CreatePublishers(n);
}

bool BlamSlam::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);

  return true;
}

bool BlamSlam::AddFactorService(blam_slam::AddFactorRequest &request,
                                blam_slam::AddFactorResponse &response) {
  // TODO - bring the service creation into this node?
  if (!request.confirmed) {
    ROS_WARN("Cannot add factor because the request is not confirmed.");
    response.success = false;
    return true;
  }

  // Get last node pose before doing loop closure
  gu::Transform3 last_key_pose;
  last_key_pose = loop_closure_.GetLastPose();

  const gtsam::Pose3 pose_from_to 
      = gtsam::Pose3(gtsam::Rot3(request.qw, request.qx, request.qy, request.qz), gtsam::Point3());
  pose_from_to.print("Between pose is ");

  response.success = loop_closure_.AddManualLoopClosure(
    static_cast<unsigned int>(request.key_from),
    static_cast<unsigned int>(request.key_to),
    pose_from_to);
  response.confirm = false;
  if (response.success){
    std::cout << "adding factor for loop closure succeeded" << std::endl;
  } else {
    std::cout << "adding factor for loop closure failed" << std::endl;
  }

  // Update the map from the loop closures
  std::cout << "Updating the map" << std::endl;
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // Get new pose
  // New key pose of last pose key
  gu::Transform3 new_key_pose = loop_closure_.GetLastPose();
  // Update to the pose of the last key
  // Current estimate
  gu::Transform3 new_pose = be_current_pose_;
  // Delta translation
  new_pose.translation = new_pose.translation + (new_key_pose.translation - last_key_pose.translation);
  // Delta rotation
  new_pose.rotation = new_pose.rotation*(new_key_pose.rotation*last_key_pose.rotation.Trans());

  // Also reset the robot's estimated position.
  be_current_pose_ = new_pose;

  // Sends pose graph to visualizer node, if graph has changed.
  loop_closure_.PublishPoseGraph();

  // Publish artifacts - should be updated from the pose-graph 
  loop_closure_.PublishArtifacts();

  // Publish updated map
  mapper_.PublishMap();

  std::cout << "Updated the map" << std::endl;

  return true;
}

bool BlamSlam::RemoveFactorService(blam_slam::RemoveFactorRequest &request,
                                   blam_slam::RemoveFactorResponse &response) {
  // TODO - bring the service creation into this node?
  if (!request.confirmed) {
    ROS_WARN("Cannot remove factor because the request is not confirmed.");
    response.success = false;
    return true;
  }
  response.success =
      loop_closure_.RemoveFactor(static_cast<unsigned int>(request.key_from),
                                 static_cast<unsigned int>(request.key_to));
  if (response.success){
    std::cout << "removing factor from pose graph succeeded" << std::endl;
  }else{
    std::cout << "removing factor from pose graph failed" << std::endl;
  }

  // Update the map from the loop closures
  std::cout << "Updating the map" << std::endl;
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // Also reset the robot's estimated position.
  be_current_pose_ = loop_closure_.GetLastPose();

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish artifacts - should be updated from the pose-graph 
  loop_closure_.PublishArtifacts();

  // Publish updated map
  mapper_.PublishMap(); // TODO publish elsewhere?

  std::cout << "Updated the map" << std::endl;

  return true;
}

bool BlamSlam::SaveGraphService(blam_slam::SaveGraphRequest &request,
                                blam_slam::SaveGraphResponse &response) {
  std::cout << "Saving graph..." << std::endl;
  response.success = loop_closure_.Save(request.filename);
  return true;
}

bool BlamSlam::LoadGraphService(blam_slam::LoadGraphRequest &request,
                                blam_slam::LoadGraphResponse &response) {
  std::cout << "Loading graph..." << std::endl;
  loop_closure_.ErasePosegraph();
  response.success = loop_closure_.Load(request.filename);

  // Regenerate the 3D map from the loaded posegraph
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());
  // change the key number for the second robot to 10000
  loop_closure_.ChangeKeyNumber();

  initial_key_ = loop_closure_.GetKey();
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = attitude_sigma_*attitude_sigma_; //0.4, 0.004; 0.2 m sd
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = position_sigma_*position_sigma_; //0.1, 0.01; sqrt(0.01) rad sd
  
  gu::Transform3 init_pose = loop_closure_.GetInitialPose();
  gu::Transform3 current_pose = loop_closure_.GetCurrentPose();
  const gu::Transform3 pose_delta = gu::PoseDelta(init_pose, current_pose);
  loop_closure_.AddFactorAtLoad(pose_delta, covariance);

  // Also reset the robot's estimated position.
  be_current_pose_ = loop_closure_.GetLastPose();
  return true;
}

bool BlamSlam::BatchLoopClosureService(blam_slam::BatchLoopClosureRequest &request,
                                blam_slam::BatchLoopClosureResponse &response) {
 
  std::cout << "BATCH LOOP CLOSURE. Looking for any loop closures..." << std::endl;

  response.success = loop_closure_.BatchLoopClosure();

  if (response.success){
    ROS_INFO("Found Loop Closures in batch loop closure");
  
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    be_current_pose_ = loop_closure_.GetLastPose();
  }else {
    ROS_INFO("No loop closures in batch loop closure");
  }
  return true; 
}


bool BlamSlam::DropUwbService(mesh_msgs::ProcessCommNodeRequest &request,
                              mesh_msgs::ProcessCommNodeResponse &response) {
  ROS_INFO_STREAM("Dropped UWB anchor is " + request.node.AnchorID);

  Eigen::Vector3d aug_robot_position = be_current_pose_.translation.Eigen();

  loop_closure_.DropUwbAnchor(request.node.AnchorID, request.node.DropTime, aug_robot_position);

  uwb_id2data_hash_[request.node.AnchorID].drop_status = true;
  
  return true;
}

void BlamSlam::PoseScanCallback(const core_msgs::PoseAndScanConstPtr& msg) {
  
  ROS_INFO("Inside PoseScanCallback");

  // Get the pose
  geometry_utils::Transform3 fe_pose = geometry_utils::ros::FromROS(msg->pose.pose);// Change name to include pose

  PointCloud::Ptr received_cloud_ptr;
  received_cloud_ptr.reset(new PointCloud);
  // sensor_msgs::PointCloud2ConstPtr pointcloud_msg;
  
  pcl::fromROSMsg( msg->scan, *received_cloud_ptr.get());

  // Process pose and scan
  ProcessPoseScanMessage(fe_pose, received_cloud_ptr);

  return;
}

void BlamSlam::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  // TODO - for other front-ends
  ROS_WARN("Point Cloud Callback Not yet implemented");
  // use filtering etc/ to sync with odome messages, then use
  // ProcessPoseScanMessage(fe_pose, msg.scan);
  return;
}

void BlamSlam::EstimateTimerCallback(const ros::TimerEvent& ev) {
  // Sort all messages accumulated since the last estimate update.
  synchronizer_.SortMessages();

  // NOT IMPLEMENTED OR USED CURRENTLY
  return;

  // // Not currently used, but consider using when using a different odom source 
  // // Sort all messages accumulated since the last estimate update.
  // synchronizer_.SortMessages();

  // // Iterate through sensor messages, passing to update functions.
  // MeasurementSynchronizer::sensor_type type;
  // unsigned int index = 0;
  // while (synchronizer_.GetNextMessage(&type, &index)) {
  //   switch(type) {

  //     // Point cloud messages.
  //     case MeasurementSynchronizer::PCL_POINTCLOUD: {
  //       const MeasurementSynchronizer::Message<PointCloud>::ConstPtr& m =
  //           synchronizer_.GetPCLPointCloudMessage(index);

  //       ProcessPointCloudMessage(m->msg);
  //       break;
  //     }

  //     // Unhandled sensor messages.
  //     default: {
  //       ROS_WARN("%s: Unhandled measurement type (%s).", name_.c_str(),
  //                MeasurementSynchronizer::GetTypeString(type).c_str());
  //       break;
  //     }
  //   }
  // }

  // // Remove processed messages from the synchronizer.
  // synchronizer_.ClearMessages();
}

void BlamSlam::ArtifactCallback(const core_msgs::Artifact& msg) {
  // Subscribe to artifact messages, include in pose graph, publish global position 

  std::cout << "Artifact message received is for id " << msg.id << std::endl;
  std::cout << "\t Parent id: " << msg.parent_id << std::endl;
  std::cout << "\t Confidence: " << msg.confidence << std::endl;
  std::cout << "\t Position:\n[" << msg.point.point.x << ", "
            << msg.point.point.y << ", " << msg.point.point.z << "]"
            << std::endl;
  std::cout << "\t Label: " << msg.label << std::endl;

  // Check for NaNs and reject 
  if (std::isnan(msg.point.point.x) || std::isnan(msg.point.point.y) || std::isnan(msg.point.point.z)){
    ROS_WARN("NAN positions input from artifact message - ignoring");
    return;
  }

  // Get artifact position 
  Eigen::Vector3d artifact_position;
  artifact_position << msg.point.point.x, msg.point.point.y, msg.point.point.z;

  Eigen::Vector3d R_artifact_position; // In robot frame

  gtsam::Key pose_key = loop_closure_.GetKeyAtTime(msg.point.header.stamp);

  // Chck if artifact is published in global frame 
  // And convert to local frame to include in pose graph 
  if (artifacts_in_global_) { // Already in fixed frame
    geometry_utils::Transform3 global_pose = loop_closure_.GetPoseAtKey(pose_key);

    // Transform artifact pose from global frame to body frame 
    Eigen::Matrix<double, 3, 3> R_global = global_pose.rotation.Eigen();
    Eigen::Matrix<double, 3, 1> T_global = global_pose.translation.Eigen();
    // std::cout << "Global robot position is: " << T_global[0] << ", " << T_global[1] << ", " << T_global[2] << std::endl;
    // std::cout << "Global robot rotation is: " << R_global << std::endl;

    R_artifact_position = R_global.transpose() * (artifact_position - T_global); // Apply transform
  } else {
    R_artifact_position = artifact_position;
  }

  std::cout << "Artifact position in robot frame is: " << R_artifact_position[0] << ", "
            << R_artifact_position[1] << ", " << R_artifact_position[2]
            << std::endl;

  std::string artifact_id = msg.parent_id; // Note that we are looking at the parent id here
  gtsam::Key cur_artifact_key; 
  bool b_is_new_artifact = false;
  gu::Transform3 last_key_pose;
  // get artifact id / key -----------------------------------------------
  // Check if the ID of the object already exists in the object hash
  if (use_artifact_loop_closure_ && artifact_id2key_hash.find(artifact_id) != artifact_id2key_hash.end() && 
      msg.label != "cellphone") {
    // Take the ID for that object - no reconciliation in the pose-graph if a cell phone (for now)
    cur_artifact_key = artifact_id2key_hash[artifact_id];
    std::cout << "artifact previously observed, artifact id " << artifact_id 
              << " with key in pose graph " 
              << gtsam::DefaultKeyFormatter(cur_artifact_key) << std::endl;
    // Get last node pose before doing artifact loop closure 
    last_key_pose = loop_closure_.GetLastPose();
  } else {
    // New artifact - increment the id counters
    b_is_new_artifact = true;
    ++largest_artifact_id_;
    cur_artifact_key = gtsam::Symbol('l', largest_artifact_id_);
    std::cout << "new artifact observed, artifact id " << artifact_id 
              << " with key in pose graph " 
              << gtsam::DefaultKeyFormatter(cur_artifact_key) << std::endl;
    // update hash
    artifact_id2key_hash[artifact_id] = cur_artifact_key;
  }

  // add to pose graph and optimize --------------------------------------
  const gtsam::Pose3 R_pose_A 
      = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(R_artifact_position[0], 
                                                  R_artifact_position[1],
                                                  R_artifact_position[2]));
  R_pose_A.print("Between pose is ");

  ArtifactInfo artifactinfo(msg.parent_id);
  artifactinfo.msg = msg;

  bool result = loop_closure_.AddArtifact(
    pose_key,
    cur_artifact_key,
    R_pose_A, 
    artifactinfo);

  if (result){
    std::cout << "adding artifact observation succeeded" << std::endl;
  } else {
    std::cout << "adding artifact observation failed" << std::endl;
  }

  if (b_is_new_artifact){
    // Don't need to update the map at all - just publish artifacts
    // Publish artifacts - from pose-graph positions
    ROS_INFO_STREAM("Publishing new artifact key: " << gtsam::DefaultKeyFormatter(cur_artifact_key));
    loop_closure_.PublishArtifacts(cur_artifact_key);
  }else{
    // Loop closure has been performed - update the graph
    // Update the map from the loop closures
    std::cout << "Updating the map" << std::endl;
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Get new pose
    // New key pose of last pose key
    gu::Transform3 new_key_pose = loop_closure_.GetLastPose();
    // Update to the pose of the last key
    // Current estimate
    gu::Transform3 new_pose = be_current_pose_;
    // Delta translation from the loop closure change to the last pose node
    new_pose.translation = new_pose.translation + (new_key_pose.translation - last_key_pose.translation);
    // Delta rotation
    new_pose.rotation = new_pose.rotation*(new_key_pose.rotation*last_key_pose.rotation.Trans());

    // Update localization
    // Also reset the robot's estimated position.
    be_current_pose_ = new_pose;

    // Visualize the pose graph updates
    loop_closure_.PublishPoseGraph();

    // Publish artifacts - from pose-graph positions
    loop_closure_.PublishArtifacts();

    // Publish updated map // TODO have criteria of change for when to publish the map?
    mapper_.PublishMap();
  }
}

void BlamSlam::UwbTimerCallback(const ros::TimerEvent& ev) {
  if (!b_use_uwb_) {
    return;
  }

  for (auto itr = uwb_id2data_hash_.begin(); itr != uwb_id2data_hash_.end(); itr++) {
    std::string uwb_id = itr->first;
    UwbMeasurementInfo data = itr->second;

    if (data.range.size() > uwb_skip_measurement_number_) {

      // Calculate the pose key number difference between the latest pose key and the pose key linked with UWB
      auto latest_pose_key = loop_closure_.GetKeyAtTime(ros::Time::now());
      auto latest_obs_key = loop_closure_.GetKeyAtTime(data.time_stamp.back());
      int key_diff = gtsam::symbolIndex(latest_pose_key) - gtsam::symbolIndex(latest_obs_key);

      // Check whether enough number of pose key is passed or not
      if (key_diff >= uwb_update_key_number_) {
        
        // Count the number of pose keys linked with UWB
        // If it's the first time observation, at least three keys are necessary to localize the UWB anchor
        unsigned int required_key_number;
        if (uwb_id2data_hash_[uwb_id].in_pose_graph == false) {
          required_key_number = uwb_required_key_number_first_;
        }
        else {
          required_key_number = uwb_required_key_number_not_first_;
        }

        auto pose_key_list = data.nearest_pose_key;
        pose_key_list.erase(std::unique(pose_key_list.begin(), pose_key_list.end()), pose_key_list.end());
        if (pose_key_list.size() >= required_key_number) {
          ProcessUwbRangeData(uwb_id);
          UwbClearBuffer(uwb_id);
        }
        else {
          ROS_INFO("Number of range measurement is NOT enough");
        }
      }
    }
  }
  return;
}

void BlamSlam::UwbClearBuffer(const std::string uwb_id) {
  // Clear the UWB data buffer after processing the data and adding RangeFactor
  uwb_id2data_hash_[uwb_id].range.clear();
  uwb_id2data_hash_[uwb_id].time_stamp.clear();
  uwb_id2data_hash_[uwb_id].robot_position.clear();
  uwb_id2data_hash_[uwb_id].dist_posekey.clear();
  uwb_id2data_hash_[uwb_id].nearest_pose_key.clear();
}


void BlamSlam::ProcessUwbRangeData(const std::string uwb_id) {
  ROS_INFO_STREAM("Start to process UWB range measurement data of " << uwb_id);

  if (loop_closure_.AddUwbFactor(uwb_id, uwb_id2data_hash_[uwb_id])) {  
    ROS_INFO("Updating the map by UWB data");
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    be_current_pose_ = loop_closure_.GetLastPose();

    // Visualize the pose graph and current loop closure radius.
    loop_closure_.PublishPoseGraph();

    // Publish updated map
    mapper_.PublishMap();

    ROS_INFO("Updated the map by UWB Range Factors");

    uwb_id2data_hash_[uwb_id].in_pose_graph = true;
  }
  return;
}

void BlamSlam::UwbSignalCallback(const uwb_msgs::Anchor& msg) {
  if (!b_use_uwb_) {
    return;
  }
  
  // Store the UWB-related data into the buffer "uwb_id2data_hash_"
  auto itr = uwb_id2data_hash_.find(msg.id);
  if (itr != end(uwb_id2data_hash_)) {
    if (uwb_id2data_hash_[msg.id].drop_status == true) {
      uwb_id2data_hash_[msg.id].range.push_back(msg.range);
      uwb_id2data_hash_[msg.id].time_stamp.push_back(msg.header.stamp);
      uwb_id2data_hash_[msg.id].robot_position.push_back(localization_.GetIntegratedEstimate().translation.Eigen());
      uwb_id2data_hash_[msg.id].nearest_pose_key.push_back(loop_closure_.GetKeyAtTime(msg.header.stamp));
    }
  } 
  else {
    ROS_WARN("Not registered UWB ID");
  }
  return;
}

void BlamSlam::VisualizationTimerCallback(const ros::TimerEvent& ev) {
  mapper_.PublishMap();
}

void BlamSlam::ProcessPoseScanMessage(geometry_utils::Transform3& fe_pose, const PointCloud::Ptr& scan) {
  ROS_INFO("Inside processPoseScanMessage");
  
  PointCloud::Ptr scan_filtered(new PointCloud);
  filter_.Filter(scan, scan_filtered);

  PointCloud::Ptr scan_transformed(new PointCloud);
  PointCloud::Ptr scan_fixed(new PointCloud);

  if (!b_first_pose_scan_revieved_){
    ROS_INFO("Frist processPoseScanMessage");
    // First update ever.

    // Update what the last pose added to pose graph
    fe_last_pose_ = fe_pose;

    // Init current pose in back-end
    be_current_pose_ = fe_pose;

    // Transform if not starting at 0, 0, 0,
    TransformPointsToFixedFrame(*scan_filtered,
                                            scan_transformed.get());


    // Transform point cloud 
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(scan_filtered, unused.get());
    loop_closure_.AddKeyScanPair(initial_key_, scan, true);
    // TODO: check how initial key is set

    // Publish
    loop_closure_.PublishPoseGraph();

    // have first scan now
    b_first_pose_scan_revieved_ = true;

    return;
  }

  // Containers.
  PointCloud::Ptr msg_neighbors(new PointCloud);
  PointCloud::Ptr msg_base(new PointCloud);
  PointCloud::Ptr msg_fixed(new PointCloud);

  // Update delta 
  geometry_utils::Transform3 pose_delta = geometry_utils::PoseDelta(fe_last_pose_, fe_pose);

  // reset fe_last_pose_
  fe_last_pose_ = fe_pose;

  bool new_keyframe = false;
  if (HandleLoopClosures(scan, pose_delta, &new_keyframe)) {
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Update current pose
    be_current_pose_ = loop_closure_.GetLastPose();
    
    // Publish artifacts - should be updated from the pose-graph 
    loop_closure_.PublishArtifacts();

  } else {
    // ROS_INFO("No new loop closures");
    // No new loop closures - but was there a new key frame? If so, add new
    // points to the map.
    if (new_keyframe) {
      // Update current pose
      be_current_pose_ = loop_closure_.GetLastPose();

      // Update the map
      TransformPointsToFixedFrame(*scan, scan_fixed.get());
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(scan_fixed, unused.get());

    } else{
      // Update current pose with input delta
      be_current_pose_ = geometry_utils::PoseUpdate(be_current_pose_, pose_delta);

    }
  }

  // Only publish the pose-graph if there is a new keyframe TODO consider changing to publishing for each new node 
  if (new_keyframe){
    // Visualize the pose graph and current loop closure radius.
    loop_closure_.PublishPoseGraph();
  }   
  

  // // Publish the incoming point cloud message from the base frame.
  // if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
  //   PointCloud base_frame_pcld = *msg;
  //   base_frame_pcld.header.frame_id = base_frame_id_;
  //   base_frame_pcld_pub_.publish(base_frame_pcld);
  // }

}

bool BlamSlam::RestartService(blam_slam::RestartRequest &request,
                                blam_slam::RestartResponse &response) {    
  ROS_INFO_STREAM(request.filename);
  // Erase the current posegraph to make space for the backup
  loop_closure_.ErasePosegraph();  
  // Run the load function to retrieve the posegraph
  response.success = loop_closure_.Load(request.filename);

  // Regenerate the 3D map from the loaded posegraph
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  initial_key_ = loop_closure_.GetKey();
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = attitude_sigma_*attitude_sigma_; //0.4, 0.004; 0.2 m sd
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = position_sigma_*position_sigma_; //0.1, 0.01; sqrt(0.01) rad sd

  // This will add a between factor after obtaining the delta between poses.
  delta_after_restart_.translation = gu::Vec3(restart_x_, restart_y_, restart_z_);
  delta_after_restart_.rotation = gu::Rot3(restart_roll_, restart_pitch_, restart_yaw_);
  loop_closure_.AddFactorAtRestart(delta_after_restart_, covariance);

  // Also reset the robot's estimated position.
  be_current_pose_ = loop_closure_.GetLastPose();
  return true;
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan, geometry_utils::Transform3 pose_delta,
                                  bool* new_keyframe) {
  if (new_keyframe == NULL) {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  unsigned int pose_key;
  // Add the new pose to the pose graph (BetweenFactor)
  // TODO rename to attitude and position sigma
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = attitude_sigma_ * attitude_sigma_; // 0.4, 0.004; 0.2 m
                                                          // sd
  for (int i = 3; i < 6; ++i)
    covariance(i, i) =
        position_sigma_ * position_sigma_; // 0.1, 0.01; sqrt(0.01) rad sd

  const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);

  // TODO: compose covariances as well

  // Add between factor
  if (!loop_closure_.AddBetweenFactor(pose_delta,
                                      covariance,
                                      stamp,
                                      &pose_key)) {
    return false;
  }

  *new_keyframe = true;

  if (!loop_closure_.AddKeyScanPair(pose_key, scan, false)) {
    return false;
  }

  std::vector<unsigned int> closure_keys;
  if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys)) {
    return false;
  }

  for (const auto& closure_key : closure_keys) {
    ROS_INFO("%s: Closed loop between poses %u and %u.", name_.c_str(),
             pose_key, closure_key);
  }
  return true;
}

bool BlamSlam::TransformPointsToFixedFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }
}

bool BlamSlam::getTransformEigenFromTF(
    const std::string& parent_frame,
    const std::string& child_frame,
    const ros::Time& time,
    Eigen::Affine3d& T) {
  ros::Duration timeout(3.0);
  tf::StampedTransform tf_tfm;
  try {
    tf_listener_.waitForTransform(parent_frame, child_frame, time, timeout);
    tf_listener_.lookupTransform(parent_frame, child_frame, time, tf_tfm);
  } catch (tf::TransformException& ex) {
    ROS_FATAL("%s", ex.what());
  }
  tf::transformTFToEigen(tf_tfm, T);
}
