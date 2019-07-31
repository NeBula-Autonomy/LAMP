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
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <ctime>

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
    use_artifact_loop_closure_(false), 
    use_two_vlps_(false),
    b_is_front_end_(false) {}

BlamSlam::~BlamSlam() {}

bool BlamSlam::Initialize(const ros::NodeHandle& n, bool from_log) {
  name_ = ros::names::append(n.getNamespace(), "BlamSlam");

  initial_key_ = 0;

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
    return false;
  }

  if (!loop_closure_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize laser loop closure.", name_.c_str());
    return false;
  }

  if (!localization_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
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

  // check if lamp is run as basestation
  b_is_basestation_ = false;
  if (!pu::Get("b_is_basestation", b_is_basestation_)) return false;
  if (!pu::Get("b_is_front_end", b_is_front_end_)) return false;
  if (!pu::Get("b_use_lo_frontend", b_use_lo_frontend_)) return false;

  // set up subscriber to all robots if run on basestation
  if (b_is_basestation_) {
    if (!pu::Get("robot_names", robot_names_))
      return false;
  }

  XmlRpc::XmlRpcValue uwb_list;
  if (!pu::Get("uwb_list", uwb_list)) return false;
  std::vector<std::string> uwb_id_list_drop;
  if (!pu::Get("uwb_drop/"+getRobotName(n), uwb_id_list_drop)) return false;
  for (int i = 0; i < uwb_list.size(); i++) {
    UwbMeasurementInfo uwb_data;
    std::string uwb_id = uwb_list[i]["id"];
    uwb_data.id = uwb_id;
    uwb_data.drop_status = true;  // This should be false. (after the enhancement of UWB firmware)
    uwb_id2data_hash_[uwb_id] = uwb_data;
    uwb_id2data_hash_[uwb_id].in_pose_graph = false;
  }
  for (auto itr = uwb_id_list_drop.begin(); itr != uwb_id_list_drop.end(); itr++) {
    uwb_id2data_hash_[*itr].holder = getRobotName(n);
    uwb_id2data_hash_[*itr].drop_status = false;  // This sentence will be removed.
  }

  if (!pu::Get("use_artifact_loop_closure", use_artifact_loop_closure_)) return false;

  if (!pu::Get("b_use_uwb", b_use_uwb_)) return false;
  if (!pu::Get("uwb_skip_measurement_number", uwb_skip_measurement_number_)) return false;
  if (!pu::Get("uwb_update_key_number", uwb_update_key_number_)) return false;
  if (!pu::Get("uwb_required_key_number_first", uwb_required_key_number_first_)) return false;
  if (!pu::Get("uwb_required_key_number_not_first", uwb_required_key_number_not_first_)) return false;
  if (!pu::Get("uwb_first_key_threshold", uwb_first_key_threshold_)) return false;
  
  if (!pu::Get("use_two_vlps", use_two_vlps_)){ROS_INFO("No setting for VLPs, use 1");};
  ROS_INFO_STREAM("VLP setting is, use two? " << use_two_vlps_);

  pu::Get("b_publish_tfs", b_publish_tfs_);
  
  
  std::string graph_filename;
  if (pu::Get("load_graph", graph_filename) && !graph_filename.empty()) {
    if (loop_closure_.Load(graph_filename)) {
      PointCloud::Ptr regenerated_map(new PointCloud);
      loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
      mapper_.Reset();
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(regenerated_map, unused.get());

      // Also reset the robot's estimated position.
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
      be_current_pose_ = loop_closure_.GetLastPose();

      // Publish updated map
      mapper_.PublishMap();
    } else {
      ROS_ERROR_STREAM("Failed to load graph from " << graph_filename);
    }
  }

  //Get the initial key value to initialize timestamp and pointcloud msgs
  initial_key_ = loop_closure_.GetInitialKey();

  // Initialize boolean to add first scan to key
  b_add_first_scan_to_key_ = false;

  // Initialize pose update to read from pose graph
  b_new_pose_available_ = true;

  // Skipe initialize artifact unique ID if base station
  if (b_is_basestation_) {
    ROS_INFO("LAMP run as base_station");
    return true;
  }
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

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);
      
  pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "localization_integrated_estimate", 10, false);

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

  if (!b_is_basestation_ && !b_use_lo_frontend_){ // NOTE THIS NEEDS TO BE CHANGED IF MORE FLEXIBILITY IS DESIRED
    if (b_is_front_end_){
      estimate_update_timer_ = nl.createTimer(
          estimate_update_rate_, &BlamSlam::EstimateTimerCallback, this);
      
      if (use_two_vlps_) {
        ROS_INFO("Subscribing to two point cloud topics.");
        pcld1_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "pcld", 10);
        pcld2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "pcld2", 10);
        pcld_synchronizer = std::unique_ptr<PcldSynchronizer>(
          new PcldSynchronizer(PcldSyncPolicy(pcld_queue_size_), *pcld1_sub_, *pcld2_sub_));
        pcld_synchronizer->registerCallback(&BlamSlam::TwoPointCloudCallback, this);
      } else {
        pcld_sub_ = nl.subscribe("pcld", 100000, &BlamSlam::PointCloudCallback, this);
      }
    } else {

      uwb_update_timer_ = nl.createTimer(uwb_update_rate_, &BlamSlam::UwbTimerCallback, this);

      uwb_sub_ =
          nl.subscribe("uwb_signal", 1000, &BlamSlam::UwbSignalCallback, this);

      artifact_sub_ = nl.subscribe(
          "artifact_relative", 10, &BlamSlam::ArtifactCallback, this);
    }
  }
  // Create pose-graph callbacks for base station
  if(b_is_basestation_){
    int num_robots = robot_names_.size();
    // init size of subscribers
    // loop through each robot to set up subscriber
    for (size_t i = 0; i < num_robots; i++) {
      ros::Subscriber keyed_scan_sub = nl.subscribe<pose_graph_msgs::KeyedScan>(
          "/" + robot_names_[i] + "/blam_slam/keyed_scans",
          10,
          &BlamSlam::KeyedScanCallback,
          this);
      ros::Subscriber pose_graph_sub = nl.subscribe<pose_graph_msgs::PoseGraph>(
          "/" + robot_names_[i] + "/blam_slam/pose_graph",
          1,
          &BlamSlam::PoseGraphCallback,
          this);
      ros::Subscriber pose_update_sub = nl.subscribe<geometry_msgs::PoseStamped>(
          "/" + robot_names_[i] + "/blam_slam/localization_incremental_estimate",
          1,
          &BlamSlam::PoseUpdateCallback,
          this);
      ros::Subscriber artifact_base_sub =
          nl.subscribe("/" + robot_names_[i] + "/blam_slam/artifact_global",
                       10,
                       &BlamSlam::ArtifactBaseCallback,
                       this);

      Subscriber_posegraphList_.push_back(pose_graph_sub);
      Subscriber_poseList_.push_back(pose_update_sub);
      Subscriber_keyedscanList_.push_back(keyed_scan_sub);
      Subscriber_artifactList_.push_back(artifact_base_sub);
      ROS_INFO_STREAM(i);
    }    
  }

  if (!b_is_front_end_){
    ros::Subscriber keyed_scan_sub =  nl.subscribe<pose_graph_msgs::KeyedScan>(
        "keyed_scans_sub",
        10,
        &BlamSlam::KeyedScanCallback,
        this);
    ros::Subscriber pose_graph_sub = nl.subscribe<pose_graph_msgs::PoseGraph>(
        "pose_graph_sub",
        10,
        &BlamSlam::PoseGraphCallback,
        this);
    ros::Subscriber artifact_base_sub =
        nl.subscribe("artifact_global_sub",
                      10,
                      &BlamSlam::ArtifactBaseCallback,
                      this);
    ros::Subscriber pose_update_sub =
        nl.subscribe("localization_incremental_estimate_sub",
                      10,
                      &BlamSlam::PoseUpdateCallback,
                      this);
    Subscriber_posegraphList_.push_back(pose_graph_sub);
    Subscriber_poseList_.push_back(pose_update_sub);
    Subscriber_keyedscanList_.push_back(keyed_scan_sub);
    Subscriber_artifactList_.push_back(artifact_base_sub);
  }

  ROS_INFO("Creating message filters");

  if (b_use_lo_frontend_){
    this->filterPointSub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "pcld_sync", 10);
    this->filterPoseSub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nl, "fe_pose", 10);


    this->poseScanSync_ = new message_filters::Synchronizer
            <
                message_filters::sync_policies::ApproximateTime
                <
                  sensor_msgs::PointCloud2,
                  geometry_msgs::PoseStamped
                >
            >(
            message_filters::sync_policies::ApproximateTime
            <
              sensor_msgs::PointCloud2,
              geometry_msgs::PoseStamped
            >(10),
            *this->filterPointSub_,
            *this->filterPoseSub_
    );
  this->poseScanSync_->registerCallback(&BlamSlam::PoseAndScanFilterCB, this);
  }


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
 
  //Convert from string and int to gtsam::Symbol 
  unsigned char prefix_from = request.prefix_from[0];
  unsigned int key_from = request.key_from;
  gtsam::Symbol id_from (prefix_from,key_from);

  unsigned char prefix_to = request.prefix_to[0]; 
  unsigned int key_to = request.key_to;
  gtsam::Symbol id_to (prefix_to,key_to);
  
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
    static_cast<gtsam::Symbol>(id_from),
    static_cast<gtsam::Symbol>(id_to),
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

  // Current estimate
  gu::Transform3 new_pose;

  // Get new pose
  if (!b_is_front_end_){
    // No updates between poses, just get from graph
    new_pose = loop_closure_.GetLastPose();
  } else {
    // New key pose of last pose key
    gu::Transform3 new_key_pose = loop_closure_.GetLastPose();
    // Update to the pose of the last key
    if (b_use_lo_frontend_){
      new_pose = be_current_pose_;
    } else {
      new_pose = localization_.GetIntegratedEstimate();
    }
    
    
    ROS_INFO_STREAM("After manual LC, old pose is " << new_pose.translation);
    ROS_INFO_STREAM("After manual LC, last node pose " << new_key_pose.translation);

    // Delta translation
    new_pose.translation = new_pose.translation + (new_key_pose.translation - last_key_pose.translation);
    // Delta rotation
    new_pose.rotation = new_pose.rotation*(new_key_pose.rotation*last_key_pose.rotation.Trans());
  }

  ROS_INFO_STREAM("After manual LC,  pose is " << new_pose.translation);

  // Also reset the robot's estimated position.
  if (b_use_lo_frontend_){
    be_current_pose_ = new_pose;
    PublishPoseWithLoFrontend();
  } else {
    localization_.SetIntegratedEstimate(new_pose);
    
    // Publish localization pose messages
    ros::Time stamp = loop_closure_.GetTimeAtLastKey();
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
  }

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
    
  //Convert from string and int to gtsam::Symbol 
  unsigned char prefix_from = request.prefix_from[0];
  unsigned int key_from = request.key_from;
  gtsam::Symbol id_from (prefix_from,key_from);

  unsigned char prefix_to = request.prefix_to[0]; 
  unsigned int key_to = request.key_to;
  gtsam::Symbol id_to (prefix_to,key_to);


  // TODO - bring the service creation into this node?
  if (!request.confirmed) {
    ROS_WARN("Cannot remove factor because the request is not confirmed.");
    response.success = false;
    return true;
  }
  bool is_batch_loop_closure = false; // TODO check if we need this
  response.success = loop_closure_.RemoveFactor(
    static_cast<gtsam::Symbol>(id_from),
    static_cast<gtsam::Symbol>(id_to),
    is_batch_loop_closure);
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
  if (b_use_lo_frontend_){
    be_current_pose_ = loop_closure_.GetLastPose();
    PublishPoseWithLoFrontend();
  } else {
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
    
    // Publish localization pose messages
    ros::Time stamp = loop_closure_.GetTimeAtLastKey();
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
  }

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish artifacts - should be updated from the pose-graph 
  loop_closure_.PublishArtifacts();

  // Publish updated map
  mapper_.PublishMap();

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
  // TODO: If robot namespace is same as loaded, then erase
  loop_closure_.ErasePosegraph();

  response.success = loop_closure_.Load(request.filename);

  // change the key number for the second robot
  // This also resets key_ in loop_closure_
  loop_closure_.ChangeKeyNumber();

  // Regenerate the 3D map from the loaded posegraph
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // TODO: Double check this
  // TODO: Do we need initial_key_ here?
  initial_key_ = loop_closure_.GetInitialKey();
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = attitude_sigma_*attitude_sigma_; //0.4, 0.004; 0.2 m sd
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = position_sigma_*position_sigma_; //0.1, 0.01; sqrt(0.01) rad sd
  
  gu::Transform3 init_pose = loop_closure_.GetInitialLoadedPose();
  gu::Transform3 current_pose = localization_.GetIntegratedEstimate(); // loop_closure_.GetCurrentPose();
  const gu::Transform3 pose_delta = gu::PoseDelta(init_pose, current_pose);
  loop_closure_.AddFactorAtLoad(pose_delta, covariance);

  // Also reset the robot's estimated position.
  if (b_use_lo_frontend_){
    be_current_pose_ = loop_closure_.GetLastPose();
    PublishPoseWithLoFrontend();
  } else {
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
    
    // Publish localization pose messages
    ros::Time stamp = loop_closure_.GetTimeAtLastKey();
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
  }

  return true;
}

bool BlamSlam::BatchLoopClosureService(blam_slam::BatchLoopClosureRequest &request,
                                blam_slam::BatchLoopClosureResponse &response) {
 
  std::cout << "BATCH LOOP CLOSURE. Looking for any loop closures..." << std::endl;

  response.success = loop_closure_.BatchLoopClosure();
  // We found one - regenerate the 3D map.
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  if (response.success){
    ROS_INFO("Found Loop Closures in batch loop closure");
  
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    if (b_use_lo_frontend_){
      be_current_pose_ = loop_closure_.GetLastPose();
      PublishPoseWithLoFrontend();
    } else {
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
      
      // Publish localization pose messages
      ros::Time stamp = loop_closure_.GetTimeAtLastKey();
      localization_.UpdateTimestamp(stamp);
      localization_.PublishPoseNoUpdate();
    }

    // Visualize the pose graph updates
    loop_closure_.PublishPoseGraph();

    // Publish artifacts - from pose-graph positions
    loop_closure_.PublishArtifacts();

  }else {
    ROS_INFO("No loop closures in batch loop closure");
  }
  return true; 
}


bool BlamSlam::DropUwbService(mesh_msgs::ProcessCommNodeRequest &request,
                              mesh_msgs::ProcessCommNodeResponse &response) {
  ROS_INFO_STREAM("Dropped UWB anchor is " + request.node.AnchorID);
  
  Eigen::Vector3d aug_robot_position;

  if (b_use_lo_frontend_){
    aug_robot_position = be_current_pose_.translation.Eigen();
  } else {
    aug_robot_position = localization_.GetIntegratedEstimate().translation.Eigen();
  }

  loop_closure_.DropUwbAnchor(request.node.AnchorID, request.node.DropTime, aug_robot_position);

  uwb_id2data_hash_[request.node.AnchorID].drop_status = true;
  
  return true;
}

void BlamSlam::TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                                     const sensor_msgs::PointCloud2::ConstPtr& pcld2) {

  // Merge point clouds
  PointCloud p1, p2;
  pcl::fromROSMsg(*pcld1, p1);
  pcl::fromROSMsg(*pcld2, p2);
  PointCloud::ConstPtr sum(new PointCloud(p1 + p2));
  PointCloudCallback(sum);
}

void BlamSlam::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  synchronizer_.AddPCLPointCloudMessage(msg);
}

void BlamSlam::EstimateTimerCallback(const ros::TimerEvent& ev) {
  // Sort all messages accumulated since the last estimate update.
  synchronizer_.SortMessages();

  // Iterate through sensor messages, passing to update functions.
  MeasurementSynchronizer::sensor_type type;
  unsigned int index = 0;
  while (synchronizer_.GetNextMessage(&type, &index)) {
    switch(type) {

      // Point cloud messages.
      case MeasurementSynchronizer::PCL_POINTCLOUD: {
        const MeasurementSynchronizer::Message<PointCloud>::ConstPtr& m =
            synchronizer_.GetPCLPointCloudMessage(index);

        ProcessPointCloudMessage(m->msg);
        break;
      }

      // Unhandled sensor messages.
      default: {
        ROS_WARN("%s: Unhandled measurement type (%s).", name_.c_str(),
                 MeasurementSynchronizer::GetTypeString(type).c_str());
        break;
      }
    }
  }

  // Remove processed messages from the synchronizer.
  synchronizer_.ClearMessages();
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

  // Get global pose (of robot)
  // geometry_utils::Transform3 global_pose = localization_.GetIntegratedEstimate();

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
    // Take the ID for that object - no reconciliation in the pose-graph of a cell phone (for now)
    cur_artifact_key = artifact_id2key_hash[artifact_id];
    std::cout << "artifact previously observed, artifact id " << artifact_id 
              << " with key in pose graph " 
              << gtsam::DefaultKeyFormatter(cur_artifact_key) << std::endl;
    // Get last node pose before doing artifact loop closure 
    last_key_pose = loop_closure_.GetLastPose();
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

  // add to pose graph and optimize --------------------------------------
  const gtsam::Pose3 R_pose_A 
      = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(R_artifact_position[0], 
                                                  R_artifact_position[1],
                                                  R_artifact_position[2]));
  // R_pose_A.print("Between pose is \n");

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
    gu::Transform3 new_pose;

    if (!b_is_front_end_){
      // No updates between poses, just get from graph
      new_pose = loop_closure_.GetLastPose();
    } else {
      if (b_use_lo_frontend_){
        new_pose = be_current_pose_;
      } else {
        new_pose = localization_.GetIntegratedEstimate();
      }
      // Delta translation from the loop closure change to the last pose node
      new_pose.translation = new_pose.translation + (new_key_pose.translation - last_key_pose.translation);
      // Delta rotation
      new_pose.rotation = new_pose.rotation*(new_key_pose.rotation*last_key_pose.rotation.Trans());
    }

    // Update localization
    // Also reset the robot's estimated position.
    if (b_use_lo_frontend_){
      be_current_pose_ = new_pose;
      PublishPoseWithLoFrontend();
    } else {
      localization_.SetIntegratedEstimate(new_pose);
      
      // Publish localization pose messages
      ros::Time stamp = loop_closure_.GetTimeAtLastKey();
      localization_.UpdateTimestamp(stamp);
      localization_.PublishPoseNoUpdate();
    }

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
    if (b_use_lo_frontend_){
      be_current_pose_ = loop_closure_.GetLastPose();
      PublishPoseWithLoFrontend();
    } else {
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
      
      // Publish localization pose messages
      ros::Time stamp = loop_closure_.GetTimeAtLastKey();
      localization_.UpdateTimestamp(stamp);
      localization_.PublishPoseNoUpdate();
    }

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
  ROS_INFO("In UWB Callback");
  if (loop_closure_.GetNumberStampsKeyed() > uwb_first_key_threshold_) {
    
    // Store the UWB-related data into the buffer "uwb_id2data_hash_"
    auto itr = uwb_id2data_hash_.find(msg.id);
    if (itr != end(uwb_id2data_hash_)) {
      if (uwb_id2data_hash_[msg.id].drop_status == true) {
        uwb_id2data_hash_[msg.id].range.push_back(msg.range);
        uwb_id2data_hash_[msg.id].time_stamp.push_back(msg.header.stamp);
        uwb_id2data_hash_[msg.id].robot_position.push_back(localization_.GetIntegratedEstimate().translation.Eigen()); // Maybe should use tfs for this? If we are in the middle of a loop closure?
        uwb_id2data_hash_[msg.id].nearest_pose_key.push_back(loop_closure_.GetKeyAtTime(msg.header.stamp));

      }
    } 
    else {
      ROS_WARN("Not registered UWB ID");
    }
  }
  return;
}

void BlamSlam::VisualizationTimerCallback(const ros::TimerEvent& ev) {
  mapper_.PublishMap();
}

void BlamSlam::PoseAndScanFilterCB(const sensor_msgs::PointCloud2ConstPtr &pointCloud, const geometry_msgs::PoseStamped pose) {

    // ROS_INFO("In message filter callback");

    geometry_utils::Transform3 fePose = geometry_utils::ros::FromROS(pose.pose);

    PointCloud::Ptr rxCloudPtr;
    rxCloudPtr.reset(new PointCloud);

    pcl::fromROSMsg(*pointCloud, *rxCloudPtr.get());

    this->ProcessPoseScanMessage(fePose, rxCloudPtr);

    // Publish pose 
    PublishPoseWithLoFrontend();

    // Publish transform between fixed frame and localization frame.
    if (b_publish_tfs_){
      geometry_msgs::TransformStamped tf;
      tf.transform = geometry_utils::ros::ToRosTransform(be_current_pose_);
      tf.header.stamp = pointCloud->header.stamp;
      tf.header.frame_id = fixed_frame_id_;
      tf.child_frame_id = base_frame_id_;
      tfbr_.sendTransform(tf);
    }

    return;
}

void BlamSlam::PublishPoseWithLoFrontend(){
  // Publish pose 
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = geometry_utils::ros::ToRosPose(be_current_pose_);
  ros_pose.header.frame_id = fixed_frame_id_;
  // ros_pose.header.stamp = pointCloud->header.stamp;
  ros_pose.header.stamp = ros::Time::now(); // TO FIX
  pose_pub_.publish(ros_pose);
}

void BlamSlam::ProcessPointCloudMessage(const PointCloud::ConstPtr& msg) {
  // Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  PointCloud::Ptr msg_transformed(new PointCloud);

  // Update odometry by performing ICP.
  if (!odometry_.UpdateEstimate(*msg_filtered)) {
    b_add_first_scan_to_key_ = true;
  }

  if (b_add_first_scan_to_key_) {
    // First update ever.
    // Transforming msg to fixed frame for non-zero initial position
    localization_.TransformPointsToFixedFrame(*msg_filtered,
                                              msg_transformed.get());
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg_transformed, unused.get());
    loop_closure_.AddKeyScanPair(initial_key_, msg, true);

    // Publish localization pose messages
    ros::Time stamp = pcl_conversions::fromPCL(msg->header.stamp);
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();

    // Publish
    loop_closure_.PublishPoseGraph();

    // Revert the first scan to key
    b_add_first_scan_to_key_ = false;
    return;
  }

  // Containers.
  PointCloud::Ptr msg_neighbors(new PointCloud);
  PointCloud::Ptr msg_base(new PointCloud);
  PointCloud::Ptr msg_fixed(new PointCloud);

  // Transform the incoming point cloud to the best estimate of the base frame.
  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg_filtered,
                                            msg_transformed.get());

  // Get approximate nearest neighbors from the map.
  mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

  // Transform those nearest neighbors back into sensor frame to perform ICP.
  localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

  // Localize to the map. Localization will output a pointcloud aligned in the
  // sensor frame.
  localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

  // Check for new loop closures.
  bool new_keyframe = false;
  if (HandleLoopClosures(msg, &new_keyframe)) {
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());

    // Publish artifacts - should be updated from the pose-graph 
    loop_closure_.PublishArtifacts();

  } else {
    // ROS_INFO("No new loop closures");
    // No new loop closures - but was there a new key frame? If so, add new
    // points to the map.
    if (new_keyframe) {
      localization_.MotionUpdate(gu::Transform3::Identity());
      localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(msg_fixed, unused.get());

      // Also reset the robot's estimated position.
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
    }
  }

  // Also reset the robot's estimated position.
  if (b_use_lo_frontend_){
    be_current_pose_ = loop_closure_.GetLastPose();
    PublishPoseWithLoFrontend();
  } else {
    // Publish localization pose messages
    ros::Time stamp = pcl_conversions::fromPCL(msg->header.stamp);
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
  }

  // Only publish the pose-graph if there is a new keyframe TODO consider changing to publishing for each new node 
  if (new_keyframe){
    // Visualize the pose graph and current loop closure radius.
    loop_closure_.PublishPoseGraph();
  }   
  

  // Publish the incoming point cloud message from the base frame.
  if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }

}

void BlamSlam::ProcessPoseScanMessage(geometry_utils::Transform3& fe_pose, const PointCloud::Ptr& scan) {
  // ROS_INFO("Inside processPoseScanMessage");
  
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

  // Also reset the robot's estimated position.
  if (b_use_lo_frontend_){
    PublishPoseWithLoFrontend();
  }

  // Only publish the pose-graph if there is a new keyframe TODO consider changing to publishing for each new node 
  if (new_keyframe){
    // Visualize the pose graph and current loop closure radius.
    loop_closure_.PublishPoseGraph();

    // Publish the map
    mapper_.PublishMap();
  }   

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

  // Bool for adding scan to key the pose added at restart
  b_add_first_scan_to_key_ = true;

    // Also reset the robot's estimated position.
    if (b_use_lo_frontend_){
      be_current_pose_ = loop_closure_.GetLastPose();
      PublishPoseWithLoFrontend();
    } else {
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
      
      // Publish localization pose messages
      ros::Time stamp = loop_closure_.GetTimeAtLastKey();
      localization_.UpdateTimestamp(stamp);
      localization_.PublishPoseNoUpdate(); // Maybe need to update the timestamp here
    }
  return true;
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan,
                                  bool* new_keyframe) {
  if (new_keyframe == NULL) {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  gtsam::Symbol pose_key;
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

  // Add between factor
  if (!loop_closure_.AddBetweenFactor(localization_.GetIncrementalEstimate(),
                                      covariance,
                                      stamp,
                                      &pose_key)) {
    return false;
  }

  *new_keyframe = true;

  if (!loop_closure_.AddKeyScanPair(pose_key, scan, false)) {
    return false;
  }

  std::vector<gtsam::Symbol> closure_keys;
  if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys)) {
    return false;
  }

  for (const auto& closure_key : closure_keys) {
    ROS_INFO("%s: Closed loop between poses %u and %u.", name_.c_str(),
             pose_key, closure_key);
  }
  return true;
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan, geometry_utils::Transform3 pose_delta,
                                  bool* new_keyframe) {
  if (new_keyframe == NULL) {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  gtsam::Symbol pose_key;
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

  std::vector<gtsam::Symbol> closure_keys;
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

std::string BlamSlam::getRobotName(const ros::NodeHandle& n) {
  std::string str_ns = n.getNamespace();
  int index_start;
  int index_end;
  int count = 0;
  for (int i=0; i<str_ns.size(); i++) {
    if (str_ns.substr(i,1) == "/") {
      if (count == 0) {index_start = i+1; count++;}
      else if (count == 1) {index_end = i-1;}
    }
  }
  int str_ns_length = index_end - index_start + 1;
  return str_ns.substr(index_start, str_ns_length);
}

// BASE STATION
void BlamSlam::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr &msg) {
  ROS_INFO_STREAM("Keyed scan message recieved");

  // Access loop closure callback
  loop_closure_.KeyedScanBaseHandler(msg);
}

void BlamSlam::PoseGraphCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr &msg) {
  ROS_INFO_STREAM("Pose Graph message recieved");
  clock_t start, fstart, cur_time;
	start = clock();  

  // Access loop closure callback
  bool found_loop;
  loop_closure_.PoseGraphBaseHandler(msg, &found_loop);

  // Update map
  if (found_loop) {
    ROS_INFO("Found Loop closure, regenerating the map");
    // Found loop closure - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    fstart = clock();
    mapper_.InsertPoints(regenerated_map, unused.get());
  }
  else {
    ROS_INFO("No Loop closure, updating the map");
    // No loop closure - add latest keyed scan to map only.
    PointCloud::Ptr incremental_map(new PointCloud);
    // gtsam::Symbol key_points = GetKeyAtTime(msg->header.stamp); // Get the latest 
    // GetLatestPointsFromKey(incremental_map.get(), key_points);
    fstart = clock();
    
    if (loop_closure_.GetLatestPoints(incremental_map.get())){
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(incremental_map, unused.get());
    }

  }
  cur_time = clock() - fstart;
  ROS_INFO_STREAM("InsertPoints completed in " << (float)cur_time/CLOCKS_PER_SEC << " seconds");

  // Also reset the robot's estimated position.
  localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
  ros::Time stamp = msg->header.stamp;
  localization_.UpdateTimestamp(stamp);
  localization_.PublishPoseNoUpdate();
  b_new_pose_available_ = true;
  // Publish Graph
  // loop_closure_.PublishPoseGraph();
  // loop_closure_.PublishArtifacts();

  // Publish map
  mapper_.PublishMap();
	cur_time = clock() - start;
  ROS_INFO_STREAM("PoseGraphCallback completed in " << (float)cur_time/CLOCKS_PER_SEC << " seconds");
}

void BlamSlam::ArtifactBaseCallback(const core_msgs::Artifact::ConstPtr& msg) {
  ROS_INFO_STREAM("Artifact message recieved");
  core_msgs::Artifact artifact;
  artifact.header = msg->header;
  artifact.name = msg->name;
  artifact.parent_id = msg->parent_id;
  artifact.seq = msg->seq;
  artifact.hotspot_name = msg->hotspot_name;
  artifact.point = msg->point;
  artifact.covariance = msg->covariance;
  artifact.confidence = msg->confidence;
  artifact.label = msg->label;
  artifact.thumbnail = msg->thumbnail;

  ArtifactInfo artifactinfo(msg->parent_id);
  artifactinfo.msg = artifact;

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

  // Publish artifacts - should be updated from the pose-graph
  loop_closure_.PublishArtifacts();
}

void BlamSlam::PoseUpdateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

  // // ROS_INFO("new pose update received from frontend");

  // // Set the pose estimate to the latest pose graph node if there is a new one
  // if (b_new_pose_available_) {
  //   current_pose_est_ = loop_closure_.GetLastPose();
  //   b_new_pose_available_ = false;
  // }

  // // update with the pose delta
  // current_pose_est_ = gu::PoseUpdate(current_pose_est_, gu::ros::FromROS(msg->pose));
  // localization_.SetIntegratedEstimate(current_pose_est_);

  // // Update the timestamp
  // ros::Time stamp = msg->header.stamp;
  // localization_.UpdateTimestamp(stamp);

  // // Publish the updated pose
  // localization_.PublishPoseNoUpdate();
}


