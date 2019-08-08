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

#ifndef BLAM_SLAM_H
#define BLAM_SLAM_H

#include <ros/ros.h>

#include <blam_slam/AddFactor.h>
#include <blam_slam/RemoveFactor.h>
#include <blam_slam/SaveGraph.h>
#include <blam_slam/Restart.h>
#include <blam_slam/LoadGraph.h>
#include <blam_slam/BatchLoopClosure.h>

#include <measurement_synchronizer/MeasurementSynchronizer.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <laser_loop_closure/LaserLoopClosure.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_mapper/PointCloudMapper.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <core_msgs/Artifact.h>
#include <core_msgs/PoseAndScan.h>
#include <uwb_msgs/Anchor.h>
#include <mesh_msgs/ProcessCommNode.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

class BlamSlam {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  BlamSlam();
  ~BlamSlam();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  // The from_log argument specifies whether to run SLAM online (subscribe to
  // topics) or by loading messages from a bag file.
  bool Initialize(const ros::NodeHandle& n, bool from_log);

  // Sensor message processing.
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);
  void ProcessPoseScanMessage(geometry_utils::Transform3& fe_pose, const PointCloud::Ptr& scan);

  // UWB range measurement data processing
  void ProcessUwbRangeData(const std::string uwb_id);

  // Transform a point cloud from the sensor frame into the fixed frame using
  // the current best position estimate.
  bool TransformPointsToFixedFrame(const PointCloud& points,
                                   PointCloud* points_transformed) const;  

  int marker_id_;
  
  //listener for tf published by fiducials
  tf::TransformListener tf_listener_;

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Sensor callbacks.
  void PoseScanCallback(const core_msgs::PoseAndScanConstPtr& msg);
  void PointCloudCallback(const PointCloud::ConstPtr& msg);
  void TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                             const sensor_msgs::PointCloud2::ConstPtr& pcld2);
  void ArtifactCallback(const core_msgs::Artifact& msg);
  void UwbSignalCallback(const uwb_msgs::Anchor& msg);
  void RepubPoseGraphCallback(const std_msgs::Empty& msg);

  // Timer callbacks.
  void EstimateTimerCallback(const ros::TimerEvent& ev);
  void VisualizationTimerCallback(const ros::TimerEvent& ev);
  void UwbTimerCallback(const ros::TimerEvent& ev);

  void PoseAndScanFilterCB(const sensor_msgs::PointCloud2ConstPtr& pointCloud, const geometry_msgs::PoseStamped pose);
  // Base Station Callbacks
  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr &msg);
  void PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr &msg);
  void ArtifactBaseCallback(const core_msgs::Artifact::ConstPtr& msg);
  void PoseUpdateCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // Loop closing. Returns true if at least one loop closure was found. Also
  // output whether or not a new keyframe was added to the pose graph.
  bool HandleLoopClosures(const PointCloud::ConstPtr& scan, bool* new_keyframe);
  bool HandleLoopClosures(const PointCloud::ConstPtr& scan, geometry_utils::Transform3 pose_delta, bool* new_keyframe);

  // Generic add Factor service - for human loop closures to start
  bool AddFactorService(blam_slam::AddFactorRequest &request,
                        blam_slam::AddFactorResponse &response);
  // Generic remove Factor service - removes edges from pose graph
  bool RemoveFactorService(blam_slam::RemoveFactorRequest &request,
                           blam_slam::RemoveFactorResponse &response);

  // Service for restarting from last saved posegraph
  bool RestartService(blam_slam::RestartRequest &request,
                        blam_slam::RestartResponse &response);
  // Drop UWB from a robot
  bool DropUwbService(mesh_msgs::ProcessCommNodeRequest &request,
                      mesh_msgs::ProcessCommNodeResponse &response);
  
  // Clear UWB buffer
  void UwbClearBuffer(const std::string uwb_id);

  // Service for rinning lazer loop closure again
  bool BatchLoopClosureService(blam_slam::BatchLoopClosureRequest &request,
                        blam_slam::BatchLoopClosureResponse &response);

  bool use_chordal_factor_;

  // Service to write the pose graph and all point clouds to a zip file.
  bool SaveGraphService(blam_slam::SaveGraphRequest &request,
                        blam_slam::SaveGraphResponse &response);

  bool LoadGraphService(blam_slam::LoadGraphRequest &request,
                        blam_slam::LoadGraphResponse &response);                      

  // Publish Artifacts
  void PublishArtifact(const Eigen::Vector3d& W_artifact_position,
                       const core_msgs::Artifact& msg);

  // Send signal to republish the pose graph from the front end
  void SendRepubPoseGraphFlag();  
  
// Publish pose when using LO Frontend
  void PublishPoseWithLoFrontend();

  bool getTransformEigenFromTF(const std::string& parent_frame,
                               const std::string& child_frame,
                               const ros::Time& time,
                               Eigen::Affine3d& T);
  
  std::string getRobotName(const ros::NodeHandle& n);

  // The node's name.
  std::string name_;
  std::string robot_name_;

  std::string blam_frame_;
  std::string world_frame_;

  // The intial key in the pose graph
  gtsam::Symbol initial_key_;

  // The delta between where LAMP was last saved, and where it is restarted.
  geometry_utils::Transform3 delta_after_restart_;

  geometry_utils::Transform3 delta_after_load_;

  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;
  double uwb_update_rate_;
  ros::Timer estimate_update_timer_;
  ros::Timer visualization_update_timer_;
  ros::Timer uwb_update_timer_;

  // Covariances
  double position_sigma_;
  double attitude_sigma_;

  // Subscribers.
  ros::Subscriber pose_scan_sub_;
  ros::Subscriber pcld_sub_;
  ros::Subscriber artifact_sub_;
  ros::Subscriber uwb_sub_;
  ros::Subscriber repub_pg_sub_;
  std::vector<ros::Subscriber> Subscriber_posegraphList_;
  std::vector<ros::Subscriber> Subscriber_keyedscanList_;
  std::vector<ros::Subscriber> Subscriber_artifactList_;
  std::vector<ros::Subscriber> Subscriber_poseList_;

  // Whether to use two point clouds.
  bool use_two_vlps_{false};
  // Queue size of the approximate time policy that synchronizes the two point clouds.
  int pcld_queue_size_{10};
  // Filters
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld2_sub_;
  // Synchronization policy for the two point clouds.
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PcldSyncPolicy;
  // Synchronizer for the two point clouds.
  typedef message_filters::Synchronizer<PcldSyncPolicy> PcldSynchronizer;
  std::unique_ptr<PcldSynchronizer> pcld_synchronizer;

  // Publishers
  ros::Publisher base_frame_pcld_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher repub_pg_sig_pub_;

  // Transform broadcasting to other nodes.
  tf2_ros::TransformBroadcaster tfbr_;

  double restart_x_;
  double restart_y_;
  double restart_z_;
  double restart_roll_;
  double restart_pitch_;
  double restart_yaw_;

  // GT AprilTag world coordinates
  double aprilTag4_x_;
  double aprilTag4_y_;
  double aprilTag4_z_;
  double aprilTag6_x_;
  double aprilTag6_y_;
  double aprilTag6_z_;
  double aprilTag26_x_;
  double aprilTag26_y_;
  double aprilTag26_z_;
  
  // Artifact prefix
  unsigned char artifact_prefix_;

  // Services
  ros::ServiceServer add_factor_srv_;
  ros::ServiceServer remove_factor_srv_;
  ros::ServiceServer save_graph_srv_;
  ros::ServiceServer restart_srv_;
  ros::ServiceServer load_graph_srv_;
  ros::ServiceServer batch_loop_closure_srv_;
  ros::ServiceServer drop_uwb_srv_;

  // Names of coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;
  bool artifacts_in_global_;
  int largest_artifact_id_; 
  bool use_artifact_loop_closure_;
  bool b_use_uwb_;
  bool b_add_first_scan_to_key_;

  //Basestation/back-end/mid-end
  bool b_is_basestation_;
  std::vector<std::string> robot_names_;
  bool b_is_front_end_;
  bool b_use_lo_frontend_{false}; 

  bool b_publish_tfs_{false};

  // Pose updating
  bool b_new_pose_available_;
  geometry_utils::Transform3 current_pose_est_;

  // Object IDs
  std::unordered_map<std::string, gtsam::Key> artifact_id2key_hash;

  // UWB
  bool b_uwb_test_data_collection_;
  unsigned int uwb_skip_measurement_number_;
  unsigned int uwb_update_key_number_;
  unsigned int uwb_required_key_number_first_;
  unsigned int uwb_required_key_number_not_first_;
  unsigned int uwb_first_key_threshold_;
  std::map<std::string, UwbMeasurementInfo> uwb_id2data_hash_;

  // Class objects (BlamSlam is a composite class).
  MeasurementSynchronizer synchronizer_;
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  LaserLoopClosure loop_closure_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;

  // Pose handler
  geometry_utils::Transform3 fe_last_pose_; // Note that this never touches loop closure updates
  bool b_first_pose_scan_revieved_;

  geometry_utils::Transform3 be_current_pose_;
  
  // Pose and Scan filters
  message_filters::Subscriber<sensor_msgs::PointCloud2>* filterPointSub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* filterPoseSub_;
  message_filters::Synchronizer
      <
          message_filters::sync_policies::ApproximateTime
          <
            sensor_msgs::PointCloud2,
            geometry_msgs::PoseStamped
          >
      >* poseScanSync_;

};

#endif
