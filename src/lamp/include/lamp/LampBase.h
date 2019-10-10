/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_H
#define LAMP_BASE_H

// Includes
#include <ros/ros.h>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
// #include <tf2_ros/transform_broadcaster.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

// #include <core_msgs/Artifact.h>
// #include <core_msgs/PoseAndScan.h>
// #include <uwb_msgs/Anchor.h>
// #include <mesh_msgs/ProcessCommNode.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_ros/point_cloud.h>

#include <factor_handlers/LampDataHandlerBase.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <pose_graph_merger/merger.h>

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

#include <math.h>

// TODO - review and make pure-virtual all functions that are not implemented
// here

// Services

// Class definition
class LampBase {
 public:
   typedef std::vector<pose_graph_msgs::PoseGraphEdge> EdgeMessages;
   typedef std::vector<pose_graph_msgs::PoseGraphNode> PriorMessages;
   // Constructor
   LampBase();

   // Destructor
   ~LampBase();

   // Define main interface functions

   virtual bool Initialize(const ros::NodeHandle& n);

 protected:
  // TODO: make most of these pure virtual

  // Use this for any "private" things to be used in the derived class
  // Node initialization.
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Set precisions for fixed covariance settings
  bool SetFactorPrecisions();

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  // instantiate all handlers that are being used in the derived classes
  virtual bool InitializeHandlers(const ros::NodeHandle& n) = 0;

  // Main update timer callback
  virtual void ProcessTimerCallback(const ros::TimerEvent& ev) = 0;
  double update_rate_;
  ros::Timer update_timer_;

  // retrieve data from all handlers
  virtual bool CheckHandlers();

  // Callback for loop closures
  void LaserLoopClosureCallback(const pose_graph_msgs::PoseGraphConstPtr msg);
  void AddLoopClosureToGraph(const pose_graph_msgs::PoseGraphConstPtr msg);
  pose_graph_msgs::PoseGraph
  ChangeCovarianceInMessage(pose_graph_msgs::PoseGraph msg,
                            gtsam::SharedNoiseModel noise);

  // Functions to publish
  bool PublishPoseGraph();
  bool PublishPoseGraphForOptimizer();

  // Convert timestamps to gtsam keys 
  gtsam::Symbol GetKeyAtTime(const ros::Time& stamp) const;
  gtsam::Symbol GetClosestKeyAtTime(const ros::Time& stamp) const;
  bool IsTimeWithinThreshold(double time, const ros::Time& target) const;

  // Convert values to PoseGraphNode Messages
  bool
  ConvertValuesToNodeMsgs(gtsam::Values values,
                          std::vector<pose_graph_msgs::PoseGraphNode>& nodes);

  // Convert internal pose graph to message
  pose_graph_msgs::PoseGraphConstPtr ConvertPoseGraphToMsg(
      gtsam::Values values, EdgeMessages edges_info, PriorMessages priors_info);

  // Placeholder for setting fixed noise
  gtsam::SharedNoiseModel SetFixedNoiseModels(std::string type);
  gtsam::SharedNoiseModel laser_lc_noise_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel artifact_noise_;

  // New pose graph values from optimizer
  virtual void
  OptimizerUpdateCallback(const pose_graph_msgs::PoseGraphConstPtr& msg);

  // Tracking info for publishing messages
  void AddNewValues(gtsam::Values new_values);
  void TrackEdges(gtsam::Symbol key_from,
                  gtsam::Symbol key_to,
                  int type,
                  gtsam::Pose3 pose,
                  gtsam::SharedNoiseModel covariance);
  void TrackPriors(ros::Time stamp,
                   gtsam::Symbol key,
                   gtsam::Pose3 pose,
                   gtsam::SharedNoiseModel covariance);

  // Typedef for stored point clouds.
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  // Variables - can be able to be accessed in the derived class
  gtsam::NonlinearFactorGraph nfg_;
  gtsam::Values values_;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans_;
  std::map<gtsam::Symbol, ros::Time> keyed_stamps_;  // All nodes
  std::map<double, gtsam::Symbol> stamp_to_odom_key_;

    // List of all factors with additional information
  EdgeMessages edges_info_;
  PriorMessages priors_info_;

  // Variables for tracking the new features only
  gtsam::Values values_new_;
  EdgeMessages edges_info_new_;
  PriorMessages priors_info_new_;

  // Publishers
  ros::Publisher pose_graph_pub_;
  ros::Publisher pose_graph_incremental_pub_;
  ros::Publisher pose_graph_to_optimize_pub_;
  ros::Publisher keyed_scan_pub_;

  // Subscribers
  ros::Subscriber back_end_pose_graph_sub_;
  ros::Subscriber laser_loop_closure_sub_;

  // Services

  // Message filters (if any)
  std::string prefix_;

  // Initial key
  gtsam::Symbol initial_key_;

  // Current key
  gtsam::Symbol key_;

  // Main process name
  std::string name_;

  // Booleans
  bool b_has_new_factor_;
  bool b_run_optimization_;
  bool b_use_fixed_covariances_;

  // Frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Pose graph merger
  Merger merger_;

  // Precisions
  double attitude_sigma_;
  double position_sigma_;
  double manual_lc_rot_precision_;
  double manual_lc_trans_precision_;
  double artifact_rot_precision_;
  double artifact_trans_precision_;
  double fiducial_trans_precision_;
  double fiducial_rot_precision_;
  double point_estimate_precision_;
  double laser_lc_rot_sigma_;
  double laser_lc_trans_sigma_;

  // Time threshold for GetKeyAtTime
  double time_threshold_; 

 private:
  // Anything just in the base class



};

#endif