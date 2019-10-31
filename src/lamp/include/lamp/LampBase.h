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
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <factor_handlers/LampDataHandlerBase.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <pose_graph_merger/merger.h>

#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>
#include <utils/PrefixHandling.h>

#include <math.h>

// TODO - review and make pure-virtual all functions that are not implemented
// here

// Services

// Class definition
class LampBase {
public:
  // Constructor
  LampBase();

  // Destructor
  ~LampBase();

  // Define main interface functions

  virtual bool Initialize(const ros::NodeHandle& n);

  // Pose graph getters for outside modules (e.g. test fixtures).
  inline const PoseGraph& graph() const {
    return pose_graph_;
  }
  inline PoseGraph& graph() {
    return pose_graph_;
  }

protected:
  // TODO: make most of these pure virtual

  // Use this for any "private" things to be used in the derived class
  // Node initialization.
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // Set precisions for fixed covariance settings
  bool SetFactorPrecisions();

  // Use this for any "private" things to be used in the derived class
  // Node initialization.
  // Set precisions for fixed covariance settings
  virtual bool CreatePublishers(const ros::NodeHandle& n);
  virtual bool InitializeHandlers(const ros::NodeHandle& n) = 0;
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

  // Generate map from keyed scans
  bool ReGenerateMapPointCloud();
  bool CombineKeyedScansWorld(PointCloud* points);
  bool GetTransformedPointCloudWorld(const gtsam::Symbol key,
                                     PointCloud* points);
  bool AddTransformedPointCloudToMap(const gtsam::Symbol key);

  // Placeholder for setting fixed noise
  gtsam::SharedNoiseModel SetFixedNoiseModels(std::string type);
  gtsam::SharedNoiseModel laser_lc_noise_;
  gtsam::SharedNoiseModel odom_noise_;
  gtsam::SharedNoiseModel artifact_noise_;

  // New pose graph values from optimizer
  virtual void
  OptimizerUpdateCallback(const pose_graph_msgs::PoseGraphConstPtr& msg);
  virtual void MergeOptimizedGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);

  // Set artifact positions
  virtual void UpdateArtifactPositions();

  // Pose graph structure storing values, factors and meta data.
  PoseGraph pose_graph_;
  gtsam::Symbol initial_key_{0};

  // Function used for retrieving internal identifier given gtsam::Symbol.
  virtual std::string MapSymbolToId(gtsam::Symbol key) const;

  // Publishers
  ros::Publisher pose_graph_pub_;
  ros::Publisher pose_graph_incremental_pub_;
  ros::Publisher pose_graph_to_optimize_pub_;
  ros::Publisher keyed_scan_pub_;

  // Subscribers
  ros::Subscriber back_end_pose_graph_sub_;
  ros::Subscriber laser_loop_closure_sub_;

  // Services

  // Main process name
  std::string name_;

  // Booleans
  bool b_has_new_factor_;
  bool b_has_new_scan_;
  bool b_run_optimization_;
  bool b_use_fixed_covariances_;
  bool b_repub_values_after_optimization_;

  // Frames.
  std::string base_frame_id_;

  // Pose graph merger
  Merger merger_;

  // Point cloud filter
  PointCloudFilter filter_;

  // Mapper
  PointCloudMapper mapper_;

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

private:
  // Anything just in the base class
};

#endif