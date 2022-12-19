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

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>

#include <factor_handlers/LampDataHandlerBase.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <pose_graph_merger/merger.h>

#include <lamp_utils/CommonFunctions.h>
#include <lamp_utils/CommonStructs.h>
#include <lamp_utils/PoseGraph.h>
#include <lamp_utils/PrefixHandling.h>

#include <math.h>

// Services

// Class definition
class LampBase {
public:
  // Constructor
  LampBase();

  // Destructor
  ~LampBase();

  // Define main interface functions

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  // Pose graph getters for outside modules (e.g. test fixtures).
  inline const PoseGraph& graph() const {
    return pose_graph_;
  }
  inline PoseGraph& graph() {
    return pose_graph_;
  }

protected:
  // Use this for any "private" things to be used in the derived class
  // Node initialization.
  virtual bool LoadParameters(const ros::NodeHandle& n) = 0;

  // Set precisions for fixed covariance settings
  bool SetFactorPrecisions();

  // Use this for any "private" things to be used in the derived class
  // Node initialization.
  // Set precisions for fixed covariance settings
  virtual bool CreatePublishers(const ros::NodeHandle& n);
  virtual bool RegisterCallbacks(const ros::NodeHandle& n) = 0;
  virtual bool InitializeHandlers(const ros::NodeHandle& n) = 0;
  virtual void ProcessTimerCallback(const ros::TimerEvent& ev) = 0;
  double update_rate_;
  ros::Timer update_timer_;

  // retrieve data from all handlers
  virtual bool CheckHandlers() = 0;

  // Callback for loop closures
  void LaserLoopClosureCallback(const pose_graph_msgs::PoseGraphConstPtr msg);
  void AddLoopClosureToGraph(const pose_graph_msgs::PoseGraphConstPtr msg);
  pose_graph_msgs::PoseGraph
  ChangeCovarianceInMessage(pose_graph_msgs::PoseGraph msg,
                            gtsam::SharedNoiseModel noise);

  // Functions to publish
  bool PublishPoseGraph(bool b_publish_incremental = true);
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
  gtsam::SharedNoiseModel point_estimate_noise_;

  // Zero noise param
  gtsam::noiseModel::Diagonal::shared_ptr zero_covariance_;

  // New pose graph values from optimizer
  void OptimizerUpdateCallback(const pose_graph_msgs::PoseGraphConstPtr& msg);
  void MergeOptimizedGraph(const pose_graph_msgs::PoseGraphConstPtr& msg);

  void PublishAllKeyedScans();

  // Pose graph structure storing values, factors and meta data.
  PoseGraph pose_graph_;

  // Function used for retrieving internal identifier given gtsam::Symbol.
  std::string MapSymbolToId(gtsam::Symbol key) const;

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
  bool b_received_optimizer_update_;
  bool b_use_fixed_covariances_;
  bool b_repub_values_after_optimization_;
  bool b_have_received_first_pg_{false};

  // Frames.
  std::string base_frame_id_;

  // Pose graph merger
  Merger merger_;

  // Mapper
  IPointCloudMapper::Ptr mapper_;

  // Precisions
  double attitude_sigma_;
  double position_sigma_;
  double manual_lc_rot_precision_;
  double manual_lc_trans_precision_;
  double fiducial_trans_precision_;
  double fiducial_rot_precision_;
  double point_estimate_precision_;
  double laser_lc_rot_sigma_;
  double laser_lc_trans_sigma_;

  double zero_noise_;

private:
  // Anything just in the base class
};

#endif
