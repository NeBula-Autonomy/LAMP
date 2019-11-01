/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_ROBOT_H
#define LAMP_ROBOT_H

// Includes
#include <lamp/LampBase.h>

#include <factor_handlers/ArtifactHandler.h>
#include <factor_handlers/AprilTagHandler.h>
#include <factor_handlers/OdometryHandler.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
// Services

// Class Definition
class LampRobot : public LampBase {
public:
  // Constructor
  LampRobot();

  // Destructor
  ~LampRobot();

  // Override base class functions where needed
  virtual bool Initialize(const ros::NodeHandle& n);

  gtsam::Symbol GetInitialKey() {
    return initial_key_;
  };
  gtsam::Symbol GetCurrentKey() {
    return pose_graph_.key;
  };

protected:
  // instantiate all handlers that are being used in the derived classes
  virtual bool InitializeHandlers(const ros::NodeHandle& n);

  // load parameters from yaml files
  virtual bool LoadParameters(const ros::NodeHandle& n);

  // retrieve data from all handlers
  virtual bool CheckHandlers(); // - inside timed callback
  // TODO consider checking handlers at different frequencies

  bool RegisterCallbacks(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  // Main update timer callback
  virtual void ProcessTimerCallback(const ros::TimerEvent& ev);

  // Initialization helper functions
  bool SetInitialPosition();
  bool SetInitialKey();

  void UpdateArtifactPositions();
  void UpdateAndPublishOdom();

  // Publishers
  ros::Publisher pose_pub_;

private:
  // Overwrite base classs functions where needed

    // Factor Hanlder Wrappers
    bool ProcessOdomData(FactorData* data);
    // Process the artifact factors if any available
    bool ProcessArtifactData(FactorData* data);
    // Process the april tag factors if any available    
    bool ProcessAprilTagData(FactorData* data);
    bool InitializeGraph(gtsam::Pose3& pose, 
                         gtsam::noiseModel::Diagonal::shared_ptr& covariance);
    // void ProcessUWBData(FactorData data);
    // Example use:
    // ProcessArtifactData(artifact_handler_.GetData());

  void HandleRelativePoseMeasurement(const ros::Time& time,
                                     const gtsam::Pose3& relative_pose,
                                     gtsam::Pose3& transform,
                                     gtsam::Pose3& global_pose,
                                     gtsam::Symbol& key_from);

  bool ConvertGlobalToRelative(const ros::Time stamp,
                               const gtsam::Pose3 pose_global,
                               gtsam::Pose3& pose_relative);

    // Data Handler classes
    OdometryHandler odometry_handler_; 
    ArtifactHandler artifact_handler_;
    AprilTagHandler april_tag_handler_;
    // Manual LC
    // IMU
    // TS
    // LoopClosure

  // Add new functions as needed

  // Add new variables as needed

  // Parameters
  gtsam::Vector6 initial_noise_;
  bool b_artifacts_in_global_;

  // Test class fixtures
  friend class TestLampRobot;

  struct Parameters {
    // Apply a voxel grid filter.
    bool grid_filter;

    // Resolution of voxel grid filter.
    double grid_res;

    // Apply a random downsampling filter.
    bool random_filter;

    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;
  } params_;
};

#endif
