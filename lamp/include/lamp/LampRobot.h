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
#include <factor_handlers/OdometryHandler.h>
#include <factor_handlers/StationaryHandler.h>

#include <pcl_ros/point_cloud.h>
#include <lamp_utils/LampPcldFilter.h>
// Services

// Class Definition
class LampRobot : public LampBase {
 public:
  // Constructor
  LampRobot();

  // Destructor
  ~LampRobot();

  // Override base class functions where needed
  bool Initialize(const ros::NodeHandle& n) override;

  inline gtsam::Symbol GetInitialKey() const {
    return pose_graph_.initial_key;
  };
  inline gtsam::Symbol GetCurrentKey() const { return pose_graph_.key; };

 protected:
  // instantiate all handlers that are being used in the derived classes
  bool InitializeHandlers(const ros::NodeHandle& n) override;

  // load parameters from yaml files
  bool LoadParameters(const ros::NodeHandle& n) override;

  // retrieve data from all handlers
  bool CheckHandlers() override;  // - inside timed callback
  // TODO consider checking handlers at different frequencies

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  // Main update timer callback
  void ProcessTimerCallback(const ros::TimerEvent& ev) override;

  // Initialization helper functions
  bool SetInitialPosition();
  bool SetInitialKey();

  void UpdateArtifactPositions() override;
  void UpdateAndPublishOdom();

  // Publishers
  ros::Publisher pose_pub_;

  // Flag for artifact initialization
  bool is_artifact_initialized;

 private:
  // Overwrite base classs functions where needed

   // Factor Handler Wrappers
   bool ProcessOdomData(std::shared_ptr<FactorData> data);
   // Process the artifact factors if any available
   bool ProcessArtifactData(std::shared_ptr<FactorData> data);

   // Process Stationary data when robot stops
   bool ProcessStationaryData(std::shared_ptr<FactorData> data);

   bool InitializeGraph(gtsam::Pose3& pose,
                        gtsam::noiseModel::Diagonal::shared_ptr& covariance);
   // Example use:
   // ProcessArtifactData(artifact_handler_.GetData());
   void AddKeyedScanAndPublish(PointCloud::Ptr new_scan,
                               gtsam::Symbol current_key);

   void HandleRelativePoseMeasurement(const ros::Time& time,
                                      const gtsam::Pose3& relative_pose,
                                      gtsam::Pose3& transform,
                                      gtsam::Pose3& global_pose,
                                      gtsam::Symbol& key_from);

   void
   HandleRelativePoseMeasurementWithFixedKey(const ros::Time& time,
                                             const gtsam::Pose3& relative_pose,
                                             const gtsam::Symbol& key_from,
                                             gtsam::Pose3& transform,
                                             gtsam::Pose3& global_pose);

   bool ConvertGlobalToRelative(const ros::Time stamp,
                                const gtsam::Pose3 pose_global,
                                gtsam::Pose3& pose_relative);

   // Data Handler classes
   OdometryHandler odometry_handler_;
   ArtifactHandler artifact_handler_;
   StationaryHandler stationary_handler_;

   // Add new functions as needed

   // Add new variables as needed

   // Parameters
   gtsam::Vector6 initial_noise_;
   bool b_artifacts_in_global_;
   bool b_add_imu_factors_;
   bool b_init_pg_pub_;
   int factor_count_;
   int init_count_;
   float init_wait_time_;
   float repub_first_wait_time_;

   // Test class fixtures
   friend class TestLampRobot;
   friend class TestLampRobotArtifact;

   // Point cloud filter
   LampPcldFilter filter_;
   LampPcldFilterParams filter_params_;
};

#endif
