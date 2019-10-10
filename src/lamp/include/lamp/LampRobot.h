/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_ROBOT_H
#define LAMP_ROBOT_H

// Includes 
#include <lamp/LampBase.h>

#include <factor_handlers/OdometryHandler.h>
#include <pcl/common/transforms.h>

// Services

// Class Definition
class LampRobot : public LampBase {

  public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // Constructor
    LampRobot();

    // Destructor
    ~LampRobot();    

    // Override base class functions where needed 
    virtual bool Initialize(const ros::NodeHandle& n);

    gtsam::Symbol GetInitialKey() {return initial_key_;};
    gtsam::Symbol GetCurrentKey() {return key_;};

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

    // Generate map from keyed scans
    bool GenerateMapPointCloud();
    bool AddTransformedPointCloudToMap(gtsam::Symbol key);


    PointCloudFilter filter_;
    PointCloudMapper mapper_;

    // Publishers
    ros::Publisher pose_pub_;

  private:
    // Overwrite base classs functions where needed


    // Factor Hanlder Wrappers
    bool ProcessOdomData(FactorData data);
    bool ProcessArtifactData(FactorData data);
    void ProcessAprilData(FactorData data);
    bool InitializeGraph(gtsam::Pose3& pose, gtsam::noiseModel::Diagonal::shared_ptr& covariance);
    // void ProcessUWBData(FactorData data);
    // Example use:
    // ProcessArtifactData(artifact_handler_.GetData());

    void HandleRelativePoseMeasurement(const ros::Time& time,
                                       const gtsam::Pose3& relative_pose,
                                       gtsam::Pose3& transform,
                                       gtsam::Pose3& global_pose,
                                       gtsam::Symbol& key_from);

    // Data Handler classes
    OdometryHandler odometry_handler_; 
    // ArtifactHandler artifact_handler_;
    // AprilHandler april_handler_;
    // Manual LC
    // IMU
    // TS
    // LoopClosure



    // Add new functions as needed


    // Add new variables as needed


    // Parameters
    gtsam::Vector6 initial_noise_;


    // Test class fixtures
    friend class TestLampRobot;

};


#endif
