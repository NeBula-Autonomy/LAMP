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
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/inference/Symbol.h>

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

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

#include <factor_handlers/LampDataHandlerBase.h>

#include <utils/CommonStructs.h>

// TODO - review and make pure-virtual all functions that are not implemented here

// Services

// Class definition
class LampBase {
  public:

    // Constructor
    LampBase();

    // Destructor
    ~LampBase();

    // Define main interface functions

    virtual bool Initialize();

  protected:

    // Use this for any "private" things to be used in the derived class
    // Node initialization.
    virtual bool LoadParameters(const ros::NodeHandle& n) = 0;
    // bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
    // bool RegisterLogCallbacks(const ros::NodeHandle& n);
    virtual bool RegisterOnlineCallbacks(const ros::NodeHandle& n) = 0;
    virtual bool CreatePublishers(const ros::NodeHandle& n) = 0;
    
    // instantiate all handlers that are being used in the derived classes
    virtual bool InitializeHandlers() = 0; 

    // retrieve data from all handlers
    virtual bool CheckHandlers() = 0; 

    // Functions to publish
    void PublishPoseGraph();

    // Convert timestamps to gtsam keys 
    gtsam::Key getKeyAtTime(const ros::Time& stamp) const;

    // Variables - can be able to be accessed in the derived class
    gtsam::NonlinearFactorGraph nfg_;
    gtsam::Values values_;

    // Keep a list of keyed laser scans and keyed timestamps.
    std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans_;
    std::map<gtsam::Symbol, ros::Time> keyed_stamps_;
    std::map<double, gtsam::Symbol> stamps_keyed_;

    
    // Booleans
    bool b_run_optimization_;

    // Publishers

    // Subscribers

    // Services 

    // Message filters (if any)

  private:
    // Anything just in the base class

};

#endif