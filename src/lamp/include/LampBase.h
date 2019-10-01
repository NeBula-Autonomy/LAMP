/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_BASE_H
#define LAMP_BASE_H

// Includes 
#include <ros/ros.h>

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

// Services



// Class definition
class LampBase {
  public:
    // typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // Constructor
    LampBase();

    // Destructor
    ~LampBase();

    // Define main interface functions

    bool Initialize();

    bool public_variable_;

  protected:

    // Use this for any "private" things to be used in the derived class
    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    // bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
    // bool RegisterLogCallbacks(const ros::NodeHandle& n);
    // bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);


    // Private variables - won't be able to be accessed in the derived class
    float example_variable_;
    int variable_2_;

    // Booleans
    bool example_boolean_;

    // Publishers

    // Subscribers

    // Services 

    // Message filters (if any)

  private:
    // Anything just in the base class

};

#endif