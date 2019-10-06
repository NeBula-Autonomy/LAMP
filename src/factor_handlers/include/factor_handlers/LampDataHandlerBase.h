/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_DATA_HANDLER_BASE_H
#define LAMP_DATA_HANDLER_BASE_H

// Includes 
#include <ros/ros.h>

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/CommonStructs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

class LampDataHandlerBase {

  public:

    LampDataHandlerBase();
    ~LampDataHandlerBase();

    virtual bool Initialize(const ros::NodeHandle& n);
    virtual FactorData GetData();

  protected:

    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
    bool RegisterLogCallbacks(const ros::NodeHandle& n);
    bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Reset Factor data
    void ResetFactorData();

    // Callback functions
    // void DataCallback();

    // LAMP Interface
    FactorData factors_;

  private:

};

#endif

// TODO - remaptopic names for the handlers in the launch file