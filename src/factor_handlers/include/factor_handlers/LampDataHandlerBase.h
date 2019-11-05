/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef LAMP_DATA_HANDLER_BASE_H
#define LAMP_DATA_HANDLER_BASE_H

// Includes
#include <ros/ros.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>

#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/CommonStructs.h>
#include <utils/CommonFunctions.h>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

class LampDataHandlerBase {

  public:

    LampDataHandlerBase();
    ~LampDataHandlerBase();

    virtual bool Initialize(const ros::NodeHandle& n);
    virtual std::shared_ptr<FactorData> GetData() = 0;


  protected:
    // Node initialization.
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
    bool RegisterLogCallbacks(const ros::NodeHandle& n);
    bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // Reset Factor data
    void ResetFactorData();

  private:
};

#endif

// TODO - remaptopic names for the handlers in the launch file