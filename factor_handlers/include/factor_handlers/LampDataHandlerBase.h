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
#include <lamp_utils/CommonStructs.h>
#include <lamp_utils/CommonFunctions.h>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

class LampDataHandlerBase {

  public:

    LampDataHandlerBase();
    ~LampDataHandlerBase();

    virtual std::shared_ptr<FactorData> GetData() = 0;
};

#endif

// TODO - remaptopic names for the handlers in the launch file