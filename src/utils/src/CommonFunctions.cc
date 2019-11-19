/*
 * Copyright Notes
 *
 * Authors:
 * Yun Chang       (yunchang@mit.edu)
 */

#include "utils/CommonFunctions.h"
#include "utils/CommonStructs.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <ros/console.h>

using gtsam::BetweenFactor;
using gtsam::NonlinearFactorGraph;
using gtsam::PriorFactor;
using gtsam::RangeFactor;
using gtsam::Values;
using gtsam::noiseModel::Gaussian;

namespace gu = geometry_utils;

namespace utils {

pose_graph_msgs::PoseGraphEdge
GtsamToRosMsg(gtsam::Symbol key_from,
              gtsam::Symbol key_to,
              int type,
              const gtsam::Pose3& pose,
              const gtsam::SharedNoiseModel& covariance) {
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key_from;
  edge.key_to = key_to;
  edge.type = type;
  // edge.header.frame_id = fixed_frame_id_;
  // edge.header.stamp = keyed_stamps_[key_to];
  edge.pose = gr::ToRosPose(ToGu(pose));

  // Update the covariance
  UpdateCovariance(edge, covariance);

  return edge;
}

pose_graph_msgs::PoseGraphNode
GtsamToRosMsg(ros::Time stamp,
              const std::string& fixed_frame_id,
              gtsam::Symbol key,
              const gtsam::Pose3& pose,
              const gtsam::SharedNoiseModel& covariance) {
  pose_graph_msgs::PoseGraphNode prior;
  prior.key = key;
  prior.header.frame_id = fixed_frame_id;
  prior.header.stamp = stamp;
  prior.pose = gr::ToRosPose(ToGu(pose));
  ROS_INFO_STREAM("Quaternion before norm is " << prior.pose.orientation);
  double norm = pow(prior.pose.orientation.x*prior.pose.orientation.x+prior.pose.orientation.y*prior.pose.orientation.y+prior.pose.orientation.z*prior.pose.orientation.z + prior.pose.orientation.w*prior.pose.orientation.w,0.5);
  prior.pose.orientation.x = prior.pose.orientation.x/norm;
  prior.pose.orientation.y = prior.pose.orientation.y/norm;
  prior.pose.orientation.z = prior.pose.orientation.z/norm;
  prior.pose.orientation.w = prior.pose.orientation.w/norm;
  // TODO set prior.ID
  ROS_INFO_STREAM("Quaternion after norm is " << prior.pose.orientation);

  // Update the covariance
  UpdateCovariance(prior, covariance);

  return prior;
}

geometry_msgs::Pose GtsamToRosMsg(const gtsam::Pose3& pose) {
  // Convert with existing tools - TODO - find a better way to do this
  geometry_msgs::Pose msg = gr::ToRosPose(ToGu(pose));

  return msg;
}

// Convert gtsam data types to a ros message
geometry_msgs::PoseWithCovariance
GtsamToRosMsg(const gtsam::Pose3& pose, const gtsam::Matrix66& covariance) {
  geometry_msgs::PoseWithCovariance msg;

  // Convert with existing tools
  msg.pose = gr::ToRosPose(ToGu(pose));
  UpdateCovariance(msg, covariance);

  return msg;
}

} // namespace utils
