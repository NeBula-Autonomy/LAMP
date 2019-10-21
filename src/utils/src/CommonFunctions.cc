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

gtsam::Pose3 EdgeMessageToPose(const pose_graph_msgs::PoseGraphEdge& msg_edge) {
  gtsam::Point3 delta_translation(msg_edge.pose.position.x,
                                  msg_edge.pose.position.y,
                                  msg_edge.pose.position.z);
  gtsam::Rot3 delta_orientation(
      gtsam::Rot3::quaternion(msg_edge.pose.orientation.w,
                              msg_edge.pose.orientation.x,
                              msg_edge.pose.orientation.y,
                              msg_edge.pose.orientation.z));
  gtsam::Pose3 delta = gtsam::Pose3(delta_orientation, delta_translation);

  return delta;
}

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

  // TODO set prior.ID

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
