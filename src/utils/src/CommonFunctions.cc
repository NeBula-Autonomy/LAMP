/*
 * Copyright Notes
 *
 * Authors:
 * Yun Chang       (yunchang@mit.edu)
 */

#include "utils/CommonFunctions.h"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
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

namespace utils {

// Pose graph msg to gtsam conversion
void PoseGraphMsgToGtsam(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg,
                         NonlinearFactorGraph* graph_nfg,
                         Values* graph_vals) {
  // Check not null
  if (NULL == graph_msg || NULL == graph_nfg || NULL == graph_vals) {
    ROS_ERROR("Null values in PoseGraphMsgToGtsam");
    return
  }

  graph_nfg->clear();
  graph_vals->clear();

  for (const auto& msg_edge : msg->edges) {
    gtsam::Point3 delta_translation(msg_edge.pose.position.x,
                                    msg_edge.pose.position.y,
                                    msg_edge.pose.position.z);
    gtsam::Rot3 delta_orientation(
        Rot3::quaternion(msg_edge.pose.orientation.w,
                         msg_edge.pose.orientation.x,
                         msg_edge.pose.orientation.y,
                         msg_edge.pose.orientation.z));
    gtsam::Pose3 delta = gtsam::Pose3(delta_orientation, delta_translation);

    // TODO(Yun) fill in covariance
    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
      covariance(i, i) = 0.316 * 0.316;  // 0.4, 0.004; 0.2 m sd
    for (int i = 3; i < 6; ++i)
      covariance(i, i) = 0.141 * 0.141;  // 0.1, 0.01; sqrt(0.01) rad sd
    //-------------------------------------------------------------------------

    if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ODOM) {
      // Add to posegraph
      ROS_DEBUG_STREAM("Adding Odom edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));

    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
      ROS_DEBUG_STREAM("Adding loop closure edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));

    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      ROS_DEBUG_STREAM("Adding artifact edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));

    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) {
      ROS_DEBUG_STREAM("Adding UWB range factor for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      double range = msg_edge.range;
      double sigmaR = msg_edge.range_error;
      gtsam::noiseModel::Base::shared_ptr rangeNoise =
          gtsam::noiseModel::Isotropic::Sigma(1, sigmaR);
      graph_nfg->add(RangeFactor<Pose3, Pose3>(gtsam::Symbol(msg_edge.key_from),
                                               gtsam::Symbol(msg_edge.key_to),
                                               range,
                                               rangeNoise));

    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN) {
      ROS_DEBUG_STREAM("Adding UWB beteen factor for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance_uwb)));
    }
  }

  //-----------------------------------------------//
  // Add the values
  for (const pose_graph_msgs::PoseGraphNode& msg_node : msg->nodes) {
    gtsam::Pose3 full_pose;
    gtsam::Point3 pose_translation(msg_node.pose.position.x,
                                   msg_node.pose.position.y,
                                   msg_node.pose.position.z);
    gtsam::Rot3 pose_orientation(Rot3::quaternion(msg_node.pose.orientation.w,
                                                  msg_node.pose.orientation.x,
                                                  msg_node.pose.orientation.y,
                                                  msg_node.pose.orientation.z));
    full_pose = gtsam::Pose3(pose_orientation, pose_translation);

    gtsam::Key key = gtsam::Key(msg->key);
    graph_vals->insert(key, full_pose);

    // TODO right now this only accounts for translation part of prior (see
    for (const pose_graph_msgs::PoseGraphNode& msg_prior : msg->priors) {
      ROS_DEBUG_STREAM("Adding prior in pose graph callback for key "
                       << gtsam::DefaultKeyFormatter(msg_prior.key));
      // create the prior factor

      // create precisions
      gtsam::Vector6 prior_precisions;
      prior_precisions.head<3>().setConstant(0.0);
      prior_precisions.tail<3>().setConstant(10.0);
      // TODO(Yun) create parameter for this
      static const gtsam::SharedNoiseModel& prior_noise =
          gtsam::noiseModel::Diagonal::Precisions(prior_precisions);

      gtsam::Point3 pose_translation(msg_prior.pose.position.x,
                                     msg_prior.pose.position.y,
                                     msg_prior.pose.position.z);
      gtsam::Rot3 pose_orientation(
          Rot3::quaternion(msg_prior.pose.orientation.w,
                           msg_prior.pose.orientation.x,
                           msg_prior.pose.orientation.y,
                           msg_prior.pose.orientation.z));
      gtsam::Pose3 prior_pose =
          gtsam::Pose3(pose_orientation, pose_translation);

      graph_nfg_->add(
          PriorFactor<gtsam::Pose3>(msg_prior.key, prior_pose, prior_noise));
    }
  }
}

}  // namespace utils
