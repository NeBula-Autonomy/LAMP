/*
 * Copyright Notes
 *
 * Authors:
 * Yun Chang       (yunchang@mit.edu)
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <utils/CommonStructs.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

namespace utils {

// Pose graph msg to gtsam conversion
// TODO remove this function, use PoseGraph functions instead.
void PoseGraphMsgToGtsam(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg,
                         gtsam::NonlinearFactorGraph* graph_nfg,
                         gtsam::Values* graph_vals);

// Convert edge/node message to gtsam pose
template <typename MessageT>
gtsam::Pose3 MessageToPose(const MessageT& msg) {
  gtsam::Point3 delta_translation(
      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  gtsam::Rot3 delta_orientation(
      gtsam::Rot3::quaternion(msg.pose.orientation.w,
                              msg.pose.orientation.x,
                              msg.pose.orientation.y,
                              msg.pose.orientation.z));
  gtsam::Pose3 delta = gtsam::Pose3(delta_orientation, delta_translation);

  return delta;
}

// Extract the covariance (as Gaussian Covariance in Shared Noise Model) from
// edge or node message
template <typename MessageT>
Gaussian::shared_ptr MessageToCovariance(const MessageT& msg) {
  // TODO: unit test
  gtsam::Matrix66 covariance;
  for (size_t i = 0; i < msg.covariance.size(); i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    covariance(row, col) = msg.covariance[i];
  }
  Gaussian::shared_ptr noise = Gaussian::Covariance(covariance);

  return noise;
}

// Update covariances in an edge or node message
template <typename MessageT>
void UpdateCovariance(MessageT& msg, const gtsam::Matrix66& covariance) {
  for (size_t i = 0; i < msg.covariance.size(); i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    msg.covariance[i] = covariance(row, col);
  }
}
template <typename MessageT>
void UpdateCovariance(MessageT& msg, const gtsam::SharedNoiseModel& noise) {
  gtsam::Matrix66 covariance =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise)
          ->covariance();
  UpdateCovariance(msg, covariance);
}

// Convert gtsam data types to a ros message
geometry_msgs::PoseWithCovariance
GtsamToRosMsg(const gtsam::Pose3& pose, const gtsam::Matrix66& covariance);

geometry_msgs::Pose GtsamToRosMsg(const gtsam::Pose3& pose);

pose_graph_msgs::PoseGraphEdge
GtsamToRosMsg(gtsam::Symbol key_from,
              gtsam::Symbol key_to,
              int type,
              const gtsam::Pose3& pose,
              const gtsam::SharedNoiseModel& covariance);

pose_graph_msgs::PoseGraphNode
GtsamToRosMsg(ros::Time stamp,
              const std::string& fixed_frame_id,
              gtsam::Symbol key,
              const gtsam::Pose3& pose,
              const gtsam::SharedNoiseModel& covariance);

inline gu::Transform3 ToGu(const gtsam::Pose3& pose) {
  gu::Transform3 out;
  out.translation(0) = pose.translation().x();
  out.translation(1) = pose.translation().y();
  out.translation(2) = pose.translation().z();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      out.rotation(i, j) = pose.rotation().matrix()(i, j);
  }

  return out;
}

inline gtsam::Pose3 ToGtsam(const gu::Transform3& pose) {
  gtsam::Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  gtsam::Rot3 r(pose.rotation(0, 0),
                pose.rotation(0, 1),
                pose.rotation(0, 2),
                pose.rotation(1, 0),
                pose.rotation(1, 1),
                pose.rotation(1, 2),
                pose.rotation(2, 0),
                pose.rotation(2, 1),
                pose.rotation(2, 2));

  return gtsam::Pose3(r, t);
}

inline gtsam::Point3 ToGtsam(const gu::Vec3& point) {
  gtsam::Point3 pt(point(0), point(1), point(2));
  return pt;
}

inline Mat66 ToGu(const Gaussian::shared_ptr& covariance) {
  gtsam::Matrix66 gtsam_covariance = covariance->covariance();

  Mat66 out;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      out(i, j) = gtsam_covariance(i, j);

  return out;
}

inline Gaussian::shared_ptr ToGtsam(const Mat66& covariance) {
  gtsam::Matrix66 gtsam_covariance;

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      gtsam_covariance(i, j) = covariance(i, j);

  return Gaussian::Covariance(gtsam_covariance);
}

inline Gaussian::shared_ptr ToGtsam(const Mat1212& covariance) {
  gtsam::Vector12 gtsam_covariance;
  // TODO CHECK
  for (int i = 0; i < 12; ++i)
    gtsam_covariance(i) = covariance(i, i);
  return gtsam::noiseModel::Diagonal::Covariance(gtsam_covariance);
}
} // namespace utils
#endif // COMMON_FUNCTIONS_H
