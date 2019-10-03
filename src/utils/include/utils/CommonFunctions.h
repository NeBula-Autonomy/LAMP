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

namespace gu = geometry_utils;

namespace utils {
  // Pose graph msg to gtsam conversion
  void PoseGraphMsgToGtsam(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg,
                          gtsam::NonlinearFactorGraph* graph_nfg,
                          gtsam::Values* graph_vals);


  // inline ConvertMsgToPoseGraph(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg,
  //                         gtsam::NonlinearFactorGraph* graph_nfg,
  //                         gtsam::Values* graph_vals){

  // }

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

    gtsam::Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
          pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
          pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));

    return gtsam::Pose3(r, t);
  }

  inline gtsam::Point3 ToGtsam(const gu::Vec3& point) {
    gtsam::Point3 pt(point(0), point(1), point(2)); 
    return pt;
  }

  inline Mat66 ToGu(
      const Gaussian::shared_ptr& covariance) {
    gtsam::Matrix66 gtsam_covariance = covariance->covariance();

    Mat66 out;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        out(i, j) = gtsam_covariance(i, j);

    return out;
  }

  inline Gaussian::shared_ptr ToGtsam(
      const Mat66& covariance) {
    gtsam::Matrix66 gtsam_covariance;

    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        gtsam_covariance(i, j) = covariance(i, j);

    return Gaussian::Covariance(gtsam_covariance);
  }

  inline Gaussian::shared_ptr ToGtsam(
      const Mat1212& covariance) {
    gtsam::Vector12 gtsam_covariance; 
    // TODO CHECK
    for (int i = 0; i < 12; ++i) 
      gtsam_covariance(i) = covariance(i,i);
    return gtsam::noiseModel::Diagonal::Covariance(gtsam_covariance);
  }
}  // namespace utils
#endif  // COMMON_FUNCTIONS_H
