/*
 * Copyright Notes
 *
 * Authors: 
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

// Typedef for 6x6 covariance matrices (x, y, z, roll, pitch, yaw).
typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;
typedef geometry_utils::MatrixNxNBase<double, 12> Mat1212;

// Struct definition 
struct FactorData {
  std::string type; // odom, artifact, loop clsoure
  // Vector for possible multiple factors
  std::vector<gtsam::Pose3> transforms; // The transform (for odom, loop closures etc.) and pose for TS
  std::vector<Mat1212> covariances; // Covariances for each transform 
  std::vector<std::pair<ros::Time, ros::Time>> time_stamps; // Time when the measurement as acquired
  // TODO - use ros::Time or something else?

  std::vector<gtsam::Key> artifact_key; // key for the artifacts
};

#endif