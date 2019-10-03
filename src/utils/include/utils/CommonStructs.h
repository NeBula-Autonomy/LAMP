/*
 * Copyright Notes
 *
 * Authors: 
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */


#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/inference/Symbol.h>

#include <geometry_utils/Transform3.h>
#include <geometry_utils/GeometryUtilsROS.h>

// Typedef for 6x6 covariance matrices (x, y, z, roll, pitch, yaw).
typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;

// GTSAM edge types
typedef std::pair<gtsam::Symbol, gtsam::Symbol> Edge;
typedef std::pair<gtsam::Symbol, gtsam::Symbol> ArtifactEdge;
typedef std::pair<gtsam::Symbol, gtsam::Pose3> Prior; 


// Struct definition 
struct FactorData {
  bool b_has_data; // False if there is no data
  std::string type; // odom, artifact, loop clsoure
  // Vector for possible multiple factors
  std::vector<gtsam::Pose3> transforms; // The transform (for odom, loop closures etc.) and pose for TS
  std::vector<Mat66> covariances; // Covariances for each transform 
  std::vector<std::pair<ros::Time, ros::Time>> time_stamps; // Time when the measurement as acquired
  // TODO - use ros::Time or something else?

  std::vector<gtsam::Key> artifact_key; // key for the artifacts
};

#endif