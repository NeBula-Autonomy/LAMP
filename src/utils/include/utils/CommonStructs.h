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
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>

// Typedef for 6x6 covariance matrices (x, y, z, roll, pitch, yaw).
typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;
typedef geometry_utils::MatrixNxNBase<double, 12> Mat1212;

// Noise models
typedef gtsam::noiseModel::Gaussian Gaussian;
typedef gtsam::noiseModel::Diagonal Diagonal;

// GTSAM edge types
typedef std::pair<gtsam::Symbol, gtsam::Symbol> Edge;
typedef std::pair<gtsam::Symbol, gtsam::Symbol> ArtifactEdge;
typedef std::pair<gtsam::Symbol, gtsam::Pose3> Prior;


// ---------------------------------------------------------
// Data structures for each factor type 
struct ArtifactFactor {
  ros::Time stamp; 
  gtsam::Symbol key;

  gtsam::Point3 position;
  gtsam::SharedNoiseModel covariance;
};

struct AprilTagFactor {
  ros::Time stamp; 
  gtsam::Symbol key;

  gtsam::Point3 position;
  gtsam::Point3 ground_truth;
  gtsam::SharedNoiseModel covariance;
};

struct OdometryFactor {
  std::pair<ros::Time, ros::Time> stamps;

  gtsam::Pose3 transform;
  gtsam::SharedNoiseModel covariance;
};

struct LoopClosureFactor {
  ros::Time stamp;
  gtsam::Symbol key_from;
  gtsam::Symbol key_to;

  gtsam::Pose3 transform;
  gtsam::SharedNoiseModel covariance;
};

struct PriorFactor {
  ros::Time stamp;
  gtsam::Symbol key;

  gtsam::Pose3 pose;
  gtsam::SharedNoiseModel covariance;
};

// ---------------------------------------------------------

// Base factor data class
class FactorData {

  public: 

    FactorData() { };
    virtual ~FactorData() { };

    bool b_has_data;   // False if there is no data
    std::string type;  // odom, artifact, loop closure

};

// Derived classes
class OdomData : public FactorData {

  public: 

    OdomData() { };
    virtual ~OdomData() { };

    std::vector<OdometryFactor> factors;
};

class ArtifactData : public FactorData {

  public: 

    ArtifactData() { };
    virtual ~ArtifactData() { };

    std::vector<ArtifactFactor> factors;
};

class AprilTagData : public FactorData {

  public: 

    AprilTagData() { };
    virtual ~AprilTagData() { };

    std::vector<AprilTagFactor> factors;
};
#endif
