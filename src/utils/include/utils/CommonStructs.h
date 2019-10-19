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

#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/PoseStamped.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

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

typedef pose_graph_msgs::PoseGraphNode GraphNode;
typedef pose_graph_msgs::PoseGraphEdge GraphEdge;
typedef pose_graph_msgs::PoseGraph::ConstPtr GraphMsgPtr;

typedef std::vector<GraphEdge> EdgeMessages;
typedef std::vector<GraphNode> NodeMessages;

// Typedef for stored point clouds.
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct PoseGraph {
  gtsam::Values values;
  gtsam::NonlinearFactorGraph nfg;

  std::string fixed_frame_id;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans;
  std::map<gtsam::Symbol, ros::Time> keyed_stamps; // All nodes
  std::map<double, gtsam::Symbol> stamp_to_odom_key;

  // Message filters (if any)
  std::string prefix;

  // Initial key
  gtsam::Symbol initial_key;

  // Current key
  gtsam::Symbol key;

  EdgeMessages edges;
  NodeMessages priors;

  gtsam::Vector6 initial_noise{gtsam::Vector6::Zero()};

  // Saves pose graph and accompanying point clouds to a zip file.
  template <typename PGOSolver>
  bool Save(const std::string& zipFilename, PGOSolver& solver) const;

  // Loads pose graph and accompanying point clouds from a zip file.
  template <typename PGOSolver>
  bool Load(const std::string& zipFilename, PGOSolver& solver);

  // Convert entire pose graph to message.
  GraphMsgPtr ToMsg() const;

  // Convert incremental pose graph with given values, edges and priors to
  // message.
  GraphMsgPtr ToMsg(const gtsam::Values& values,
                    const EdgeMessages& edges,
                    const NodeMessages& priors) const;

  // Incremental update from pose graph message.
  void UpdateFromMsg(const GraphMsgPtr& msg);
};

// Struct definition
struct FactorData {
  bool b_has_data;  // False if there is no data
  std::string type; // odom, artifact, loop clsoure
  // Vector for possible multiple factors
  std::vector<gtsam::Pose3> transforms; // The transform (for odom, loop
                                        // closures etc.) and pose for TS
  std::vector<gtsam::SharedNoiseModel>
      covariances; // Covariances for each transform
  std::vector<std::pair<ros::Time, ros::Time>>
      time_stamps; // Time when the measurement as acquired (first, second)
  // TODO - use ros::Time or something else?

  std::vector<gtsam::Symbol> artifact_key; // key for the artifacts
};

#endif

#include "utils/SaveLoadGraph.hpp"
