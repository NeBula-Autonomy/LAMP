/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#ifndef COMMON_STRUCTS_H
#define COMMON_STRUCTS_H

#include <boost/function.hpp>

// GTSAM
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>

#include <utils/PrefixHandling.h>

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

typedef pose_graph_msgs::PoseGraphNode NodeMessage;
typedef pose_graph_msgs::PoseGraphEdge EdgeMessage;
typedef pose_graph_msgs::PoseGraph::ConstPtr GraphMsgPtr;

typedef std::vector<EdgeMessage> EdgeMessages;
typedef std::vector<NodeMessage> NodeMessages;

// Implements strictly-less-than operator for edge messages.
struct EdgeMessageComparator {
  bool operator()(const EdgeMessage& lhs, const EdgeMessage& rhs) const {
    if (lhs.key_from < rhs.key_from)
      return true;
    if (lhs.key_from > rhs.key_from)
      return false;
    if (lhs.key_to < rhs.key_to)
      return true;
    if (lhs.key_to > rhs.key_to)
      return false;
    return lhs.type < rhs.type;
  }
};

// Implements strictly-less-than operator for node messages.
struct NodeMessageComparator {
  bool operator()(const NodeMessage& lhs, const NodeMessage& rhs) const {
    return lhs.key < rhs.key;
  }
};

// Use sets of edges/nodes to avoid duplicates.
typedef std::set<EdgeMessage, EdgeMessageComparator> EdgeSet;
typedef std::set<NodeMessage, NodeMessageComparator> NodeSet;

// Typedef for stored point clouds.
typedef pcl::PointXYZINormal Point;
typedef pcl::PointCloud<Point> PointCloud;

// Function that maps gtsam::Symbol to internal identifier string.
typedef boost::function<std::string(gtsam::Symbol)> SymbolIdMapping;

// Forward declaration.
class PoseGraph;

// GTSAM factor (edge).
struct Factor {
  gtsam::Symbol key_from;
  gtsam::Symbol key_to;
  int type;
  gtsam::Pose3 transform;
  gtsam::SharedNoiseModel covariance;

  // Optional pointer to parent pose graph.
  PoseGraph* graph{nullptr};

  EdgeMessage ToMsg() const;
  static Factor FromMsg(const EdgeMessage& msg);
};

// GTSAM node (prior).
struct Node {
  ros::Time stamp;
  std::string fixed_frame_id;
  gtsam::Symbol key;
  // Type-dependent ID that is optionally set.
  std::string ID{""};
  gtsam::Pose3 pose;
  gtsam::SharedNoiseModel covariance;

  // Optional pointer to parent pose graph.
  PoseGraph* graph{nullptr};

  NodeMessage ToMsg() const;
  static Node FromMsg(const NodeMessage& msg);

  Node(const ros::Time& stamp,
       gtsam::Symbol key,
       const gtsam::Pose3& pose,
       const gtsam::SharedNoiseModel& covariance,
       PoseGraph* graph = nullptr);
  Node() : stamp(ros::Time::now()) {}
};

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

  pcl::PointCloud<Point>::Ptr point_cloud;
  bool b_has_point_cloud;

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

struct ImuFactor {
  // This is required because gtsam::Pose3AttitudeFactor has a deleted
  // operator= so the attitude can't be assigned a value
  ImuFactor(gtsam::Pose3AttitudeFactor factor) : attitude(factor) {}

  gtsam::Pose3AttitudeFactor attitude;
};

struct UwbFactor {
  ros::Time stamp;
  gtsam::Symbol key_from;
  gtsam::Symbol key_to;

  // pose_graph_msgs::PoseGraphEdge::UWB_RANGE
  // pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN
  int type;

  // Only for range factors
  double range;
  double range_error;

  // Only for between factors
  gtsam::Pose3 pose;
};

struct PoseData {
  ros::Time stamp;
  gtsam::Pose3 pose;
};

// ---------------------------------------------------------

// Base factor data class
class FactorData {
public:
  FactorData() : b_has_data(false){};
  virtual ~FactorData(){};

  bool b_has_data;  // False if there is no data
  std::string type; // odom, artifact, loop closure
};

// Derived classes
class OdomData : public FactorData {
public:
  OdomData(){};
  virtual ~OdomData(){};

  std::vector<OdometryFactor> factors;
};

class ArtifactData : public FactorData {
public:
  ArtifactData(){};
  virtual ~ArtifactData(){};

  std::vector<ArtifactFactor> factors;
};

class LoopClosureData : public FactorData {
public:
  LoopClosureData(){};
  virtual ~LoopClosureData(){};

  std::vector<LoopClosureFactor> factors;
};

class ImuData : public FactorData {
public:
  ImuData(){};
  virtual ~ImuData(){};

  std::vector<ImuFactor> factors;
};

class PoseGraphData : public FactorData {
public:
  PoseGraphData(){};
  virtual ~PoseGraphData(){};

  std::vector<pose_graph_msgs::PoseGraph::ConstPtr> graphs;
  std::vector<pose_graph_msgs::KeyedScan::ConstPtr> scans;
};

class RobotPoseData : public FactorData {
public:
  RobotPoseData(){};
  virtual ~RobotPoseData(){};

  // Stores most recent pose for each robot
  std::map<std::string, PoseData> poses;
};

class AprilTagData : public FactorData {
public:
  AprilTagData(){};
  virtual ~AprilTagData(){};

  std::vector<AprilTagFactor> factors;
};

class UwbData : public FactorData {
public:
  UwbData(){};
  virtual ~UwbData(){};

  std::vector<UwbFactor> factors;
};

#endif
