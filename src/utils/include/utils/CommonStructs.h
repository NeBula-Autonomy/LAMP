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
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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

// Pose graph structure storing values, factors and meta data.
class PoseGraph {
public:
  const gtsam::Values& GetValues() const {
    return values_;
  }
  const gtsam::Values& GetNewValues() const {
    return values_new_;
  }
  const gtsam::NonlinearFactorGraph& GetNfg() const {
    return nfg_;
  }

  // Modifiable references to pose graph data structures.
  gtsam::Values& GetValues() {
    return values_;
  }
  gtsam::Values& GetNewValues() {
    return values_new_;
  }
  gtsam::NonlinearFactorGraph& GetNfg() {
    return nfg_;
  }

  // Function that maps gtsam::Symbol to std::string (internal identifier for
  // node messages).
  SymbolIdMapping symbol_id_map;

  std::string fixed_frame_id;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans;
  std::map<gtsam::Symbol, ros::Time> keyed_stamps; // All nodes
  std::map<double, gtsam::Symbol> stamp_to_odom_key;

  void InsertKeyedScan(gtsam::Symbol key, const PointCloud::ConstPtr& scan);
  void InsertKeyedStamp(gtsam::Symbol key, const ros::Time& stamp);
  void InsertStampedOdomKey(double seconds, gtsam::Symbol key);

  inline bool HasKey(gtsam::Symbol key) const {
    return values_.exists(key);
  }
  // Check if given key has a registered time stamp.
  inline bool HasStamp(gtsam::Symbol key) const {
    return keyed_stamps.find(key) != keyed_stamps.end();
  }
  inline bool HasScan(gtsam::Symbol key) const {
    return keyed_scans.find(key) != keyed_scans.end();
  }

  // Message filters (if any)
  std::string prefix{""};

  // Initial key
  gtsam::Symbol initial_key{0};

  // Current key
  gtsam::Symbol key{0};

  gtsam::Vector6 initial_noise{gtsam::Vector6::Zero()};

  inline gtsam::Pose3 LastPose() const {
    return values_.at<gtsam::Pose3>(key - 1);
  }
  inline gtsam::Pose3 GetPose(gtsam::Symbol key) const {
    return values_.at<gtsam::Pose3>(key);
  }

  void Initialize(gtsam::Symbol initial_key,
                  const gtsam::Pose3& pose,
                  const Diagonal::shared_ptr& covariance);

  // Tracks edge (factor) without updating nfg. Returns true if new factor is
  // added.
  bool TrackFactor(const Factor& factor);
  bool TrackFactor(const EdgeMessage& msg);
  bool TrackFactor(gtsam::Symbol key_from,
                   gtsam::Symbol key_to,
                   int type,
                   const gtsam::Pose3& transform,
                   const gtsam::SharedNoiseModel& covariance);

  // Tracks nodes and updates values. Returns true if new node is added.
  // Updates the internal keyed_stamp map for this key.
  bool TrackNode(const Node& node);
  bool TrackNode(const NodeMessage& msg);
  bool TrackNode(const ros::Time& stamp,
                 gtsam::Symbol key,
                 const gtsam::Pose3& pose,
                 const gtsam::SharedNoiseModel& covariance);

  // Tracks priors (one-sided edges). Returns true if new prior is added.
  // Does NOT update the internal keyed_stamp map for this key.
  bool TrackPrior(const Factor& prior);
  bool TrackPrior(const Node& prior);
  bool TrackPrior(const EdgeMessage& msg);
  bool TrackPrior(gtsam::Symbol key,
                  const gtsam::Pose3& pose,
                  const gtsam::SharedNoiseModel& covariance);

  // Adds gtsam::Values to internal values and values_new without updating node
  // messages.
  void AddNewValues(const gtsam::Values& new_values);
  // Adds factors to internal nfg without updating edge messages.
  void AddNewFactors(const gtsam::NonlinearFactorGraph& nfg);

  inline void ClearNewValues() {
    values_new_.clear();
  }
  bool EraseValue(const gtsam::Symbol key);

  // Time threshold for time-based lookup functions.
  static double time_threshold;
  // Convert timestamps to gtsam keys.
  gtsam::Symbol GetKeyAtTime(const ros::Time& stamp) const;
  // Returns symbol of closest node to given time stamp or the last symbol if
  // check_threshold, empty symbol otherwise.
  gtsam::Symbol GetClosestKeyAtTime(const ros::Time& stamp,
                                    bool check_threshold = true) const;
  inline static bool IsTimeWithinThreshold(double time,
                                           const ros::Time& target) {
    return std::abs(time - target.toSec()) <= time_threshold;
  }

  // Saves pose graph and accompanying point clouds to a zip file.
  template <typename PGOSolver>
  bool Save(const std::string& zipFilename, PGOSolver& solver) const;

  // Loads pose graph and accompanying point clouds from a zip file.
  template <typename PGOSolver>
  bool Load(const std::string& zipFilename, PGOSolver& solver);

  // Convert entire pose graph to message.
  GraphMsgPtr ToMsg() const;

  // Generates message from factors and values that were modified since the
  // last update.
  GraphMsgPtr ToIncrementalMsg() const;

  // Incremental update from pose graph message.
  void UpdateFromMsg(const GraphMsgPtr& msg);
  
  // Update all values_new_ so the incremental publisher republishes the whole graph 
  void AddAllValuesToNew();

  inline void ClearIncrementalMessages() {
    edges_new_.clear();
    nodes_new_.clear();
    priors_new_.clear();
    values_new_.clear();
  }

  // Clears entire pose graph (values, factors, meta data)
  inline void Reset() {
    ClearIncrementalMessages();
    edges_.clear();
    nodes_.clear();
    priors_.clear();
    values_.clear();
    nfg_ = gtsam::NonlinearFactorGraph();
    keyed_scans.clear();
    keyed_stamps.clear();
    stamp_to_odom_key.clear();
  }

  inline const EdgeSet& GetEdges() const {
    return edges_;
  }
  inline const NodeSet& GetNodes() const {
    return nodes_;
  }
  inline const EdgeSet& GetPriors() const {
    return priors_;
  }

  inline const EdgeSet& GetNewEdges() const {
    return edges_new_;
  }
  inline const NodeSet& GetNewNodes() const {
    return nodes_new_;
  }
  inline const EdgeSet& GetNewPriors() const {
    return priors_new_;
  }

  // Retrieves node at the given key, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const NodeMessage* FindNode(gtsam::Key key) const;

  // Retrieves edge connecting the given keys, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const EdgeMessage* FindEdge(gtsam::Key key_from, gtsam::Key key_to) const;

  // Retrieves prior of the given key, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const EdgeMessage* FindPrior(gtsam::Key key) const;

private:
  gtsam::Values values_;
  gtsam::NonlinearFactorGraph nfg_;

  // Cached messages for edges, nodes and priors to reduce publishing overhead.
  EdgeSet edges_;
  NodeSet nodes_;
  EdgeSet priors_;

  // Variables for tracking the new features only
  gtsam::Values values_new_;
  EdgeSet edges_new_;
  NodeSet nodes_new_;
  EdgeSet priors_new_;

  // Convert incremental pose graph with given values, edges and priors to
  // message.
  GraphMsgPtr ToMsg_(const EdgeSet& edges,
                     const NodeSet& nodes,
                     const EdgeSet& priors) const;
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

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
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
  double range;
  double range_error;
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

class AprilTagData : public FactorData {
public:
  AprilTagData(){};
  virtual ~AprilTagData(){};

  std::vector<AprilTagFactor> factors;
};

class UwbData : public FactorData {

  public:
    
    UwbData() { };
    virtual ~UwbData() { };

    std::vector<UwbFactor> factors;
};


#endif

// need to include source file for templatized save/load functions
#include "utils/PoseGraphFileIO.hpp"
