/*
 * Copyright Notes
 *
 * Authors:
 * Alex Stephens       (alex.stephens@jpl.nasa.gov)
 * Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 * Yun Chang           (yunchang@mit.edu)
 */

#ifndef POSE_GRAPH_H
#define POSE_GRAPH_H

#include <utils/CommonStructs.h>
#include <utils/PrefixHandling.h>

// Pose graph structure storing values, factors and meta data.
class PoseGraph {
 public:
  bool b_first_;
  inline const gtsam::Values& GetValues() const { return values_; }
  inline const gtsam::Values& GetNewValues() const { return values_new_; }
  inline const gtsam::NonlinearFactorGraph& GetNfg() const { return nfg_; }

  // Modifiable references to pose graph data structures.
  inline gtsam::Values& GetValues() { return values_; }
  inline gtsam::Values& GetNewValues() { return values_new_; }
  inline gtsam::NonlinearFactorGraph& GetNfg() { return nfg_; }

  // Function that maps gtsam::Symbol to std::string (internal identifier for
  // node messages).
  SymbolIdMapping symbol_id_map;

  std::string fixed_frame_id;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<gtsam::Symbol, PointCloud::ConstPtr> keyed_scans;
  std::map<gtsam::Symbol, ros::Time> keyed_stamps;  // All nodes
  std::map<double, gtsam::Symbol> stamp_to_odom_key;

  void InsertKeyedScan(const gtsam::Symbol& key,
                       const PointCloud::ConstPtr& scan);
  void InsertKeyedStamp(const gtsam::Symbol& key, const ros::Time& stamp);
  void InsertStampedOdomKey(double seconds, const gtsam::Symbol& key);

  inline bool HasKey(const gtsam::Symbol& key) const {
    return values_.exists(key);
  }
  // Check if given key has a registered time stamp.
  inline bool HasStamp(const gtsam::Symbol& key) const {
    return keyed_stamps.find(key) != keyed_stamps.end();
  }
  inline bool HasScan(const gtsam::Symbol& key) const {
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
  inline void AddLastNodeToNew() {
    gtsam::Key last_node_key = key - 1;
    const gtsam::Pose3& pose = LastPose();

    if (values_new_.exists(last_node_key)) {
      values_new_.update(last_node_key, pose);
    } else {
      values_new_.insert(last_node_key, pose);
    }
  }

  inline gtsam::Pose3 GetPose(gtsam::Symbol key) const {
    return values_.at<gtsam::Pose3>(key);
  }

  void Initialize(const gtsam::Symbol& initial_key,
                  const gtsam::Pose3& pose,
                  const Diagonal::shared_ptr& covariance);

  // Tracks edge (factor) and updates nfg. Returns true if new edge is added.
  bool TrackFactor(const Factor& factor);
  bool TrackFactor(const EdgeMessage& msg);
  bool TrackFactor(const gtsam::Symbol& key_from,
                   const gtsam::Symbol& key_to,
                   int type,
                   const gtsam::Pose3& transform,
                   const gtsam::SharedNoiseModel& covariance,
                   bool create_msg = true);
  bool TrackUWBFactor(const gtsam::Symbol& key_from,
                      const gtsam::Symbol& key_to,
                      double range,
                      double range_error,
                      bool create_msg = true);
  bool TrackIMUFactor(const gtsam::Symbol& key_to,
                      const geometry_msgs::Point& meas,
                      const geometry_msgs::Point& ref,
                      double att_noise,
                      bool create_msg = true);

  // Tracks nodes and updates values. Returns true if new node is added.
  // Updates the internal keyed_stamp map for this key.
  bool TrackNode(const Node& node);
  bool TrackNode(const NodeMessage& msg);
  bool TrackNode(const ros::Time& stamp,
                 const gtsam::Symbol& key,
                 const gtsam::Pose3& pose,
                 const gtsam::SharedNoiseModel& covariance,
                 const std::string& id = "",
                 bool create_msg = true);

  // Tracks priors (one-sided edges). Returns true if new prior is added.
  // Does NOT update the internal keyed_stamp map for this key.
  bool TrackPrior(const Factor& prior);
  bool TrackPrior(const Node& prior);
  bool TrackPrior(const EdgeMessage& msg);
  bool TrackPrior(const gtsam::Symbol& key,
                  const gtsam::Pose3& pose,
                  const gtsam::SharedNoiseModel& covariance,
                  bool create_msg = true);

  // Removal tools
  void RemoveRobotFromGraph(std::string robot_name);
  void RemoveEdgesWithPrefix(unsigned char prefix);
  void RemoveValuesWithPrefix(unsigned char prefix);

  // Adds gtsam::Values to internal values and values_new without updating node
  // messages.
  void AddNewValues(const gtsam::Values& new_values);
  // Adds factors to internal nfg without updating edge messages.
  void AddNewFactors(const gtsam::NonlinearFactorGraph& nfg);

  inline void ClearNewValues() { values_new_.clear(); }
  bool EraseValue(const gtsam::Symbol& key);

  gtsam::Pose3 LastPose(char c) const;

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
  bool Save(const std::string& zipFilename) const;

  // Loads pose graph and accompanying point clouds from a zip file.
  bool Load(const std::string& zipFilename,
            const std::string& pose_graph_topic_name = "pose_graph");

  // Convert entire pose graph to message.
  GraphMsgPtr ToMsg() const;

  // Generates message from factors and values that were modified since the
  // last update.
  GraphMsgPtr ToIncrementalMsg() const;

  // Incremental update from pose graph message.
  void UpdateFromMsg(const GraphMsgPtr& msg);

  // Update all values_new_ so the incremental publisher republishes the whole
  // graph
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

  inline const EdgeSet& GetEdges() const { return edges_; }
  inline const NodeSet& GetNodes() const { return nodes_; }
  inline const EdgeSet& GetPriors() const { return priors_; }

  inline const EdgeSet& GetNewEdges() const { return edges_new_; }
  inline const NodeSet& GetNewNodes() const { return nodes_new_; }
  inline const EdgeSet& GetNewPriors() const { return priors_new_; }

  // Retrieves node at the given key, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const NodeMessage* FindNode(const gtsam::Key& key) const;

  // Retrieves edge connecting the given keys, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const EdgeMessage* FindEdge(const gtsam::Key& key_from,
                              const gtsam::Key& key_to) const;

  // Retrieves prior of the given key, returns nullptr otherwise.
  // Returns const ptr because std::set only has const_iterators.
  const EdgeMessage* FindPrior(const gtsam::Key& key) const;

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

#endif