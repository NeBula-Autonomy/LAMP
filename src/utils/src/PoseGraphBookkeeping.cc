#include "utils/CommonFunctions.h"
#include "utils/CommonStructs.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/AttitudeFactor.h>


bool PoseGraph::TrackFactor(const Factor& factor) {
  return TrackFactor(factor.key_from,
                     factor.key_to,
                     factor.type,
                     factor.transform,
                     factor.covariance);
}

bool PoseGraph::TrackFactor(const EdgeMessage& msg) {
  if (msg.type == pose_graph_msgs::PoseGraphEdge::PRIOR) {
    TrackPrior(msg);
    return false;
  }

  if (edges_.find(msg) != edges_.end()) {
    ROS_DEBUG_STREAM("Edge of type " << msg.type << " from key "
                                     << gtsam::DefaultKeyFormatter(msg.key_from)
                                     << " to key "
                                     << gtsam::DefaultKeyFormatter(msg.key_to)
                                     << " already exists.");
    return false;
  }

  bool success = true;
  if (msg.type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) {
    success = TrackUWBFactor(gtsam::Symbol(msg.key_from),
                             gtsam::Symbol(msg.key_to),
                             msg.range,
                             msg.range_error,
                             false);
  } else if (msg.type == pose_graph_msgs::PoseGraphEdge::IMU) {
    geometry_msgs::Point ref;
    ref.z = 1;
    success = TrackIMUFactor(gtsam::Symbol(msg.key_to),
                            msg.pose.position,
                            ref,
                            msg.covariance[0],
                            false);
  } else {
    gtsam::Pose3 delta = utils::MessageToPose(msg);
    Gaussian::shared_ptr noise = utils::MessageToCovariance(msg);

    // Track factor without (re)creating an edge message
    success = TrackFactor(gtsam::Symbol(msg.key_from),
                          gtsam::Symbol(msg.key_to),
                          msg.type,
                          delta,
                          noise,
                          false);
  }

  if (success) {
    edges_.insert(msg);
    edges_new_.insert(msg);
  }
  return success;
}

bool PoseGraph::TrackFactor(gtsam::Symbol key_from,
                            gtsam::Symbol key_to,
                            int type,
                            const gtsam::Pose3& transform,
                            const gtsam::SharedNoiseModel& covariance,
                            bool create_msg) {
  if (type == pose_graph_msgs::PoseGraphEdge::PRIOR) {
    return TrackPrior(key, transform, covariance);
  }

  if (create_msg) {
    auto msg =
        utils::GtsamToRosMsg(key_from, key_to, type, transform, covariance);
    if (edges_.find(msg) != edges_.end()) {
      ROS_DEBUG_STREAM("Edge of type " << type << " from key "
                                       << gtsam::DefaultKeyFormatter(key_from)
                                       << " to key "
                                       << gtsam::DefaultKeyFormatter(key_to)
                                       << " already exists.");
      return false;
    }
    edges_.insert(msg);
    edges_new_.insert(msg);
  }

  if (type == pose_graph_msgs::PoseGraphEdge::ODOM) {
    // Add to posegraph
    ROS_DEBUG_STREAM("Adding Odom edge for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to));
    nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_from, key_to, transform, covariance));
  } else if (type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
    ROS_DEBUG_STREAM("Adding loop closure edge for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to));
    nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_from, key_to, transform, covariance));
  } else if (type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
    ROS_DEBUG_STREAM("Adding artifact edge for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to));
    nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_from, key_to, transform, covariance));
  } else if (type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) {
    ROS_ERROR_STREAM("Cannot track UWB range factor for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to)
                     << " using PoseGraph::TrackFactor(). Use "
                        "PoseGraph::TrackUWBFactor() instead.");
    return false;
  } else if (type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN) {
    ROS_DEBUG_STREAM("Adding UWB between factor for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to));
    nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        key_from, key_to, transform, covariance));
  } else if (type == pose_graph_msgs::PoseGraphEdge::IMU) {
    ROS_ERROR_STREAM("Cannot track IMU range factor for key "
                     << gtsam::DefaultKeyFormatter(key_from) << " to key "
                     << gtsam::DefaultKeyFormatter(key_to)
                     << " using PoseGraph::TrackFactor(). Use "
                        "PoseGraph::TrackIMUFactor() instead.");
    return false;
  } else {
    ROS_DEBUG_STREAM("Cannot add edge of unknown type "
                     << type << " between keys "
                     << gtsam::DefaultKeyFormatter(key_from) << " and "
                     << gtsam::DefaultKeyFormatter(key_to) << ".");
  }
  return true;
}

bool PoseGraph::TrackUWBFactor(gtsam::Symbol key_from,
                               gtsam::Symbol key_to,
                               double range,
                               double range_error,
                               bool create_msg) {
  gtsam::noiseModel::Base::shared_ptr noise =
      gtsam::noiseModel::Isotropic::Sigma(1, range_error);
  if (create_msg) {
    auto msg = utils::GtsamToRosMsg(key_from,
                                    key_to,
                                    pose_graph_msgs::PoseGraphEdge::UWB_RANGE,
                                    gtsam::Pose3(),
                                    noise);

    if (edges_.find(msg) != edges_.end()) {
      ROS_DEBUG_STREAM("UWB range factor from key "
                       << gtsam::DefaultKeyFormatter(key_from) << " to key "
                       << gtsam::DefaultKeyFormatter(key_to)
                       << " already exists.");
      return false;
    }
    msg.range = range;
    msg.range_error = range_error;
    edges_.insert(msg);
    edges_new_.insert(msg);
  }

  nfg_.add(gtsam::RangeFactor<gtsam::Pose3, gtsam::Pose3>(
      key_from, key_to, range, noise));
  return true;
}

bool PoseGraph::TrackIMUFactor(gtsam::Symbol key_to,
                               geometry_msgs::Point meas,
                               geometry_msgs::Point ref,
                               double att_noise,
                               bool create_msg) {
  gtsam::noiseModel::Base::shared_ptr noise =
      gtsam::noiseModel::Isotropic::Sigma(2, att_noise);
  
  ROS_INFO_STREAM("TrackIMUFactor - CreateAttitudeFactor for key " << gtsam::DefaultKeyFormatter(key_to));
  gtsam::Unit3 ref_unit(ref.x, ref.y, ref.z); 
  gtsam::Unit3 meas_gt(meas.x, meas.y, meas.z);
  
  gtsam::Pose3AttitudeFactor factor(key_to, meas_gt, noise, ref_unit);
  
  if (create_msg) {
    auto msg = utils::GtsamToRosMsg(key_to,
                                    key_to,
                                    pose_graph_msgs::PoseGraphEdge::IMU,
                                    gtsam::Pose3(),
                                    noise);

    if (edges_.find(msg) != edges_.end()) {
      ROS_INFO_STREAM("IMU factor for key "
                       << gtsam::DefaultKeyFormatter(key_to)
                       << " already exists.");
      return false;
    }
    msg.pose.position = meas;
    // msg.covariance[0] = 
    edges_.insert(msg);
    edges_new_.insert(msg);
  }

  nfg_.add(factor);
  return true;
}

bool PoseGraph::TrackNode(const Node& node) {
  return TrackNode(node.stamp, node.key, node.pose, node.covariance);
}

bool PoseGraph::TrackNode(const NodeMessage& msg) {
  const auto pose = utils::MessageToPose(msg);
  Gaussian::shared_ptr noise = utils::MessageToCovariance(msg);

  // Track node without creating another node message
  if (!TrackNode(msg.header.stamp, gtsam::Symbol(msg.key), pose, noise, false))
    return false;

  // make copy to modify ID
  auto msg_found = nodes_.find(msg);
  if (msg_found == nodes_.end()) {
    NodeMessage m = msg;
    if (m.ID.empty() && !symbol_id_map.empty())
      m.ID = symbol_id_map(msg.key);
    nodes_.insert(m);
    nodes_new_.insert(m);
  } else {
    nodes_.erase(msg_found);
    nodes_.insert(msg);
  }
  return true;
}

bool PoseGraph::TrackNode(const ros::Time& stamp,
                          gtsam::Symbol key,
                          const gtsam::Pose3& pose,
                          const gtsam::SharedNoiseModel& covariance,
                          bool create_msg) {
  // TODO use covariance?

  if (values_.exists(key)) {
    values_.update(key, pose);
  } else {
    values_.insert(key, pose);
  }
  if (values_new_.exists(key)) {
    values_new_.update(key, pose);
  } else {
    values_new_.insert(key, pose);
  }
  keyed_stamps[key] = stamp;

  if (create_msg) {
    NodeMessage msg =
        utils::GtsamToRosMsg(stamp, fixed_frame_id, key, pose, covariance);
    // make copy to modify ID
    auto msg_found = nodes_.find(msg);
    if (msg_found == nodes_.end()) {
      NodeMessage m = msg;
      if (m.ID.empty() && !symbol_id_map.empty())
        m.ID = symbol_id_map(msg.key);
      nodes_.insert(m);
      nodes_new_.insert(m);
    } else {
      nodes_.erase(msg_found);
      nodes_.insert(msg);
    }
  }

  return true;
}

bool PoseGraph::TrackNode(const ros::Time& stamp,
                          gtsam::Symbol key,
                          const gtsam::Pose3& pose,
                          const gtsam::SharedNoiseModel& covariance,
                          const std::string id,
                          bool create_msg) {
  // TODO use covariance?

  if (values_.exists(key)) {
    values_.update(key, pose);
  } else {
    values_.insert(key, pose);
  }
  if (values_new_.exists(key)) {
    values_new_.update(key, pose);
  } else {
    values_new_.insert(key, pose);
  }
  keyed_stamps[key] = stamp;

  if (create_msg) {
    NodeMessage msg =
        utils::GtsamToRosMsg(stamp, fixed_frame_id, key, pose, covariance);
    msg.ID = id;
    // make copy to modify ID
    auto msg_found = nodes_.find(msg);
    if (msg_found == nodes_.end()) {
      NodeMessage m = msg;
      nodes_.insert(m);
      nodes_new_.insert(m);
    } else {
      nodes_.erase(msg_found);
      nodes_.insert(msg);
    }
  }

  return true;
}

bool PoseGraph::TrackPrior(const EdgeMessage& msg) {
  if (msg.type != pose_graph_msgs::PoseGraphEdge::PRIOR) {
    return TrackFactor(msg);
  }
  if (priors_.find(msg) != priors_.end()) {
    // prior already exists
    ROS_DEBUG_STREAM("Prior at key " << gtsam::DefaultKeyFormatter(msg.key_from)
                                     << " already exists.");
    return false;
  }

  // create precisions // TODO - the precision should come from the message
  // shouldn't it? As we will use priors for different applications
  // gtsam::Vector6 prior_precisions;
  // prior_precisions.head<3>().setConstant(0.0);
  // prior_precisions.tail<3>().setConstant(10.0);
  // // TODO(Yun) create parameter for this
  // static const gtsam::SharedNoiseModel& prior_noise =
  //     gtsam::noiseModel::Diagonal::Precisions(prior_precisions);

  gtsam::Pose3 delta = utils::MessageToPose(msg);
  Gaussian::shared_ptr noise = utils::MessageToCovariance(msg);

  if (!TrackPrior(gtsam::Symbol(msg.key_from), delta, noise, false))
    return false;

  priors_new_.insert(msg);
  priors_.insert(msg);
  return true;
}

bool PoseGraph::TrackPrior(const Factor& factor) {
  return TrackPrior(factor.key_from, factor.transform, factor.covariance);
}

bool PoseGraph::TrackPrior(const Node& node) {
  return TrackPrior(node.key, node.pose, node.covariance);
}

bool PoseGraph::TrackPrior(gtsam::Symbol key,
                           const gtsam::Pose3& pose,
                           const gtsam::SharedNoiseModel& covariance,
                           bool create_msg) {
  if (create_msg) {
    auto msg = utils::GtsamToRosMsg(
        key, key, pose_graph_msgs::PoseGraphEdge::PRIOR, pose, covariance);

    if (priors_.find(msg) != priors_.end()) {
      // prior already exists
      ROS_DEBUG_STREAM("Prior at key " << gtsam::DefaultKeyFormatter(key)
                                       << " already exists.");
      return false;
    }
    priors_new_.insert(msg);
    priors_.insert(msg);
  }
  ROS_DEBUG_STREAM("Adding prior factor for key "
                   << gtsam::DefaultKeyFormatter(key));
  gtsam::PriorFactor<gtsam::Pose3> factor(key, pose, covariance);
  nfg_.add(factor);
  return true;
}

// Removal tools 
void PoseGraph::RemoveRobotFromGraph(std::string robot_name){

  // Get Prefixes
  unsigned char prefix = utils::GetRobotPrefix(robot_name);
  unsigned char art_prefix = utils::GetArtifactPrefix(robot_name);

  ROS_WARN_STREAM("Removing all edges and nodes in the graph for robot: " << robot_name << " with prefix " << prefix);

  // Remove robot information
  RemoveEdgesWithPrefix(prefix);
  RemoveValuesWithPrefix(prefix);

  // Remove artifacts
  RemoveEdgesWithPrefix(art_prefix);
  RemoveValuesWithPrefix(art_prefix);
}

void PoseGraph::RemoveEdgesWithPrefix(unsigned char prefix){
  ROS_DEBUG("Removing edges msg");
  // Remove edge messages
  EdgeSet new_edges;
  auto e = edges_.begin();
  while (e != edges_.end()) {
    if (gtsam::Symbol(e->key_from).chr() != prefix &&
        gtsam::Symbol(e->key_to).chr() != prefix) {
      // Is an edge to remove
      new_edges.insert(*e);
    }
    e++;
  }
  edges_ = new_edges;

  // Remove prior messages
  EdgeSet new_priors;
  auto p = priors_.begin();
  while (p != priors_.end()) {
    if (gtsam::Symbol(p->key_from).chr() != prefix &&
        gtsam::Symbol(p->key_to).chr() != prefix) {
      // Is an edge to remove
      new_priors.insert(*p);
    }
    p++;
  }
  priors_ = new_priors;

  ROS_DEBUG("Removing edges gtsam");
  // Remove edge factors
  gtsam::NonlinearFactorGraph new_nfg; 
  auto f = nfg_.begin();
  while (f != nfg_.end()) {
    auto factor = *f;
    if (gtsam::Symbol(factor->front()).chr() != prefix &&
        gtsam::Symbol(factor->back()).chr() != prefix) {
      // If this factor does not connect to nodes with prefix
      // Add to new nonlinear factor graph
      new_nfg.add(factor);
    }
    f++;
  }
  nfg_ = new_nfg; 
}

void PoseGraph::RemoveValuesWithPrefix(unsigned char prefix){
  ROS_DEBUG("Removing values msg");
  // Remove edge messages

  NodeSet new_nodes;
  auto n = nodes_.begin();

  while (n != nodes_.end()) {
    if (gtsam::Symbol(n->key).chr() != prefix){
      // Is an edge to keep 
      new_nodes.insert(*n);
    } 
    n++;
  }

  nodes_ = new_nodes;

  ROS_DEBUG("Removing values gtsam");
  // Remove gtsam values
  gtsam::Values new_values;
  auto v = values_.begin();

  while (v != values_.end()) {
    auto value = *v;
    if (gtsam::Symbol(value.key).chr() != prefix){
      // Is an edge to keep
      new_values.insert(value.key, value.value);

      // Update the latest key
      key = value.key;
    } 
    v++;
  }

  values_ = new_values;

}

// DEPRECATED!!
void PoseGraph::AddNewValues(const gtsam::Values& new_values) {
  // Main values variable
  for (auto v : new_values) {
    if (!values_.tryInsert(v.key, v.value).second) {
      values_.update(v.key, v.value);
    }
    if (!values_new_.tryInsert(v.key, v.value).second) {
      values_new_.update(v.key, v.value);
    }
  }
}

// DEPRECATED!!
void PoseGraph::AddNewFactors(const gtsam::NonlinearFactorGraph& nfg) {
  nfg_.add(nfg);
}

void PoseGraph::Initialize(gtsam::Symbol initial_key,
                           const gtsam::Pose3& pose,
                           const Diagonal::shared_ptr& covariance) {
  nfg_ = gtsam::NonlinearFactorGraph();
  values_ = gtsam::Values();

  b_first_ = true;

  Node prior;
  prior.stamp = ros::Time::now();
  ROS_INFO_STREAM("Initial node timestamp is " << prior.stamp.toSec());
  prior.key = initial_key;
  prior.pose = pose;
  prior.covariance = covariance;

  TrackPrior(prior);
  TrackNode(prior);
}

void PoseGraph::InsertKeyedScan(gtsam::Symbol key,
                                const PointCloud::ConstPtr& scan) {
  keyed_scans.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));
}

void PoseGraph::InsertKeyedStamp(gtsam::Symbol key, const ros::Time& stamp) {
  keyed_stamps.insert(std::pair<gtsam::Symbol, ros::Time>(key, stamp));
}

void PoseGraph::InsertStampedOdomKey(double seconds, gtsam::Symbol key) {
  stamp_to_odom_key.insert(std::pair<double, gtsam::Symbol>(seconds, key));
}
