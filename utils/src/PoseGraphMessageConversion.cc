#include "utils/CommonFunctions.h"
#include "utils/PoseGraph.h"

#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;

GraphMsgPtr PoseGraph::ToMsg() const {
  return ToMsg_(edges_, nodes_, priors_);
}

GraphMsgPtr PoseGraph::ToIncrementalMsg() const {
  return ToMsg_(edges_new_, nodes_new_, priors_new_);
}

GraphMsgPtr PoseGraph::ToMsg_(const EdgeSet& edges,
                              const NodeSet& nodes,
                              const EdgeSet& priors) const {
  // Create the Pose Graph Message
  auto* msg = new pose_graph_msgs::PoseGraph;
  msg->header.frame_id = fixed_frame_id;
  // Set timestamp to now
  msg->header.stamp = ros::Time::now();
  // // Get timestamp from latest keyed pose
  // if (HasStamp(key - 1))
  //   msg->header.stamp = keyed_stamps.at(key - 1);
  // else {
  //   ROS_WARN_STREAM("No time stamp exists for latest key ("
  //                   << (key - 1)
  //                   << ") while converting pose graph to message.");
  // }

  msg->nodes.reserve(nodes.size());
  for (const auto& node : nodes)
    msg->nodes.emplace_back(node);

  // Add the factors  // TODO: check integration of this tracking with all
  // handlers
  msg->edges.reserve(edges.size() + priors.size());
  for (const auto& edge : edges)
    msg->edges.emplace_back(edge);
  for (const auto& prior : priors)
    msg->edges.emplace_back(prior);

  return GraphMsgPtr(msg);
}

void PoseGraph::UpdateFromMsg(const GraphMsgPtr& msg) {
  for (const auto& edge : msg->edges) {
    TrackFactor(edge);
  }
  for (const auto& node : msg->nodes) {
    TrackNode(node);
  }
}

void PoseGraph::AddAllValuesToNew() {
  values_new_ = values_;
}

EdgeMessage Factor::ToMsg() const {
  return utils::GtsamToRosMsg(key_from, key_to, type, transform, covariance);
}

Factor Factor::FromMsg(const EdgeMessage& msg) {
  Factor factor;
  factor.type = msg.type;
  factor.key_from = gtsam::Symbol(msg.key_from);
  factor.key_to = gtsam::Symbol(msg.key_to);
  factor.transform = utils::MessageToPose(msg);
  factor.covariance = utils::MessageToCovariance(msg);
  return factor;
}

NodeMessage Node::ToMsg() const {
  NodeMessage msg =
      utils::GtsamToRosMsg(stamp, fixed_frame_id, key, pose, covariance);
  // Determine internal ID from pose graph
  if (graph && !graph->symbol_id_map.empty())
    msg.ID = graph->symbol_id_map(key);
  return msg;
}