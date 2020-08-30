#include "utils/CommonFunctions.h"
#include "utils/CommonStructs.h"

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

// Pose graph msg to gtsam conversion
// TODO remove this
void utils::PoseGraphMsgToGtsam(const GraphMsgPtr& graph_msg,
                                gtsam::NonlinearFactorGraph* graph_nfg,
                                gtsam::Values* graph_vals) {
  using gtsam::BetweenFactor;
  using gtsam::PriorFactor;
  using gtsam::RangeFactor;

  *graph_nfg = gtsam::NonlinearFactorGraph();
  *graph_vals = gtsam::Values();

  for (const auto& msg_edge : graph_msg->edges) {
    // gtsam::Point3 delta_translation(msg_edge.pose.position.x,
    //                                 msg_edge.pose.position.y,
    //                                 msg_edge.pose.position.z);
    // gtsam::Rot3 delta_orientation(
    //     gtsam::Rot3::quaternion(msg_edge.pose.orientation.w,
    //                             msg_edge.pose.orientation.x,
    //                             msg_edge.pose.orientation.y,
    //                             msg_edge.pose.orientation.z));
    // gtsam::Pose3 delta = gtsam::Pose3(delta_orientation, delta_translation);

    gtsam::Pose3 delta = utils::MessageToPose(msg_edge);

    Gaussian::shared_ptr noise = utils::MessageToCovariance(msg_edge);

    // // TODO(Yun) fill in covariance
    // gtsam::Matrix66 covariance;
    // for (size_t i = 0; i < msg_edge.covariance.size(); i++) {
    //   size_t row = static_cast<size_t>(i / 6);
    //   size_t col = i % 6;
    //   covariance(row, col) = msg_edge.covariance[i];
    // }
    // Gaussian::shared_ptr noise = Gaussian::Covariance(covariance);
    //-------------------------------------------------------------------------

    if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ODOM) {
      // Add to posegraph
      ROS_DEBUG_STREAM("Adding Odom edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(
          BetweenFactor<gtsam::Pose3>(gtsam::Symbol(msg_edge.key_from),
                                      gtsam::Symbol(msg_edge.key_to),
                                      delta,
                                      noise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
      ROS_DEBUG_STREAM("Adding loop closure edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(
          BetweenFactor<gtsam::Pose3>(gtsam::Symbol(msg_edge.key_from),
                                      gtsam::Symbol(msg_edge.key_to),
                                      delta,
                                      noise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      ROS_DEBUG_STREAM("Adding artifact edge for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(
          BetweenFactor<gtsam::Pose3>(gtsam::Symbol(msg_edge.key_from),
                                      gtsam::Symbol(msg_edge.key_to),
                                      delta,
                                      noise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) {
      ROS_DEBUG_STREAM("Adding UWB range factor for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      double range = msg_edge.range;
      double sigmaR = msg_edge.range_error;
      gtsam::noiseModel::Base::shared_ptr rangeNoise =
          gtsam::noiseModel::Isotropic::Sigma(1, sigmaR);
      graph_nfg->add(RangeFactor<gtsam::Pose3, gtsam::Pose3>(
          gtsam::Symbol(msg_edge.key_from),
          gtsam::Symbol(msg_edge.key_to),
          range,
          rangeNoise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN) {
      ROS_DEBUG_STREAM("Adding UWB between factor for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from)
                       << " to key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      graph_nfg->add(
          BetweenFactor<gtsam::Pose3>(gtsam::Symbol(msg_edge.key_from),
                                      gtsam::Symbol(msg_edge.key_to),
                                      delta,
                                      noise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::PRIOR) {
      Gaussian::shared_ptr prior_noise = utils::MessageToCovariance(msg_edge);

      ROS_DEBUG_STREAM("Adding prior factor for key "
                       << gtsam::DefaultKeyFormatter(msg_edge.key_from));
      graph_nfg->add(gtsam::PriorFactor<gtsam::Pose3>(
          gtsam::Symbol(msg_edge.key_from), delta, prior_noise));
    } 
    
    else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::IMU) {
      ROS_DEBUG_STREAM("Adding prior factor for IMU");

      gtsam::Unit3 meas_gt(msg_edge.pose.position.x, msg_edge.pose.position.y, msg_edge.pose.position.z);

      // Use top left covariance matrix element only (assume uniform noise)
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Isotropic::Sigma(2, msg_edge.covariance[0]);

      gtsam::Unit3 ref(0, 0, 1);
      graph_nfg->add(gtsam::Pose3AttitudeFactor(msg_edge.key_to, ref, noise, meas_gt));

    }
  }

  //-----------------------------------------------//
  // Add the values
  for (const pose_graph_msgs::PoseGraphNode& msg_node : graph_msg->nodes) {
    gtsam::Pose3 full_pose;
    gtsam::Point3 pose_translation(msg_node.pose.position.x,
                                   msg_node.pose.position.y,
                                   msg_node.pose.position.z);
    gtsam::Rot3 pose_orientation(
        gtsam::Rot3::quaternion(msg_node.pose.orientation.w,
                                msg_node.pose.orientation.x,
                                msg_node.pose.orientation.y,
                                msg_node.pose.orientation.z));
    full_pose = gtsam::Pose3(pose_orientation, pose_translation);

    gtsam::Key key = gtsam::Key(msg_node.key);
    graph_vals->insert(key, full_pose);
  }

  // TODO right now this only accounts for translation part of prior (see
  // for (const pose_graph_msgs::PoseGraphNode& msg_prior : graph_msg->priors) {
  //   ROS_DEBUG_STREAM("Adding prior in pose graph callback for key "
  //                    << gtsam::DefaultKeyFormatter(msg_prior.key));
  //   // create the prior factor

  //   // create precisions // TODO - the precision should come from the message
  //   // shouldn't it? As we will use priors for different applications
  //   gtsam::Vector6 prior_precisions;
  //   prior_precisions.head<3>().setConstant(0.0);
  //   prior_precisions.tail<3>().setConstant(10.0);
  //   // TODO(Yun) create parameter for this
  //   static const gtsam::SharedNoiseModel& prior_noise =
  //       gtsam::noiseModel::Diagonal::Precisions(prior_precisions);

  //   gtsam::Point3 pose_translation(msg_prior.pose.position.x,
  //                                  msg_prior.pose.position.y,
  //                                  msg_prior.pose.position.z);
  //   gtsam::Rot3 pose_orientation(
  //       gtsam::Rot3::quaternion(msg_prior.pose.orientation.w,
  //                               msg_prior.pose.orientation.x,
  //                               msg_prior.pose.orientation.y,
  //                               msg_prior.pose.orientation.z));
  //   gtsam::Pose3 prior_pose = gtsam::Pose3(pose_orientation,
  //   pose_translation);

  //   graph_nfg->add(
  //       PriorFactor<gtsam::Pose3>(msg_prior.key, prior_pose, prior_noise));
  // }
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