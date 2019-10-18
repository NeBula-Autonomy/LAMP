#include <pose_graph_serializer/serializer.h>

#include <parameter_utils/ParameterUtils.h>
#include <utils/CommonFunctions.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

GraphPtr PoseGraphSerializer::ConvertToMsg(const PoseGraph& pose_graph) {
  // Create the Pose Graph Message
  auto* msg = new pose_graph_msgs::PoseGraph;
  msg->header.frame_id = pose_graph.fixed_frame_id;
  // TODO: get time stamp?
  msg->header.stamp =
      pose_graph.keyed_stamps.at(pose_graph.key - 1); // Get timestamp from latest keyed pose

  // Get Values
  // Converts the internal values
  for (const auto& keyed_pose : pose_graph.values) {
    gu::Transform3 t = utils::ToGu(pose_graph.values.at<gtsam::Pose3>(keyed_pose.key));

    gtsam::Symbol sym_key = gtsam::Symbol(keyed_pose.key);

    // Populate the message with the pose's data.
    GraphNode node;
    node.key = keyed_pose.key;
    node.header.frame_id = pose_graph.fixed_frame_id;
    node.pose = gr::ToRosPose(t);

    // Get timestamp
    // Note keyed_stamps are for all nodes TODO: check this is followed
    // TODO: check if time stamps are necessary
    node.header.stamp = pose_graph.keyed_stamps.at(keyed_pose.key);

    // Get the IDs
    if (pose_graph.keyed_scans.count(keyed_pose.key)) {
      // Key frame, note in the ID
      node.ID = "key_frame";
    } else if (sym_key.chr() == pose_graph.prefix[0]) {
      // Odom or key frame
      node.ID = "odom";
    } else if (sym_key.chr() == 'u') {
      // UWB
      // node.ID = uwd_handler_.GetUWBID(keyed_pose.key); // TODO
      node.ID = "UWB"; // TEMPORARY
    } else {
      // Artifact
      // node.ID = artifact_handler_.GetArtifactID(keyed_pose.key);// TODO
      node.ID = "Artifact"; // TEMPORARY
    }

    // TODO: fill covariance

    // Add to the vector of nodes
    msg->nodes.push_back(node);
  }

  // Add the factors  // TODO: check integration of this tracking with all
  // handlers
  msg->edges = pose_graph.edges;
  msg->priors = pose_graph.priors;

  return GraphPtr(msg);
}

bool ConvertFromMsg(const GraphPtr& msg,
                    PoseGraph* pose_graph) {
  return true;
}