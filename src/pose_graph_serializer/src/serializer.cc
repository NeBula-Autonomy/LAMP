#include <pose_graph_serializer/serializer.h>

namespace gu = geometry_utils;

bool PoseGraphSerializer::ConvertValuesToNodeMsgs(
    const gtsam::Values& values,
    std::vector<pose_graph_msgs::PoseGraphNode>* nodes) {
  // Converts the internal values
  for (const auto& keyed_pose : values) {
    gu::Transform3 t = utils::ToGu(values.at<gtsam::Pose3>(keyed_pose.key));

    gtsam::Symbol sym_key = gtsam::Symbol(keyed_pose.key);

    // Populate the message with the pose's data.
    pose_graph_msgs::PoseGraphNode node;
    node.key = keyed_pose.key;
    node.header.frame_id = fixed_frame_id_;
    node.pose = gr::ToRosPose(t);

    // Get timestamp
    // Note keyed_stamps are for all nodes TODO - check this is followed
    /// TODO check if time stamps are necessary
    // node.header.stamp = keyed_stamps_[keyed_pose.key];

    // Get the IDs
    if (keyed_scans_.count(keyed_pose.key)) {
      // Key frame, note in the ID
      node.ID = "key_frame";
    } else if (sym_key.chr() == prefix_[0]) {
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

    // TODO - fill covariance

    // Add to the vector of nodes
    nodes->push_back(node);
  }
  return true;
}

GraphPtr PoseGraphSerializer::ConvertToMsg(const gtsam::Values& values,
                                           const EdgeMessages& edges_info,
                                           const NodeMessages& priors_info) {
  // Create the Pose Graph Message
  auto* msg = new pose_graph_msgs::PoseGraph;
  msg->header.frame_id = fixed_frame_id_;
  msg->header.stamp =
      keyed_stamps_[key_ - 1]; // Get timestamp from latest keyed pose

  // Get Values
  ConvertValuesToNodeMsgs(values, &msg->nodes);

  // Add the factors  // TODO - check integration of this tracking with all
  // handlers
  msg->edges = edges_info;
  msg->priors = priors_info;

  return pose_graph_msgs::PoseGraphConstPtr(msg);
}
