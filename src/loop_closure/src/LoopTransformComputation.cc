/**
 * @file   LoopTransformComputation.cc
 * @brief  Base class for classes to find transform of loop closures
 * @author Yun Chang
 */
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopTransformComputation.h"

namespace lamp_loop_closure {

LoopTransformComputation::LoopTransformComputation() {}
LoopTransformComputation::~LoopTransformComputation() {}

bool LoopTransformComputation::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);  // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool LoopTransformComputation::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_closure_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("loop_closures", 10, false);
  return true;
}

bool LoopTransformComputation::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "loop_candidates", 100, &LoopTransformComputation::InputCallback, this);
  return true;
}

void LoopTransformComputation::PublishLoopClosures() {
  pose_graph_msgs::PoseGraph loop_closures_msg;
  loop_closures_msg.edges = output_queue_;
  loop_closure_pub_.publish(loop_closures_msg);
  output_queue_.clear();
}

void LoopTransformComputation::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  for (auto candidate : input_candidates->candidates) {
    input_queue_.push(candidate);
  }
  return;
}

pose_graph_msgs::PoseGraphEdge LoopTransformComputation::CreateLoopClosureEdge(
    const gtsam::Symbol& key1,
    const gtsam::Symbol& key2,
    const geometry_utils::Transform3& delta,
    const gtsam::Matrix66& covariance) const {
  // Create the new loop closure edge
  pose_graph_msgs::PoseGraphEdge edge;
  edge.key_from = key1;
  edge.key_to = key2;
  edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;
  edge.pose = gr::ToRosPose(delta);

  // Convert matrix covariance to vector
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      edge.covariance[6 * i + j] = covariance(i, j);
    }
  }

  return edge;
}

}  // namespace lamp_loop_closure