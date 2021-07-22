/**
 * @file   LoopComputation.cc
 * @brief  Base class for classes to find transform of loop closures
 * @author Yun Chang
 */
#include <utils/CommonFunctions.h>

#include "loop_closure/LoopComputation.h"

namespace lamp_loop_closure {

LoopComputation::LoopComputation() {}
LoopComputation::~LoopComputation() {}

bool LoopComputation::LoadParameters(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n); // Nodehandle for subscription/publishing
  param_ns_ = utils::GetParamNamespace(n.getNamespace());
  return true;
}

bool LoopComputation::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_closure_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("loop_closures", 10, false);
  status_pub_ = nl.advertise<pose_graph_msgs::LoopComputationStatus>("loop_computation_status", 10, false);
  return true;
}

void LoopComputation::PublishCompletedAllStatus() {
  pose_graph_msgs::LoopComputationStatus status;
  status.type = status.COMPLETED_ALL;
  status_pub_.publish(status);
}

bool LoopComputation::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  loop_candidate_sub_ = nl.subscribe<pose_graph_msgs::LoopCandidateArray>(
      "prioritized_loop_candidates", 100, &LoopComputation::InputCallback, this);

  return true;
}

std::vector<pose_graph_msgs::PoseGraphEdge> LoopComputation::GetCurrentOutputQueue(){
    return output_queue_;
}

void LoopComputation::PublishLoopClosures() {
  if (loop_closure_pub_.getNumSubscribers() > 0) {
    pose_graph_msgs::PoseGraph loop_closures_msg;
    loop_closures_msg.edges = output_queue_;
    loop_closure_pub_.publish(loop_closures_msg);
    output_queue_.clear();
    PublishCompletedAllStatus();
  }
}


void LoopComputation::InputCallback(
    const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates) {
  for (auto candidate : input_candidates->candidates) {
    input_queue_.push(candidate);
  }
  return;
}

pose_graph_msgs::PoseGraphEdge LoopComputation::CreateLoopClosureEdge(
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


} // namespace lamp_loop_closure