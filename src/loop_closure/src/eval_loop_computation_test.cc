/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/IcpLoopComputation.h>
#include <loop_closure/TestUtils.h>
#include <ros/ros.h>
#include <utils/CommonFunctions.h>

namespace tu = test_utils;

namespace lamp_loop_closure {
class EvalIcpLoopCompute {
public:
  bool LoadParameters(const ros::NodeHandle& n) {
    return icp_lc_.LoadParameters(n);
  }

  void
  AddLoopCandidates(const pose_graph_msgs::LoopCandidateArray& candidates) {
    pose_graph_msgs::LoopCandidateArray::Ptr input(
        new pose_graph_msgs::LoopCandidateArray(candidates));
    icp_lc_.InputCallback(input);
    return;
  }

  void
  AddKeyedScans(const std::vector<pose_graph_msgs::KeyedScan>& keyed_scans) {
    for (auto ks : keyed_scans) {
      pose_graph_msgs::KeyedScan::Ptr ks_msg(
          new pose_graph_msgs::KeyedScan(ks));
      icp_lc_.KeyedScanCallback(ks_msg);
    }
  }

  void AddKeyedPoses(const std::vector<gtsam::Pose3>& keyed_poses) {
    pose_graph_msgs::PoseGraph::Ptr pg(new pose_graph_msgs::PoseGraph);
    for (size_t i = 0; i < keyed_poses.size(); i++) {
      pose_graph_msgs::PoseGraphNode node;
      node.key = i;
      node.pose = utils::GtsamToRosMsg(keyed_poses.at(i));
      pg->nodes.push_back(node);
    }

    icp_lc_.KeyedPoseCallback(pg);
  }

  void ComputeLoopClosures() {
    icp_lc_.ComputeTransforms();
  }

  std::vector<pose_graph_msgs::PoseGraphEdge> GetLoopClosures() {
    return icp_lc_.output_queue_;
  }

protected:
  IcpLoopComputation icp_lc_;
};

} // namespace lamp_loop_closure

int main(int argc, char** argv) {
  ros::init(argc, argv, "eval_loop_computation_test");
  ros::NodeHandle n("~");

  std::string dataset_path, output_csv;
  n.getParam("dataset_path", dataset_path);
  n.getParam("output_csv", output_csv);
  // Load dataset
  tu::TestData test_data;
  ROS_INFO("Loading dataset from %s ...", dataset_path.c_str());
  if (!LoadExistingTestData(dataset_path, &test_data)) {
    ROS_ERROR("Failed to load dataset. ");
    return EXIT_FAILURE;
  }
  ROS_INFO("Loaded dataset with %d candidates.",
           test_data.test_candidates_.candidates.size());

  lamp_loop_closure::EvalIcpLoopCompute evaluate;
  if (!evaluate.LoadParameters(n)) {
    ROS_ERROR("Failed to load parameters for EvalIcpLoopCompute class. ");
    return EXIT_FAILURE;
  }

  // Add the keyed scans and keyed poses
  evaluate.AddKeyedScans(test_data.keyed_scans_);
  evaluate.AddKeyedPoses(test_data.gt_keyed_poses_);

  // Now add the candidates
  evaluate.AddLoopCandidates(test_data.test_candidates_);

  ROS_INFO("Computing loop closures... ");
  // Compute
  evaluate.ComputeLoopClosures();

  // Get the computed loop closures
  std::vector<pose_graph_msgs::PoseGraphEdge> results =
      evaluate.GetLoopClosures();

  ROS_INFO("Detected %d loop closures.", results.size());

  tu::OutputTestSummary(test_data, results, output_csv);

  return EXIT_SUCCESS;
}
