/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/IcpLoopComputation.h>
#include <loop_closure/TestUtils.h>
#include <parameter_utils/ParameterUtils.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <ros/ros.h>
#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>

namespace tu = test_utils;
namespace pu = parameter_utils;

namespace lamp_loop_closure {
class EvalIcpLoopCompute {
public:
  bool LoadParameters(const ros::NodeHandle& n) {
    // Load filtering parameters.
    if (!pu::Get("filtering/grid_filter", filter_params_.grid_filter))
      return false;
    if (!pu::Get("filtering/grid_res", filter_params_.grid_res))
      return false;
    if (!pu::Get("filtering/random_filter", filter_params_.random_filter))
      return false;
    if (!pu::Get("filtering/decimate_percentage",
                 filter_params_.decimate_percentage))
      return false;
    // Cap to [0.0, 1.0].
    filter_params_.decimate_percentage =
        std::min(1.0, std::max(0.0, filter_params_.decimate_percentage));

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
      pose_graph_msgs::KeyedScan::Ptr ks_msg(new pose_graph_msgs::KeyedScan);
      FilterKeyedScans(ks, ks_msg);
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

  void FilterKeyedScans(const pose_graph_msgs::KeyedScan& original_ks,
                        pose_graph_msgs::KeyedScan::Ptr new_ks) {
    PointCloud::Ptr new_scan(new PointCloud);
    pcl::fromROSMsg(original_ks.scan, *new_scan);
    // Filter and publish scan
    const int n_points = static_cast<int>(
        (1.0 - filter_params_.decimate_percentage) * new_scan->size());

    // // Apply random downsampling to the keyed scan
    if (filter_params_.random_filter) {
      pcl::RandomSample<Point> random_filter;
      random_filter.setSample(n_points);
      random_filter.setInputCloud(new_scan);
      random_filter.filter(*new_scan);
    }

    // Apply voxel grid filter to the keyed scan
    if (filter_params_.grid_filter) {
      // TODO - have option to turn on and off keyed scans
      pcl::VoxelGrid<Point> grid;
      grid.setLeafSize(filter_params_.grid_res,
                       filter_params_.grid_res,
                       filter_params_.grid_res);
      grid.setInputCloud(new_scan);
      grid.filter(*new_scan);
    }

    pcl::toROSMsg(*new_scan, new_ks->scan);
    new_ks->key = original_ks.key;
  }

protected:
  IcpLoopComputation icp_lc_;
  struct FilterParams {
    // Voxel grid filter.
    bool grid_filter;
    // Resolution of voxel grid filter.
    double grid_res;
    // Random downsampling filter.
    bool random_filter;
    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;
  } filter_params_;
};

} // namespace lamp_loop_closure

int main(int argc, char** argv) {
  ros::init(argc, argv, "eval_loop_computation_test");
  ros::NodeHandle n("~");

  std::string dataset_path, output_dir, test_name;
  n.getParam("dataset_path", dataset_path);
  n.getParam("output_dir", output_dir);
  n.getParam("test_name", test_name);
  bool use_gt_odom;
  n.getParam("use_gt_odom", use_gt_odom);
  // Load dataset
  tu::TestData test_data;
  ROS_INFO("Loading dataset from %s ...", dataset_path.c_str());
  if (!LoadExistingTestData(dataset_path, &test_data)) {
    ROS_ERROR("Failed to load dataset. ");
    return EXIT_FAILURE;
  }
  ROS_INFO("Loaded dataset with %d candidates.",
           test_data.real_candidates_.candidates.size());

  lamp_loop_closure::EvalIcpLoopCompute evaluate;
  if (!evaluate.LoadParameters(n)) {
    ROS_ERROR("Failed to load parameters for EvalIcpLoopCompute class. ");
    return EXIT_FAILURE;
  }

  // Add the keyed scans and keyed poses
  evaluate.AddKeyedScans(test_data.keyed_scans_);
  if (use_gt_odom) {
    evaluate.AddKeyedPoses(test_data.gt_keyed_poses_);
  } else {
    evaluate.AddKeyedPoses(test_data.odom_keyed_poses_);
  }

  // Now add the candidates
  evaluate.AddLoopCandidates(test_data.real_candidates_);

  ROS_INFO("Computing loop closures... ");
  // Compute
  evaluate.ComputeLoopClosures();

  // Get the computed loop closures
  std::vector<pose_graph_msgs::PoseGraphEdge> results =
      evaluate.GetLoopClosures();

  ROS_INFO("Detected %d loop closures.", results.size());

  tu::OutputTestSummary(test_data, results, output_dir, test_name);

  return EXIT_SUCCESS;
}
