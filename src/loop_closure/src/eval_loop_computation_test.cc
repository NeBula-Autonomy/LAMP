/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/IcpLoopComputation.h>
#include <loop_closure/TestUtils.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <utils/CommonFunctions.h>
#include <utils/CommonStructs.h>
#include <utils/PointCloudUtils.h>

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

    if (!pu::Get("filtering/adaptive_filter", filter_params_.adaptive_filter))
      return false;
    if (!pu::Get("filtering/adaptive_target", filter_params_.adaptive_target))
      return false;
    if (!pu::Get("filtering/adaptive_max_grid",
                 filter_params_.adaptive_max_grid))
      return false;
    if (!pu::Get("filtering/adaptive_min_grid",
                 filter_params_.adaptive_min_grid))
      return false;

    if (!pu::Get("normals_computation/method",
                 normals_compute_params_.search_method))
      return false;
    if (!pu::Get("normals_computation/k", normals_compute_params_.k))
      return false;
    if (!pu::Get("normals_computation/radius", normals_compute_params_.radius))
      return false;
    if (!pu::Get("normals_computation/num_threads",
                 normals_compute_params_.num_threads))
      return false;

    if (!icp_lc_.LoadParameters(n))
      return false;

    if (icp_lc_.number_of_threads_in_icp_computation_pool_ > 1) {
      ROS_INFO_STREAM("Thread Pool Initialized with "
                      << icp_lc_.number_of_threads_in_icp_computation_pool_
                      << " threads");
      icp_lc_.icp_computation_pool_.resize(
          icp_lc_.number_of_threads_in_icp_computation_pool_);
    }

    min_ks_size_ = std::numeric_limits<size_t>::max();
    max_ks_size_ = 0;
    return true;
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

      if (ks_msg->scan.width < min_ks_size_)
        min_ks_size_ = ks_msg->scan.width;
      if (ks_msg->scan.width > max_ks_size_)
        max_ks_size_ = ks_msg->scan.width;

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

  void ClearOutput() {
    icp_lc_.output_queue_.clear();
  }

  void AdaptiveFilter(const double& target_pt_size,
                      const double& init_leaf_size,
                      const double& min_leaf_size,
                      const double& max_leaf_size,
                      const PointCloud& original_cloud,
                      PointCloud::Ptr new_cloud) {
    if (original_cloud.size() < target_pt_size) {
      *new_cloud = original_cloud;
      return;
    }
    double leaf_size = init_leaf_size;

    *new_cloud = original_cloud;
    Eigen::Matrix<double, 3, 1> obs_eigenv;
    utils::ComputeIcpObservability(new_cloud, &obs_eigenv);
    double prev_observability =
        obs_eigenv.minCoeff() / static_cast<double>(new_cloud->size());
    int count = 0;
    while (abs(new_cloud->size() - target_pt_size) > 10 && count < 100) {
      *new_cloud = original_cloud;
      pcl::VoxelGrid<Point> grid;
      grid.setLeafSize(leaf_size, leaf_size, leaf_size);
      grid.setInputCloud(new_cloud);
      grid.filter(*new_cloud);

      utils::ComputeIcpObservability(new_cloud, &obs_eigenv);
      double observability =
          obs_eigenv.minCoeff() / static_cast<double>(new_cloud->size());

      double size_factor = static_cast<double>(new_cloud->size()) /
          static_cast<double>(target_pt_size);
      double obs_factor = prev_observability / observability;

      leaf_size = std::min(
          max_leaf_size,
          std::max(min_leaf_size, leaf_size * size_factor * obs_factor));

      count++;
    }
  }

  void AdaptiveRandomFilter(const double& target_pt_size,
                            const double& obs_epsilon,
                            const PointCloud& original_cloud,
                            PointCloud::Ptr new_cloud) {
    if (original_cloud.size() < target_pt_size) {
      *new_cloud = original_cloud;
      return;
    }
    *new_cloud = original_cloud;
    Eigen::Matrix<double, 3, 1> obs_eigenv;
    utils::ComputeIcpObservability(new_cloud, &obs_eigenv);
    double prev_observability =
        obs_eigenv.minCoeff() / static_cast<double>(new_cloud->size());
    int count = 0;
    int decrement = (original_cloud.size() - target_pt_size) / 100;
    while (new_cloud->size() > target_pt_size && count < 100) {
      const int n_points = new_cloud->size() - decrement;
      pcl::RandomSample<Point> random_filter;
      random_filter.setSample(n_points);
      random_filter.setInputCloud(new_cloud);
      random_filter.filter(*new_cloud);

      Eigen::Matrix<double, 3, 1> obs_eigenv;
      utils::ComputeIcpObservability(new_cloud, &obs_eigenv);
      double observability =
          obs_eigenv.minCoeff() / static_cast<double>(new_cloud->size());

      if (prev_observability - observability > obs_epsilon)
        break;

      prev_observability = observability;
      count++;
    }
  }

  void FilterKeyedScans(const pose_graph_msgs::KeyedScan& original_ks,
                        pose_graph_msgs::KeyedScan::Ptr new_ks) {
    PointCloud::Ptr new_scan(new PointCloud);
    PointCloud adaptive_input;
    pcl::fromROSMsg(original_ks.scan, adaptive_input);
    // Filter and publish scan
    // Adaptive filter
    if (filter_params_.adaptive_filter) {
      AdaptiveFilter(filter_params_.adaptive_target,
                     0.25,
                     filter_params_.adaptive_min_grid,
                     filter_params_.adaptive_max_grid,
                     adaptive_input,
                     new_scan);
    } else {
      *new_scan = adaptive_input;
    }

    // Apply random downsampling to the keyed scan
    if (filter_params_.random_filter) {
      const int n_points = static_cast<int>(
          (1.0 - filter_params_.decimate_percentage) * new_scan->size());
      pcl::RandomSample<Point> random_filter;
      random_filter.setSample(n_points);
      random_filter.setInputCloud(new_scan);
      random_filter.filter(*new_scan);
      // AdaptiveRandomFilter(n_points, 0.1, *new_scan, new_scan);
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

    // Remove normals
    PointXyziCloud::Ptr no_normals_scan(new PointXyziCloud);
    utils::ConvertPointCloud(new_scan, no_normals_scan);

    // Recompute normals
    utils::AddNormals(no_normals_scan, normals_compute_params_, new_scan);

    pcl::toROSMsg(*new_scan, new_ks->scan);
    new_ks->key = original_ks.key;
  }

  inline size_t maxKsSize() {
    return max_ks_size_;
  }

  inline size_t minKsSize() {
    return min_ks_size_;
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
    // Adaptive filter
    bool adaptive_filter;
    // Adaptive point size
    int adaptive_target;
    // Adaptive constraint
    double adaptive_max_grid;
    double adaptive_min_grid;
  } filter_params_;

  size_t max_ks_size_;
  size_t min_ks_size_;

  utils::NormalComputeParams normals_compute_params_;
};

} // namespace lamp_loop_closure

int main(int argc, char** argv) {
  ros::init(argc, argv, "eval_loop_computation_test");
  ros::start();
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
  auto start = std::chrono::high_resolution_clock::now();
  // Compute
  evaluate.ComputeLoopClosures();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start) /
      1000;

  // Get the computed loop closures
  std::vector<pose_graph_msgs::PoseGraphEdge> results =
      evaluate.GetLoopClosures();

  // Clear output
  evaluate.ClearOutput();

  // Test false candidates
  evaluate.AddLoopCandidates(test_data.fake_candidates_);
  evaluate.ComputeLoopClosures();

  std::vector<pose_graph_msgs::PoseGraphEdge> false_results =
      evaluate.GetLoopClosures();

  ROS_INFO("%d seconds to complete lc analysis: ", duration);
  ROS_INFO("Max keyed scan size: %d and Min keyed scan size: %d",
           evaluate.maxKsSize(),
           evaluate.minKsSize());
  ROS_INFO("Detected %d loop closures.", results.size());
  ROS_INFO("Detected %d incorrect loop closures.", false_results.size());

  tu::OutputTestSummary(
      test_data, results, false_results, output_dir, test_name);

  return EXIT_SUCCESS;
}
