/**
 * @file   IcpLoopComputation.h
 * @brief  Find transform of loop closures via ICP
 * @author Yun Chang
 */
#pragma once

#include "ThreadPool.h"
#include "loop_closure/PointCloudUtils.h"
#include <geometry_utils/GeometryUtils.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <multithreaded_gicp/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <unordered_map>
#include <utils/CommonStructs.h>

#include "loop_closure/LoopComputation.h"

namespace lamp_loop_closure {

class IcpLoopComputation : public LoopComputation {
  typedef pcl::PointCloud<pcl::Normal> Normals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> Features;
  typedef pcl::search::KdTree<Point> KdTree;
  friend class TestLoopComputation;
  friend class EvalIcpLoopCompute;

public:
  IcpLoopComputation();
  ~IcpLoopComputation();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

  // Compute transform and populate output queue
  void ComputeTransforms() override;

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);

  void KeyedPoseCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

  void ProcessTimerCallback(const ros::TimerEvent& ev);

  bool SetupICP(pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point>& icp);

  bool PerformAlignment(const gtsam::Symbol& key1,
                        const gtsam::Symbol& key2,
                        const gtsam::Pose3& pose1,
                        const gtsam::Pose3& pose2,
                        geometry_utils::Transform3* delta,
                        gtsam::Matrix66* covariance,
                        double* fitness_score,
                        bool re_initialize_icp = false);

  void GetSacInitialAlignment(PointCloud::ConstPtr source,
                              PointCloud::ConstPtr target,
                              Eigen::Matrix4f* tf_out,
                              double& sac_fitness_score);

  void GetTeaserInitialAlignment(PointCloud::ConstPtr source,
                                 PointCloud::ConstPtr target,
                                 Eigen::Matrix4f* tf_out,
                                 int& n_inliers);

  bool
  ComputeICPCovariancePointPlane(const PointCloud::ConstPtr& query_cloud,
                                 const PointCloud::ConstPtr& reference_cloud,
                                 const std::vector<size_t>& correspondences,
                                 const Eigen::Matrix4f& T,
                                 Eigen::Matrix<double, 6, 6>* covariance);

  bool ComputeICPCovariancePointPoint(const PointCloud::ConstPtr& pointCloud,
                                      const Eigen::Matrix4f& T,
                                      const double& icp_fitness,
                                      Eigen::Matrix<double, 6, 6>& covariance);

  void AccumulateScans(const gtsam::Key& key, PointCloud::Ptr scan_out);

protected:
  // Define subscriber
  ros::Subscriber keyed_scans_sub_;
  ros::Subscriber keyed_poses_sub_;

  // Timer
  ros::Timer update_timer_;

  // Store keyed scans
  std::unordered_map<gtsam::Key, PointCloudConstPtr> keyed_scans_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> keyed_poses_;

  double max_tolerable_fitness_;
  double icp_tf_epsilon_;
  double icp_corr_dist_;
  unsigned int icp_iterations_;
  unsigned int icp_threads_;
  bool icp_transform_thresholding_;
  double icp_max_translation_;
  double icp_max_rotation_;

  // SAC feature alignment parameters
  unsigned int sac_iterations_;
  unsigned int sac_num_prev_scans_;
  unsigned int sac_num_next_scans_;
  double sac_normals_radius_;
  double sac_features_radius_;
  double sac_fitness_score_threshold_;

  // Teaser++ alignment parameters
  double teaser_inlier_threshold_;
  double rotation_cost_threshold_;
  double rotation_max_iterations_;
  double noise_bound_;
  double TEASER_FPFH_normals_radius_;
  double TEASER_FPFH_features_radius_;

  utils::HarrisParams harris_params_;

  double laser_lc_rot_sigma_;
  double laser_lc_trans_sigma_;
  bool b_use_fixed_covariances_;

  bool b_accumulate_source_;

  enum class IcpInitMethod {
    IDENTITY,
    ODOMETRY,
    ODOM_ROTATION,
    FEATURES,
    TEASERPP,
    CANDIDATE
  };

  enum class IcpCovarianceMethod { POINT2POINT, POINT2PLANE };

  IcpInitMethod icp_init_method_;

  IcpCovarianceMethod icp_covariance_method_;

  // ICP
  pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point> icp_;


  ThreadPool icp_computation_pool_;

  size_t number_of_threads_in_icp_computation_pool_;
};

} // namespace lamp_loop_closure