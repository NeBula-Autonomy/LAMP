/*
LaserLoopClosure.h
Author: Yun Chang
Lidar pointcloud based loop closure
*/

#ifndef LASER_LOOP_CLOSURE_H_
#define LASER_LOOP_CLOSURE_H_

#include "loop_closure/LoopClosureBase.h"
#include "utils/PointCloudUtils.h"

#include <unordered_map>

#include <multithreaded_gicp/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <gtsam/inference/Symbol.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <multithreaded_gicp/gicp.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_mapper/PointCloudMapper.h>

#include <map>
#include <utils/CommonStructs.h>

class LaserLoopClosure : public LoopClosure {
public:
  LaserLoopClosure(const ros::NodeHandle& n);
  ~LaserLoopClosure();

  bool Initialize(const ros::NodeHandle& n);

  typedef pcl::PointCloud<pcl::Normal> Normals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> Features;

private:
  void AccumulateScans(
      gtsam::Key key,
      PointCloud::Ptr scan_out);
  void GetInitialAlignment(
      PointCloud::ConstPtr source,
      PointCloud::ConstPtr target,
      Eigen::Matrix4f* tf_out,
      double& sac_fitness_score);

  void GetTEASERInitialAlignment(PointCloud::ConstPtr source,
                                 PointCloud::ConstPtr target,
                                 Eigen::Matrix4f* tf_out,
                                 double& n_inliers);

  bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);

  bool CheckForLoopClosure(
      gtsam::Symbol key1,
      gtsam::Symbol key2,
      bool b_inter_robot,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);
  bool PerformLoopClosure(
      gtsam::Symbol key1,
      gtsam::Symbol key2,
      bool b_use_prior,
      gtsam::Pose3 prior,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg);
  void SeedCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg);

  bool PerformAlignment(const gtsam::Symbol key1,
                        const gtsam::Symbol key2,
                        bool b_use_delta_as_prior,
                        geometry_utils::Transform3* delta,
                        gtsam::Matrix66* covariance,
                        double& fitness_score);

  double DistanceBetweenKeys(gtsam::Symbol key1, gtsam::Symbol key2);

  void GenerateGTFromPC(std::string gt_pc_filename);

  void TriggerGTCallback(const std_msgs::String::ConstPtr& msg);

  bool SetupICP();

  void PublishPointCloud(ros::Publisher&, PointCloud&);

  bool ComputeICPCovariancePointPoint(const PointCloud::ConstPtr& pointCloud,
                                      const Eigen::Matrix4f& T,
                                      const double& icp_fitness,
                                      Eigen::Matrix<double, 6, 6>& covariance);

  bool
  ComputeICPCovariancePointPlane(const PointCloud::ConstPtr& query_cloud,
                                 const PointCloud::ConstPtr& reference_cloud,
                                 const std::vector<size_t>& correspondences,
                                 const Eigen::Matrix4f& T,
                                 Eigen::Matrix<double, 6, 6>* covariance);

  void ComputeIcpObservability(PointCloud::ConstPtr cloud,
                               Eigen::Matrix<double, 3, 1>* eigenvalues);

  void ComputeAp_ForPoint2PlaneICP(const PointCloud::Ptr query_normalized,
                                   const Normals::Ptr reference_normals,
                                   const std::vector<size_t>& correspondences,
                                   const Eigen::Matrix4f& T,
                                   Eigen::Matrix<double, 6, 6>& Ap);

  void PublishLCComputationTime(const double& lc_computation_time,
                                const ros::Publisher& pub);

private:
  ros::Subscriber keyed_scans_sub_;
  ros::Subscriber loop_closure_seed_sub_;
  ros::Subscriber pc_gt_trigger_sub_;

  std::unordered_map<gtsam::Key, PointCloud::ConstPtr> keyed_scans_;

  ros::Publisher gt_pub_;
  ros::Publisher current_scan_pub_;
  ros::Publisher aligned_scan_pub_;
  ros::Publisher lc_computation_time_pub_;

  bool b_check_observability_;
  double min_observability_ratio_;
  double max_tolerable_fitness_;
  double translation_threshold_nodes_;
  double distance_before_reclosing_;
  double max_rotation_deg_;
  double max_rotation_rad_;
  size_t skip_recent_poses_;
  double proximity_threshold_;
  double icp_tf_epsilon_;
  double icp_corr_dist_;
  unsigned int icp_iterations_;
  unsigned int icp_threads_;

  unsigned int sac_iterations_;
  unsigned int sac_num_prev_scans_;
  unsigned int sac_num_next_scans_;
  double sac_normals_radius_;
  double sac_features_radius_;
  double sac_fitness_score_threshold_;

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

  double gt_rot_sigma_;
  double gt_trans_sigma_;
  double gt_prior_covar_;

  PointCloudFilter filter_;

  enum class IcpInitMethod {
    IDENTITY,
    ODOMETRY,
    ODOM_ROTATION,
    FEATURES,
    TEASERPP
  };

  enum class IcpCovarianceMethod { POINT2POINT, POINT2PLANE };

  IcpInitMethod icp_init_method_;

  IcpCovarianceMethod icp_covariance_method_;

  // ICP
  pcl::MultithreadedGeneralizedIterativeClosestPoint<Point, Point> icp_;

  // Test class fixtures
  friend class TestLaserLoopClosure;
};
#endif // LASER_LOOP_CLOSURE_H_
