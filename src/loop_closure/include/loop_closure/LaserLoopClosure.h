/*
LaserLoopClosure.h
Author: Yun Chang
Lidar pointcloud based loop closure
*/

#ifndef LASER_LOOP_CLOSURE_H_
#define LASER_LOOP_CLOSURE_H_

#include "loop_closure/LoopClosureBase.h"

#include <unordered_map>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gtsam/inference/Symbol.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <multithreaded_gicp/gicp.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_mapper/PointCloudMapper.h>

#include <map>

class LaserLoopClosure : public LoopClosure {
public:
  LaserLoopClosure(const ros::NodeHandle& n);
  ~LaserLoopClosure();

  bool Initialize(const ros::NodeHandle& n);

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloudConstPtr;
  typedef pcl::PointCloud<pcl::Normal> Normals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> Features;

private:
  void AccumulateScans(
      gtsam::Key key,
      PointCloud::Ptr scan_out);
  void ComputeNormals(
      PointCloud::ConstPtr input,
      Normals::Ptr normals);
  void NormalizePCloud(PointCloud::ConstPtr cloud,
                       PointCloud::Ptr pclptr_normalized);
  void ComputeKeypoints(PointCloud::ConstPtr source,
                        Normals::Ptr source_normals,
                        PointCloud::Ptr source_keypoints);
  void ComputeFeatures(PointCloud::ConstPtr keypoints,
                       PointCloud::ConstPtr input,
                       Normals::Ptr normals,
                       Features::Ptr features);
  void GetInitialAlignment(
      PointCloud::ConstPtr source,
      PointCloud::ConstPtr target,
      Eigen::Matrix4f* tf_out,
      double& sac_fitness_score);
  
  bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);

  bool CheckForLoopClosure(
          gtsam::Symbol key1,
          gtsam::Symbol key2,
          std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);
  bool CheckForInterRobotLoopClosure(
          gtsam::Symbol key1,
          gtsam::Symbol key2,
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


  pose_graph_msgs::PoseGraphEdge CreateLoopClosureEdge(
          gtsam::Symbol key1, 
          gtsam::Symbol key2,
          geometry_utils::Transform3& delta, 
          gtsam::Matrix66& covariance);

  pose_graph_msgs::PoseGraphEdge CreatePriorEdge(
          gtsam::Symbol key,
          geometry_utils::Transform3& delta, 
          gtsam::Matrix66& covariance);

  void GenerateGTFromPC(std::string gt_pc_filename);

  void TriggerGTCallback(const std_msgs::String::ConstPtr& msg);

  bool SetupICP(
      pcl::MultithreadedGeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                         pcl::PointXYZI>& icp);

  void PublishPointCloud(ros::Publisher&, PointCloud&);

  bool ComputeICPCovariancePointPoint(const PointCloud::ConstPtr& pointCloud,
                                      const Eigen::Matrix4f& T,
                                      const double& icp_fitness,
                                      Eigen::Matrix<double, 6, 6>& covariance);

  bool ComputeICPCovariancePointPlane(const PointCloud::ConstPtr& pointCloud,
                                      const Eigen::Matrix4f& T,
                                      const double& icp_fitness,
                                      Eigen::Matrix<double, 6, 6>& covariance);

  void ComputeIcpObservability(PointCloud::ConstPtr cloud,
                               Eigen::Matrix<double, 6, 1>* eigenvalues);

  void ComputeAp_ForPoint2PlaneICP(const PointCloud::Ptr pcl_normalized,
                                   const Normals::Ptr pcl_normals,
                                   Eigen::Matrix<double, 6, 6>& Ap);

  void ComputeDiagonalAndUpperRightOfAi(Eigen::Vector3d& a_i,
                                        Eigen::Vector3d& n_i,
                                        Eigen::Matrix<double, 6, 6>& A_i);

 private:
  ros::Subscriber keyed_scans_sub_;
  ros::Subscriber loop_closure_seed_sub_;
  ros::Subscriber pc_gt_trigger_sub_;

  std::unordered_map<gtsam::Key, PointCloud::ConstPtr> keyed_scans_;

  // last_closure_key_<a,b> stores the last key for robot a on which there was a 
  // loop closure between robots a and b
  std::map< std::pair<char,char>, gtsam::Key> last_closure_key_;
  ros::Publisher gt_pub_;
  ros::Publisher current_scan_pub_;
  ros::Publisher aligned_scan_pub_;

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

  double harris_threshold_;
  bool harris_suppression_;
  double harris_radius_;
  bool harris_refine_;
  int harris_response_;

  double laser_lc_rot_sigma_;
  double laser_lc_trans_sigma_;
  bool b_use_fixed_covariances_;

  double gt_rot_sigma_;
  double gt_trans_sigma_;
  double gt_prior_covar_;

  PointCloudFilter filter_;

  enum class IcpInitMethod { IDENTITY, ODOMETRY, ODOM_ROTATION, FEATURES };

  enum class IcpCovarianceMethod { POINT2POINT, POINT2PLANE };

  IcpInitMethod icp_init_method_;

  IcpCovarianceMethod icp_covariance_method_;

  // Test class fixtures
  friend class TestLaserLoopClosure;
};
#endif // LASER_LOOP_CLOSURE_H_