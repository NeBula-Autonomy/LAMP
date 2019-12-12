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
#include <pose_graph_msgs/KeyedScan.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>

#include <geometry_utils/Transform3.h>
#include <point_cloud_filter/PointCloudFilter.h>

class LaserLoopClosure : public LoopClosure {
public:
  LaserLoopClosure(const ros::NodeHandle& n);
  ~LaserLoopClosure();

  bool Initialize(const ros::NodeHandle& n);

  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> Normals;
  typedef pcl::PointCloud<pcl::FPFHSignature33> Features;

private:
  void AccumulateScans(
      gtsam::Key key,
      PointCloud::Ptr scan_out);
  void ComputeNormals(
      PointCloud::ConstPtr input,
      Normals::Ptr normals);
  void ComputeFeatures(
      PointCloud::ConstPtr input,
      Normals::Ptr normals,
      Features::Ptr features);
  void GetInitialAlignment(
      PointCloud::ConstPtr source,
      PointCloud::ConstPtr target,
      Eigen::Matrix4f* tf_out);
  
  bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges);

  bool CheckForLoopClosure(
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

private:
  ros::Subscriber keyed_scans_sub_;
  ros::Subscriber loop_closure_seed_sub_;

  std::unordered_map<gtsam::Key, PointCloud::ConstPtr> keyed_scans_;

  gtsam::Key last_closure_key_;
  double max_tolerable_fitness_;
  double translation_threshold_nodes_;
  double distance_before_reclosing_;
  size_t skip_recent_poses_;
  double proximity_threshold_;
  double icp_tf_epsilon_;
  double icp_corr_dist_;
  unsigned int icp_iterations_;

  unsigned int sac_iterations_;
  unsigned int sac_num_prev_scans_;
  unsigned int sac_num_next_scans_;
  double sac_normals_radius_;
  double sac_features_radius_;

  double laser_lc_rot_sigma_;
  double laser_lc_trans_sigma_;

  PointCloudFilter filter_;

  enum class IcpInitMethod { IDENTITY, ODOMETRY, ODOM_ROTATION, FEATURES };

  IcpInitMethod icp_init_method_;
};
#endif // LASER_LOOP_CLOSURE_H_