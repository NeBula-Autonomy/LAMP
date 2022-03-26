/*
PointCloudUtils.h
Author: Yun Chang
Some utility functions for workng with Point Clouds
*/

#ifndef POINT_CLOUD_UTILS_H_
#define POINT_CLOUD_UTILS_H_

#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/point_cloud.h>
#include <lamp_utils/CommonStructs.h>
#include <lamp_utils/PointCloudTypes.h>

namespace lamp_utils {

typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::FPFHSignature33> Features;

struct HarrisParams {
  double harris_threshold_;
  bool harris_suppression_;
  double harris_radius_;
  bool harris_refine_;
  int harris_response_;
};

struct NormalComputeParams {
  int search_method = 0;
  // search_method = 0: use KNN, = 1: use radius
  int k = 10;
  double radius = 1.0;
  int num_threads = 4;
};

template <typename pointT>
void ComputeNormals(const typename pcl::PointCloud<pointT>::ConstPtr& input,
                    const NormalComputeParams& params,
                    Normals::Ptr normals) {
  typename pcl::search::KdTree<pointT>::Ptr search_method(
      new pcl::search::KdTree<pointT>);
  typename pcl::NormalEstimationOMP<pointT, pcl::Normal> norm_est;

  if (params.search_method == 0) {
    norm_est.setRadiusSearch(0);
    norm_est.setKSearch(params.k);
  } else if (params.search_method == 1) {
    norm_est.setKSearch(0);
    norm_est.setRadiusSearch(params.radius);
  } else {
    ROS_ERROR(
        "Wrong normal search method in lamp basestation normal computation. ");
    EXIT_FAILURE;
  }
  norm_est.setNumberOfThreads(params.num_threads);

  norm_est.setInputCloud(input);
  norm_est.setSearchMethod(search_method);
  norm_est.compute(*normals);
}

void ExtractNormals(
    const PointCloud::ConstPtr& input,
    Normals::Ptr normals,
    const NormalComputeParams& normal_params = NormalComputeParams());

void NormalizePCloud(const PointCloud::ConstPtr& cloud,
                     PointCloud::Ptr pclptr_normalized);

// Without precomputed normals
void ComputeKeypoints(const PointCloud::ConstPtr& source,
                      const HarrisParams& params,
                      const int& num_threads,
                      PointCloud::Ptr source_keypoints);

// With precomputed normals
void ComputeKeypoints(const PointCloud::ConstPtr& source,
                      const Normals::ConstPtr& source_normals,
                      const HarrisParams& params,
                      const int& num_threads,
                      PointCloud::Ptr source_keypoints);

void ComputeFeatures(const PointCloud::ConstPtr& keypoints,
                     const PointCloud::ConstPtr& input,
                     const Normals::Ptr& normals,
                     const double& search_radius,
                     const int& num_threads,
                     Features::Ptr features);

void ComputeIcpObservability(
    PointCloudConstPtr scan,
    Eigen::Matrix<double, 3, 1>* eigenvalues,
    const NormalComputeParams& params = NormalComputeParams());

bool ComputeICPCovariancePointPoint(const PointCloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix4f& T,
                                    const double& icp_fitness,
                                    Eigen::Matrix<double, 6, 6>& covariance);

bool ComputeICPCovariancePointPlane(const PointCloud::ConstPtr& query_cloud,
                                    const PointCloud::ConstPtr& reference_cloud,
                                    const std::vector<size_t>& correspondences,
                                    const Eigen::Matrix4f& T,
                                    Eigen::Matrix<double, 6, 6>* covariance);

void ComputeAp_ForPoint2PlaneICP(const PointCloud::Ptr query_normalized,
                                 const Normals::Ptr reference_normals,
                                 const std::vector<size_t>& correspondences,
                                 const Eigen::Matrix4f& T,
                                 Eigen::Matrix<double, 6, 6>& Ap);

void ConvertPointCloud(const PointCloud::ConstPtr& point_normal_cloud,
                       PointXyziCloud::Ptr point_cloud);

void AddNormals(const PointXyziCloud::ConstPtr& point_normal_cloud,
                const NormalComputeParams& params,
                PointCloud::Ptr point_cloud);

} // namespace lamp_utils
#endif
