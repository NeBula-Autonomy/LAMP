/*
PointCloudUtils.cc
Author: Yun Chang
Some utility functions for wokring with Point Clouds
*/
#include "loop_closure/PointCloudUtils.h"

#include <geometry_utils/Transform3.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>

namespace utils {

void ComputeNormals(const PointCloud::ConstPtr& input,
                    const double& search_radius,
                    const int& num_threads,
                    Normals::Ptr normals) {
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr search_method(
      new pcl::search::KdTree<pcl::PointXYZINormal>);
  pcl::NormalEstimationOMP<pcl::PointXYZINormal, pcl::Normal> norm_est;
  norm_est.setInputCloud(input);
  norm_est.setSearchMethod(search_method);
  norm_est.setRadiusSearch(search_radius);
  norm_est.setNumberOfThreads(num_threads);
  norm_est.compute(*normals);
}

void ComputeNormals(const PointCloud::ConstPtr& input,
                    const int& num_threads,
                    Normals::Ptr normals) {
  normals->resize(input->size());
  int enable_omp = (1 < num_threads);
#pragma omp parallel for schedule(dynamic, 1) if (enable_omp)
  for (size_t i = 0; i < input->size(); ++i) {
    normals->points[i].normal_x = input->points[i].normal_x;
    normals->points[i].normal_y = input->points[i].normal_y;
    normals->points[i].normal_z = input->points[i].normal_z;
  }
}

// returns a point cloud whose centroid is the origin, and that the mean of
// the distances to the origin is 1
void NormalizePCloud(const PointCloud::ConstPtr& cloud,
                     PointCloud::Ptr pclptr_normalized) {
  Eigen::Vector4f centroid_4d;
  pcl::compute3DCentroid(*cloud, centroid_4d);
  Eigen::Vector3f centroid(centroid_4d.x(), centroid_4d.y(), centroid_4d.z());

  float dist = 0;
  for (pcl::PointCloud<Point>::const_iterator it = cloud->points.begin();
       it != cloud->points.end();
       it++) {
    Eigen::Vector3f a_i(it->x, it->y, it->z);
    dist = dist + (a_i - centroid).norm();
  }
  float factor = cloud->points.size() / dist;
  Eigen::Matrix4f transform;
  transform = Eigen::Matrix4f::Identity();
  transform.block(0, 0, 3, 3) = factor * Eigen::Matrix3f::Identity();
  transform.block(0, 3, 4, 1) = -factor * centroid_4d;
  pcl::transformPointCloud(*cloud, *pclptr_normalized, transform);
  pcl::compute3DCentroid(*pclptr_normalized, centroid_4d);
}

void ComputeKeypoints(const PointCloud::ConstPtr& source,
                      const HarrisParams& params,
                      const int& num_threads,
                      Normals::Ptr source_normals,
                      PointCloud::Ptr source_keypoints) {
  pcl::HarrisKeypoint3D<Point, Point> harris_detector;

  harris_detector.setNonMaxSupression(params.harris_suppression_);
  harris_detector.setRefine(params.harris_refine_);
  harris_detector.setInputCloud(source);
  harris_detector.setNormals(source_normals);
  harris_detector.setNumberOfThreads(num_threads);
  harris_detector.setRadius(params.harris_radius_);
  harris_detector.setThreshold(params.harris_threshold_);
  harris_detector.setMethod(
      static_cast<pcl::HarrisKeypoint3D<Point, Point>::ResponseMethod>(
          params.harris_response_));
  harris_detector.compute(*source_keypoints);
}

void ComputeFeatures(const PointCloud::ConstPtr& keypoints,
                     const PointCloud::ConstPtr& input,
                     const double& search_radius,
                     const int& num_threads,
                     Normals::Ptr normals,
                     Features::Ptr features) {
  pcl::search::KdTree<Point>::Ptr search_method(new pcl::search::KdTree<Point>);
  pcl::FPFHEstimationOMP<Point, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud(keypoints);
  fpfh_est.setSearchSurface(input);
  fpfh_est.setInputNormals(normals);
  fpfh_est.setSearchMethod(search_method);
  fpfh_est.setRadiusSearch(search_radius);
  fpfh_est.setNumberOfThreads(num_threads);
  fpfh_est.compute(*features);
}

void ComputeAp_ForPoint2PlaneICP(const PointCloud::Ptr query_normalized,
                                 const Normals::Ptr reference_normals,
                                 const std::vector<size_t> &correspondences,
                                 const Eigen::Matrix4f &T,
                                 Eigen::Matrix<double, 6, 6> &Ap) {
  Ap = Eigen::Matrix<double, 6, 6>::Zero();
  double tol = 1e-10;
  Eigen::Vector3d a_i, n_i;
  for (uint32_t i = 0; i < query_normalized->size(); i++) {

    if (i >= correspondences.size()) {
      continue;
    }
    if (query_normalized != NULL) {
      a_i << query_normalized->points[i].x,  //////
          query_normalized->points[i].y,     //////
          query_normalized->points[i].z;
    } else{
      a_i << 0,0,0;
      ROS_ERROR("Query is null");
    }

    if ((reference_normals != NULL) && (reference_normals->points.size() > correspondences[i])) {
      n_i << reference_normals->points[correspondences[i]].normal_x,  //////
          reference_normals->points[correspondences[i]].normal_y,     //////
          reference_normals->points[correspondences[i]].normal_z;
    } else {
      n_i << 0,0,0;
      ROS_ERROR("Issue with reference_normals, setting covariance 0");
    }

    if (a_i.hasNaN() || n_i.hasNaN())
      continue;


    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H.block(0, 0, 1, 3) = (a_i.cross(n_i)).transpose();
    H.block(0, 3, 1, 3) = n_i.transpose();
    Ap += H.transpose() * H;
  }
}

void ComputeIcpObservability(PointCloud::ConstPtr cloud,
                             const double& normals_radius,
                             const size_t& num_threads,
                             Eigen::Matrix<double, 3, 1>* eigenvalues) {
  // Get normals
  Normals::Ptr normals(new Normals);           // pc with normals
  PointCloud::Ptr normalized(new PointCloud);  // pc whose points have been
                                               // rearranged.
  utils::ComputeNormals(cloud, normals_radius, num_threads, normals);
  utils::NormalizePCloud(cloud, normalized);

  for (size_t i = 0; i < cloud->size(); i++) {
    Point p = cloud->points[i];
    pcl::Normal n = normals->points[i];
  }

  // Correspondence with itself (not really used anyways)
  std::vector<size_t> c(cloud->size());
  std::iota(std::begin(c), std::end(c), 0); // Fill with 0, 1, ...

  Eigen::Matrix4f T_unsued = Eigen::Matrix4f::Zero(); // Unused

  Eigen::Matrix<double, 6, 6> Ap;
  // Compute Ap and its eigenvalues
  utils::ComputeAp_ForPoint2PlaneICP(normalized, normals, c, T_unsued, Ap);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eigensolver(
      Ap.block(3, 3, 3, 3));
  if (eigensolver.info() == Eigen::Success) {
    *eigenvalues = eigensolver.eigenvalues();
  } else {
    ROS_WARN("Failed to decompose observability matrix. ");
  }
}

}  // namespace utils