/*
PointCloudUtils.cc
Author: Yun Chang
Some utility functions for wokring with Point Clouds
*/
#include "loop_closure/PointCloudUtils.h"

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_conversions/pcl_conversions.h>

namespace utils {

// void ComputeNormals(const PointCloud::ConstPtr& input,
//                    const double& search_radius,
//                    const int& num_threads,
//                    Normals::Ptr normals) {
//  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr search_method(
//      new pcl::search::KdTree<pcl::PointXYZINormal>);
//  pcl::NormalEstimationOMP<pcl::PointXYZINormal, pcl::Normal> norm_est;
//  norm_est.setInputCloud(input);
//  norm_est.setSearchMethod(search_method);
//  norm_est.setRadiusSearch(search_radius);
//  norm_est.setNumberOfThreads(num_threads);
//  norm_est.compute(*normals);
//}

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

} // namespace utils
