/*
PointCloudUtils.h
Author: Yun Chang
Some utility functions for wokring with Point Clouds
*/

#ifndef POINT_CLOUD_UTILS_H_
#define POINT_CLOUD_UTILS_H_

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <utils/CommonStructs.h>

namespace utils {

typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::FPFHSignature33> Features;

struct HarrisParams {
  double harris_threshold_;
  bool harris_suppression_;
  double harris_radius_;
  bool harris_refine_;
  int harris_response_;
};

void ComputeNormals(const PointCloud::ConstPtr& input,
                    const double& search_radius,
                    const int& num_threads,
                    Normals::Ptr normals);

void NormalizePCloud(const PointCloud::ConstPtr& cloud,
                     PointCloud::Ptr pclptr_normalized);
void ComputeKeypoints(const PointCloud::ConstPtr& source,
                      const HarrisParams& params,
                      const int& num_threads,
                      Normals::Ptr source_normals,
                      PointCloud::Ptr source_keypoints);
void ComputeFeatures(const PointCloud::ConstPtr& keypoints,
                     const PointCloud::ConstPtr& input,
                     const double& search_radius,
                     const int& num_threads,
                     Normals::Ptr normals,
                     Features::Ptr features);
} // namespace utils
#endif
