/*
test_artifacts.h
Author: Yun Chang
Test artifacts for point cloud utils unittests
*/

#pragma once

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <lamp_utils/CommonStructs.h>

PointCloud::Ptr GeneratePlane(size_t x_points = 10,
                              size_t y_points = 10,
                              float step_x = 0.1f,
                              float step_y = 0.1f,
                              std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloud>();
  pc_out->reserve(x_points * y_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      pc_out->push_back(
          Point({ix * step_x, iy * step_y, 0.0, 0.0, 0.0, 0.0, 1.0}));
    }
  }

  return pc_out;
}

PointCloud::Ptr GenerateCorner() {
  PointCloud::Ptr plane(new PointCloud);
  plane = GeneratePlane();

  // Perform transformation
  PointCloud::Ptr transformed_plane1(new PointCloud);
  Eigen::Matrix4f T_plane = Eigen::Matrix4f::Zero();
  T_plane(0, 0) = 1;
  T_plane(1, 2) = -1;
  T_plane(2, 1) = 1;
  T_plane(3, 3) = 1;
  pcl::transformPointCloudWithNormals(
      *plane, *transformed_plane1, T_plane, true);

  // Add another
  PointCloud::Ptr transformed_plane2(new PointCloud);
  T_plane = Eigen::Matrix4f::Zero();
  T_plane(0, 2) = -1;
  T_plane(1, 1) = 1;
  T_plane(2, 0) = 1;
  T_plane(3, 3) = 1;
  pcl::transformPointCloudWithNormals(
      *plane, *transformed_plane2, T_plane, true);
  *plane += *transformed_plane1;
  *plane += *transformed_plane2;
  return plane;
}

PointCloud::Ptr GenerateBox() {
  PointCloud::Ptr corner(new PointCloud);
  corner = GenerateCorner();

  // Perform transformation
  PointCloud::Ptr transformed_corner(new PointCloud);
  Eigen::Matrix4f T = Eigen::Matrix4f::Zero();
  T(0, 0) = -1;
  T(1, 1) = 1;
  T(2, 2) = -1;
  T(3, 3) = 1;
  pcl::transformPointCloudWithNormals(*corner, *transformed_corner, T, true);

  T = Eigen::Matrix4f::Zero();
  T(0, 1) = -1;
  T(1, 0) = 1;
  T(2, 2) = 1;
  T(3, 3) = 1;
  T(0, 3) = 1;
  T(1, 3) = 1;
  T(2, 3) = 1;
  pcl::transformPointCloudWithNormals(
      *transformed_corner, *transformed_corner, T, true);

  *corner += *transformed_corner;
  return corner;
}

pose_graph_msgs::KeyedScan
PointCloudToKeyedScan(const PointCloud::ConstPtr& cloud,
                      const gtsam::Key& key) {
  pose_graph_msgs::KeyedScan keyed_scan;
  pcl::toROSMsg(*cloud, keyed_scan.scan);
  keyed_scan.key = key;
  return keyed_scan;
}