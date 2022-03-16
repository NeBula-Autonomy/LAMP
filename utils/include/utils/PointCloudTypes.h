/*
 * Copyright Notes
 *
 * Authors:
 * Yun Chang                    (yunchang@mit.edu)
 */

#ifndef POINT_CLOUD_TYPES_H
#define POINT_CLOUD_TYPES_H

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

// Typedef for stored point clouds.
typedef pcl::PointXYZINormal Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

typedef pcl::PointXYZI PointXyzi;
typedef pcl::PointCloud<PointXyzi> PointXyziCloud;
typedef pcl::PointCloud<PointXyzi>::ConstPtr PointXyziCloudConstPtr;

#endif