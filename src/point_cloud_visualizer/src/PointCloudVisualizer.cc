/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 *
 */

#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_visualizer/PointCloudVisualizer.h>
#include <utils/PrefixHandling.h>

namespace pu = parameter_utils;

PointCloudVisualizer::PointCloudVisualizer() {
  // Instantiate point cloud pointers.
  incremental_points_.reset(new PointCloud);
}

PointCloudVisualizer::~PointCloudVisualizer() {}

bool PointCloudVisualizer::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudVisualizer");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  for (std::string robot : robot_names_) {
    PointCloud::Ptr cloud_to_fill(new PointCloud);
    robots_point_clouds_.insert(std::pair<unsigned char, PointCloud::Ptr>(
        utils::ROBOT_PREFIXES.at(robot), cloud_to_fill));
  }

  return true;
}

bool PointCloudVisualizer::LoadParameters(const ros::NodeHandle& n) {
  // Load visualization parameters.
  //  if (!pu::Get("visualizer/enable_visualization", enable_visualization_))
  //    return false;

  // Load coordinate frames.
  //  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
  //    return false;

  if (!pu::Get("/base1/lamp/robot_names", robot_names_)) {
    ROS_ERROR("%s: No robot names provided to base station.", name_.c_str());
    return false;
  } else {
    for (auto s : robot_names_) {
      ROS_INFO_STREAM("Registered new robot: " << s);
    }
  }

  return true;
}

bool PointCloudVisualizer::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Initialize publishers.
  incremental_points_pub_ =
      nl.advertise<sensor_msgs::PointCloud2>("incremental_points", 10, false);

  keyed_scan_sub_ = nl.subscribe<pose_graph_msgs::KeyedScan>(
      "/base1/lamp/keyed_scans",
      10,
      &PointCloudVisualizer::KeyedScanCallback,
      this);
  pose_graph_sub_ = nl.subscribe<pose_graph_msgs::PoseGraph>(
      "/base1/lamp/pose_graph",
      10,
      &PointCloudVisualizer::PoseGraphCallback,
      this);

  back_end_pose_graph_sub_ =
      nl.subscribe("/base1/lamp_pgo/optimized_values",
                   1,
                   &PointCloudVisualizer::OptimizerUpdateCallback,
                   this);

  for (std::string robot : robot_names_) {
    publishers_robots_point_clouds_.insert(
        std::pair<unsigned char, ros::Publisher>(
            utils::ROBOT_PREFIXES.at(robot),
            nl.advertise<sensor_msgs::PointCloud2>(
                robot + "/lamp/octree_map", 10, false)));
  }

  return true;
}

bool PointCloudVisualizer::InsertPointCloud(const PointCloud& points) {
  // Make sure the user wants visualization enabled.
  if (!enable_visualization_)
    return false;

  // Store the timestamp for the next time the point cloud is published.
  stamp_.fromNSec(points.header.stamp * 1e3);

  // Merge with the existing point cloud .
  *incremental_points_ += points;

  return true;
}

void PointCloudVisualizer::PublishIncrementalPointCloud() {
  // Check before doing any work.
  if (!enable_visualization_ ||
      incremental_points_pub_.getNumSubscribers() == 0)
    return;

  // Convert incremental points to ROS's sensor_msgs::PointCloud2 type.
  sensor_msgs::PointCloud2 ros_incremental_points;
  pcl::toROSMsg(*incremental_points_, ros_incremental_points);
  ros_incremental_points.header.stamp = stamp_;
  ros_incremental_points.header.frame_id = fixed_frame_id_;
  incremental_points_pub_.publish(ros_incremental_points);

  // Remove all incremental points to reset accumulation.
  incremental_points_.reset(new PointCloud);
}

void PointCloudVisualizer::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& msg) {
  const gtsam::Key& key = msg->key;
  if (pose_graph_.HasScan(key)) {
    ROS_ERROR("%s: Key %lu already has a laser scan.", name_.c_str(), key);
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg->scan, *scan);

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (key == 0) {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    pose_graph_.InsertKeyedStamp(key, stamp);
  }

  // Add the key and scan.
  pose_graph_.InsertKeyedScan(key, scan);
}

void PointCloudVisualizer::OptimizerUpdateCallback(
    const pose_graph_msgs::PoseGraphConstPtr& msg) {
  ROS_INFO_STREAM("POSE GRAPH SIZE: " << pose_graph_.keyed_scans.size());
  ROS_INFO_STREAM("POSE GRAPH SIZE2: " << pose_graph_.nodes_.size());
  pose_graph_.Reset();
  pose_graph_.UpdateFromMsg(msg);
  ROS_INFO_STREAM("Optimized: " << msg.get()->nodes.size());
  ROS_INFO_STREAM("POSE GRAPH SIZE: " << pose_graph_.keyed_scans.size());
  ROS_INFO_STREAM("POSE GRAPH SIZE2: " << pose_graph_.nodes_.size());
}

void PointCloudVisualizer::PoseGraphCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  ROS_INFO_STREAM("CAsual: " << msg.get()->nodes.size());
  ROS_INFO_STREAM("CAsual incremental: " << msg->incremental);
  ROS_INFO_STREAM(msg->incremental);
  pose_graph_.UpdateFromMsg(msg);
  VisualizePointCloud();
  if (!msg->incremental) {
    pose_graph_.Reset();
  }
}
geometry_msgs::Point
PointCloudVisualizer::GetPositionMsg(gtsam::Key key) const {
  geometry_msgs::Point p;
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN_STREAM(
        " Key "
        << gtsam::DefaultKeyFormatter(key)
        << " does not exist in GetPositionMsg - returning zeros [TO IMPROVE]");
    return p;
  }
  auto pose = pose_graph_.GetPose(key);
  p.x = pose.x();
  p.y = pose.y();
  p.z = pose.z();
  return p;
}

void PointCloudVisualizer::VisualizePointCloud() {
  for (const auto& keyed_scan : pose_graph_.keyed_scans) {
    PointCloud::Ptr temp_cloud(new PointCloud);
    GetTransformedPointCloudWorld(keyed_scan.first, temp_cloud.get());
    *robots_point_clouds_.at(keyed_scan.first.chr()) += *temp_cloud;

    //    ROS_INFO_STREAM("Cloud : "
    //                    << keyed_scan.first.chr() << " has "
    //                    <<
    //                    robots_point_clouds_.at(keyed_scan.first.chr())->size()
    //                    << " after adding : " << temp_cloud->size());
  }

  if (pose_graph_.keyed_scans.size() != 0) {
    for (std::string robot : robot_names_) {
      unsigned char robot_chr = utils::ROBOT_PREFIXES.at(robot);

      if (publishers_robots_point_clouds_.at(robot_chr).getNumSubscribers() >
          0) {
        sensor_msgs::PointCloud2 pcl_pc2;
        pcl::toROSMsg(*robots_point_clouds_.at(robot_chr), pcl_pc2);
        pcl_pc2.header.stamp = stamp_;
        pcl_pc2.header.frame_id = fixed_frame_id_;
        publishers_robots_point_clouds_.at(robot_chr).publish(pcl_pc2);
      }
    }
  }
}

bool PointCloudVisualizer::GetTransformedPointCloudWorld(
    const gtsam::Symbol key, PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return false;
  }
  points->points.clear();

  // No key associated with the scan
  if (!pose_graph_.HasScan(key)) {
    ROS_WARN("Could not find scan associated with key in "
             "GetTransformedPointCloudWorld");
    return false;
  }

  // Check that the key exists
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %s does not exist in values in GetTransformedPointCloudWorld",
             gtsam::DefaultKeyFormatter(key).c_str());
    return false;
  }

  const gu::Transform3 pose = utils::ToGu(pose_graph_.GetPose(key));
  Eigen::Matrix4d b2w;
  b2w.setZero();
  b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
  b2w.block(0, 3, 3, 1) = pose.translation.Eigen();
  b2w(3, 3) = 1;

  Eigen::Quaterniond quat(pose.rotation.Eigen());
  quat.normalize();
  b2w.block(0, 0, 3, 3) = quat.matrix();

  pcl::transformPointCloud(*pose_graph_.keyed_scans[key], *points, b2w);

  return true;
}
