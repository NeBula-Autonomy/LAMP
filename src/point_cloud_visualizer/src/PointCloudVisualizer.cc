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

#include <numeric>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_visualizer/PointCloudVisualizer.h>
#include <tf/transform_broadcaster.h>
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
    robots_point_clouds_.insert(std::pair<unsigned char, PointCloud::Ptr>(
        utils::ROBOT_PREFIXES.at(robot), new PointCloud));
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
  fixed_frame_id_ = "world";
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

  nh_ = ros::NodeHandle(n);

  // Initialize publishers.
  incremental_points_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("incremental_points", 10, false);

  cone_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("current_cone", 1, true);

  crossed_nodes_pub_ =
      nh_.advertise<visualization_msgs::Marker>("crossed_node", 1, true);

  keyed_scan_sub_ = nh_.subscribe<pose_graph_msgs::KeyedScan>(
      "/base1/lamp/keyed_scans",
      10,
      &PointCloudVisualizer::KeyedScanCallback,
      this);
  pose_graph_sub_ = nh_.subscribe<pose_graph_msgs::PoseGraph>(
      "/base1/lamp/pose_graph",
      10,
      &PointCloudVisualizer::PoseGraphCallback,
      this);

  back_end_pose_graph_sub_ =
      nh_.subscribe("/base1/lamp_pgo/optimized_values",
                    1,
                    &PointCloudVisualizer::OptimizerUpdateCallback,
                    this);

  for (std::string robot : robot_names_) {
    publishers_robots_point_clouds_.insert(
        std::pair<unsigned char, ros::Publisher>(
            utils::ROBOT_PREFIXES.at(robot),
            nh_.advertise<sensor_msgs::PointCloud2>(
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
    //    ROS_ERROR("%s: Key %lu already has a laser scan.", name_.c_str(),
    //    key);
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
  key_scans_to_update_.insert(
      std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));
}

double point2planedistnace(const pcl::PointXYZ pt,
                           const pcl::ModelCoefficients& coefficients) {
  double f1 =
      fabs(coefficients.values[0] * pt.x + coefficients.values[1] * pt.y +
           coefficients.values[2] * pt.z + coefficients.values[3]);
  double f2 =
      sqrt(pow(coefficients.values[0], 2) + pow(coefficients.values[1], 2) +
           pow(coefficients.values[2], 2));
  return f1 / f2;
}

double point2planedistnace(const gu::Transform3 pose,
                           const pcl::ModelCoefficients& coefficients) {
  pcl::PointXYZ pt;
  pt.x = pose.translation.X();
  pt.y = pose.translation.Y();
  pt.z = pose.translation.Z();
  return point2planedistnace(pt, coefficients);
}

double point2planedistnaceWithSign(const pcl::PointXYZ pt,
                                   const pcl::ModelCoefficients& coefficients) {
  double f1 = coefficients.values[0] * pt.x + coefficients.values[1] * pt.y +
      coefficients.values[2] * pt.z + coefficients.values[3];
  double f2 =
      sqrt(pow(coefficients.values[0], 2) + pow(coefficients.values[1], 2) +
           pow(coefficients.values[2], 2));
  return f1 / f2;
}

double point2planedistnaceWithSign(const gu::Transform3 pose,
                                   const pcl::ModelCoefficients& coefficients) {
  pcl::PointXYZ pt;
  pt.x = pose.translation.X();
  pt.y = pose.translation.Y();
  pt.z = pose.translation.Z();
  return point2planedistnaceWithSign(pt, coefficients);
}

void PointCloudVisualizer::PoseGraphCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  if (msg->nodes.size() != pose_graph_.GetValues().size()) {
    pose_graph_.UpdateFromMsg(msg);
    for (const auto& keyed_scan : key_scans_to_update_) {
      PointCloud::Ptr temp_cloud(new PointCloud);
      GetTransformedPointCloudWorld(keyed_scan.first, temp_cloud.get());

      *robots_point_clouds_.at(keyed_scan.first.chr()) += *temp_cloud;
      //      AddPointCloudToCorrespondingLevel(keyed_scan.first, temp_cloud);
      AddPointCloudToCorrespondingLevel2(keyed_scan.first, temp_cloud);
    }
    VisualizePointCloud();
    key_scans_to_update_.clear();
  }
}
geometry_msgs::Point
PointCloudVisualizer::GetPositionMsg(gtsam::Key key) const {
  geometry_msgs::Point p;
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN_STREAM(" Key " << gtsam::DefaultKeyFormatter(key)
                            << " does not exist in GetPositionMsg - "
                               "returning zeros [TO IMPROVE]");
    return p;
  }
  auto pose = pose_graph_.GetPose(key);
  p.x = pose.x();
  p.y = pose.y();
  p.z = pose.z();
  return p;
}

void PointCloudVisualizer::VisualizePointCloud() {
  for (std::string robot : robot_names_) {
    unsigned char robot_chr = utils::ROBOT_PREFIXES.at(robot);
    if (publishers_robots_point_clouds_.at(robot_chr).getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 pcl_pc2;
      pcl::toROSMsg(*robots_point_clouds_.at(robot_chr), pcl_pc2);
      pcl_pc2.header.stamp = stamp_;
      pcl_pc2.header.frame_id = fixed_frame_id_;
      publishers_robots_point_clouds_.at(robot_chr).publish(pcl_pc2);
    }
  }
  for (auto& level : levels_) {
    if (level.pub_.getNumSubscribers() > 0) {
      level.points_->header.stamp = stamp_.nsec;
      level.points_->header.frame_id = level.points_->header.frame_id;
      level.tf_.stamp_ = stamp_;
      broadcaster_.sendTransform(level.tf_);
      level.pub_.publish(*level.points_);
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
    //    ROS_WARN("Could not find scan associated with key in "
    //             "GetTransformedPointCloudWorld");
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
int id = 0;

double base_height = 9999.0;      // 50.0;
double base_radius = base_height; //
double height_min_ = 2.0;
double offset = 25.0;
visualization_msgs::Marker CreateConeMarker(const geometry_msgs::Pose& pose,
                                            /*double angle, */ double height,
                                            double radius) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "Triangle";
  // Set the marker action. Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;
  // Set the marker type
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  // Lifetime
  marker.lifetime = ros::Duration(0.0);

  marker.id = id++;

  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.pose.position = pose.position;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = -0.7071068;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0.7071068;

  geometry_msgs::Point p[3];
  static const double DELTA_THETA = M_PI / 16.0;
  double theta = 0;

  marker.points.clear();
  for (std::size_t i = 0; i < 32; i++) {
    p[0].x = 0;
    p[0].y = 0;
    p[0].z = 0;

    p[1].x = height;
    p[1].y = radius * cos(theta); // / angle;
    p[1].z = radius * sin(theta); // / angle;

    p[2].x = height;
    p[2].y = radius * cos(theta + DELTA_THETA); // / angle;
    p[2].z = radius * sin(theta + DELTA_THETA); // / angle;

    marker.points.push_back(p[0]);
    marker.points.push_back(p[1]);
    marker.points.push_back(p[2]);

    theta += DELTA_THETA;
  }

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  return marker;
}
int id2 = 0;

visualization_msgs::Marker CreateNodeMarker(const gu::Transform3& pose,
                                            double time = 0.0) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "spheres";
  // Set the marker action. Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;
  // Set the marker type
  marker.type = visualization_msgs::Marker::SPHERE;

  // Lifetime
  marker.lifetime = ros::Duration(time);

  marker.id = id2++;

  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = pose.translation.X();
  marker.pose.position.y = pose.translation.Y();
  marker.pose.position.z = pose.translation.Z();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  return marker;
}

bool PointCloudVisualizer::IsPointInsideTheCone(
    const gu::Transform3& current_pose,
    const gu::Transform3& point_to_test,
    double offset /*= 0.0*/) const {
  gu::Vector3Base<double> dir{0.0, 0.0, 1.0};
  double radius = base_radius;     // 15.0 * sqrt(3);
  double height_max = base_height; // 15.0;
  double height_min = height_min_;

  auto cone_dist =
      (point_to_test.translation - current_pose.translation).Dot(dir);
  // reject points that are above the cone and below "cut" part
  if (cone_dist < height_min or cone_dist > height_max)
    return false;

  auto cone_radius = (cone_dist / height_max) * radius;
  auto orth_distance =
      ((point_to_test.translation - current_pose.translation) - cone_dist * dir)
          .Norm();
  return (orth_distance <= cone_radius + offset);
}

bool PointCloudVisualizer::IsPointInsideTheNegativeCone(
    const gu::Transform3& current_pose,
    const gu::Transform3& point_to_test,
    double offset /*= 0.0*/) const {
  gu::Vector3Base<double> dir{0.0, 0.0, -1.0};
  double radius = base_radius;     // 15.0 * sqrt(3);
  double height_max = base_height; // 15.0;
  double height_min = height_min_;

  auto cone_dist =
      (point_to_test.translation - current_pose.translation).Dot(dir);

  // reject points that are above the cone and below "cut" part
  if (cone_dist < height_min or cone_dist > height_max) {
    //    ROS_INFO_STREAM("cone dist: " << cone_dist);
    return false;
  }

  auto cone_radius = (cone_dist / height_max) * radius;
  auto orth_distance =
      ((point_to_test.translation - current_pose.translation) - cone_dist * dir)
          .Norm();
  //  ROS_INFO_STREAM("Orthe distance: " << orth_distance);
  return (orth_distance <= cone_radius + offset);
}

double calculateDistance(const gu::Transform3& current_pose,
                         const gu::Transform3& node) {
  double dx2 = std::pow(current_pose.translation.X() - node.translation.X(), 2);
  double dy2 = std::pow(current_pose.translation.Y() - node.translation.Y(), 2);
  double dz2 = std::pow(current_pose.translation.Z() - node.translation.Z(), 2);

  return dx2 + dy2 + 100.0 * dz2; // 1000.0 *
}

pcl::ModelCoefficients SegmentPlane(const std::vector<gu::Transform3> nodes_) {
  // Get segmentation ready
  pcl::PointCloud<pcl::PointXYZ>::Ptr graph_as_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  graph_as_cloud->reserve(nodes_.size());
  std::vector<gtsam::Symbol> keyes;

  for (const auto& node : nodes_) {
    pcl::PointXYZ pc;
    pc.x = node.translation.X();
    pc.y = node.translation.Y();
    pc.z = node.translation.Z();
    graph_as_cloud->push_back(pc);
  }

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1.5);
  seg.setMaxIterations(100);
  seg.setEpsAngle(0.1);
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

  seg.setInputCloud(graph_as_cloud);
  seg.segment(*inliers, *coefficients);

  return *coefficients;
}

bool IsPoseBetweenConsecutiveLevels(const pcl::ModelCoefficients& level1,
                                    const pcl::ModelCoefficients& level2,
                                    const gu::Transform3& current_pose) {
  if (point2planedistnaceWithSign(current_pose, level1) < 0.0 and
      point2planedistnaceWithSign(current_pose, level2) > 0.0)
    return true;
  if (point2planedistnaceWithSign(current_pose, level1) > 0.0 and
      point2planedistnaceWithSign(current_pose, level2) < 0.0)
    return true;
  return false;
}

size_t
PointCloudVisualizer::SelectLevelForNode(const gtsam::Symbol current_key) {
  const gu::Transform3 current_pose =
      utils::ToGu(pose_graph_.GetPose(current_key));

  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(marker);
  cone_pub_.publish(markers);
  markers.markers.clear();
  auto cone_marker =
      CreateConeMarker(utils::GtsamToRosMsg(pose_graph_.GetPose(current_key)),
                       base_height,
                       base_radius);

  auto cone_marker_neg =
      CreateConeMarker(utils::GtsamToRosMsg(pose_graph_.GetPose(current_key)),
                       -base_height,
                       base_radius);
  markers.markers.push_back(cone_marker_neg);
  markers.markers.push_back(cone_marker);

  cone_pub_.publish(markers);

  std::vector<std::pair<double, size_t>> potential_levels;
  ROS_INFO_STREAM(
      "<-------------------------------------------------------------->");
  ROS_INFO_STREAM("Levels to check: " << levels_.size());
  for (size_t i = 0; i < levels_.size(); ++i) {
    size_t index = 0;
    ROS_INFO_STREAM("Level "
                    << i << " has " << levels_[i].nodes_.size()
                    << " init nodes: " << levels_[i].init_nodes_.size());
    bool node_between_planes = false;
    bool crossed_cone = false;

    if (levels_.size() > 1 and levels_.size() > i + 1 and
        levels_[i].IsInitialized() and levels_[i + 1].IsInitialized()) {
      if (IsPoseBetweenConsecutiveLevels(levels_[i].coefficients_,
                                         levels_[i + 1].coefficients_,
                                         current_pose)) {
        ROS_INFO_STREAM(
            "IsPoseBetweenConsecutiveLevels: Yes, THE NODE BETWEEN LEVELS!");
        ROS_INFO_STREAM(
            "Current level: "
            << i << " coeff " << levels_[i].coefficients_.values[0] << " "
            << levels_[i].coefficients_.values[1] << " "
            << levels_[i].coefficients_.values[2] << " "
            << levels_[i].coefficients_.values[3] << " distance: "
            << point2planedistnace(current_pose, levels_[i].coefficients_));
        ROS_INFO_STREAM("Next level: "
                        << i + 1 << " coeff "
                        << levels_[i + 1].coefficients_.values[0] << " "
                        << levels_[i + 1].coefficients_.values[1] << " "
                        << levels_[i + 1].coefficients_.values[2] << " "
                        << levels_[i + 1].coefficients_.values[3]
                        << " distance: "
                        << point2planedistnace(current_pose,
                                               levels_[i + 1].coefficients_));
        ROS_INFO_STREAM("NO SENSE TO CHECK FURTHER!");
        node_between_planes = true;
      }
    }
    if (node_between_planes) {
      if (point2planedistnace(current_pose, levels_[i].coefficients_) <
          point2planedistnace(current_pose, levels_[i + 1].coefficients_))
        potential_levels.emplace_back(std::pair<double, size_t>(
            point2planedistnace(current_pose, levels_[i].coefficients_), i));
      else
        potential_levels.emplace_back(std::pair<double, size_t>(
            point2planedistnace(current_pose, levels_[i + 1].coefficients_),
            i + 1));
      continue;
    }

    double min_distance = std::numeric_limits<double>::infinity();

    if (levels_[i].IsInitialized()) {
      ROS_INFO_STREAM("Checking level: " << i);

      for (size_t j = 0; j < levels_[i].nodes_.size(); ++j) {
        if (IsPointInsideTheCone(current_pose, levels_[i].nodes_[j], offset) or
            IsPointInsideTheNegativeCone(
                current_pose, levels_[i].nodes_[j], offset)) {
          crossed_cone = true;
          auto marker = CreateNodeMarker(levels_[i].nodes_[j]);
          crossed_nodes_pub_.publish(marker);

          if (crossed_cone) {
            ROS_INFO_STREAM("BREAKING INSIDE THE CONE NODE!");
            break;
          }
        }

        auto distance = calculateDistance(current_pose, levels_[i].nodes_[j]);
        if (distance < min_distance) {
          min_distance = distance;
          index = j;
        }
        if (j == levels_[i].nodes_.size() - 1) {
          ROS_INFO_STREAM("Adding potential level: " << i << " min distance: "
                                                     << min_distance);
          potential_levels.emplace_back(
              std::pair<double, size_t>(min_distance, i));
        }
      }
    } else {
      for (size_t k = 0; k < levels_[i].init_nodes_.size(); ++k) {
        auto distance =
            calculateDistance(current_pose, levels_[i].init_nodes_[k]);

        if (distance < min_distance) {
          min_distance = distance;
          index = k;
        }
      }
      potential_levels.emplace_back(std::pair<double, size_t>(min_distance, i));
    }
  }

  if (potential_levels.size() == 0) {
    selected_level = levels_.size();
    CreateNewLevel();
  } else {
    auto min_pair = *std::min_element(
        potential_levels.cbegin(),
        potential_levels.cend(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    ROS_INFO_STREAM("Potential levels size: " << potential_levels.size()
                                              << " the best level association: "
                                              << min_pair.first << " "
                                              << min_pair.second);
    selected_level = min_pair.second;
  }
  if (levels_[selected_level].init_nodes_.size() < 20) {
    levels_[selected_level].init_nodes_.push_back(current_pose);
    if (levels_[selected_level].init_nodes_.size() >= 15) {
      levels_[selected_level].nodes_.push_back(current_pose);
    }
  } else {
    levels_[selected_level].nodes_.push_back(current_pose);
  }

  if (levels_[selected_level].nodes_.size() != 0) {
    if (levels_[selected_level].nodes_.size() % 5 == 0) {
      ROS_INFO_STREAM("UPDATING " << selected_level << " PLANE");
      levels_[selected_level].coefficients_ =
          SegmentPlane(levels_[selected_level].nodes_);
    }
    for (size_t i = 0; i < levels_.size(); i++) {
      ROS_INFO_STREAM("Current "
                      "plane coefficients: "
                      << i << " : " << levels_[i].coefficients_.values[0] << " "
                      << levels_[i].coefficients_.values[1] << " "
                      << levels_[i].coefficients_.values[2] << " "
                      << levels_[i].coefficients_.values[3]);
    }
  }

  return selected_level;
}

void PointCloudVisualizer::AddPointCloudToCorrespondingLevel2(
    const gtsam::Symbol key, const PointCloud::Ptr& pc) {
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %s does not exist in values in GetTransformedPointCloudWorld",
             gtsam::DefaultKeyFormatter(key).c_str());
    return;
  }

  size_t selected_level = SelectLevelForNode(key);

  if (levels_[selected_level].init_nodes_.size() > 15) {
    *levels_[selected_level].points_ += *pc;
  }
}

void PointCloudVisualizer::AddPointCloudToCorrespondingLevel(
    const gtsam::Symbol key, const PointCloud::Ptr& pc) {
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %s does not exist in values in GetTransformedPointCloudWorld",
             gtsam::DefaultKeyFormatter(key).c_str());
    return;
  }

  const gu::Transform3 current_pose = utils::ToGu(pose_graph_.GetPose(key));
  size_t theClosestLevelIndex = 0;

  if (levels_.size() != 0) {
    std::vector<double> min_levels;

    int i = 0;
    for (auto& level : levels_) {
      ROS_INFO_STREAM("GOING THROUGH LEVEL " << i << "/" << levels_.size());
      ROS_INFO_STREAM(level.coefficients_.values[0]
                      << " " << level.coefficients_.values[1] << " "
                      << level.coefficients_.values[2] << " "
                      << level.coefficients_.values[3]);
      i++;
      auto lol = point2planedistnace(current_pose, level.coefficients_);

      min_levels.push_back(lol);
      ROS_INFO_STREAM("distance: " << lol
                                   << " min levels: " << min_levels.back()
                                   << " " << min_levels.size());
    }

    auto it = std::min_element(std::begin(min_levels), std::end(min_levels));
    std::cout << "index of smallest element: "
              << std::distance(std::begin(min_levels), it);

    theClosestLevelIndex = std::distance(std::begin(min_levels), it);
    ROS_INFO_STREAM("XD: " << theClosestLevelIndex);
  }

  //  size_t selected_level = SelectLevelForNode(key);
  ROS_INFO_STREAM("THE CLOSEST INDEX " << theClosestLevelIndex);
  *levels_[theClosestLevelIndex].points_ += *pc;
  ROS_INFO_STREAM("Points: " << levels_[theClosestLevelIndex].points_->size());
}

void PointCloudVisualizer::CreateNewLevel() {
  std::string name = "level_" + std::to_string(levels_.size());
  double x = static_cast<double>(levels_.size() + 1) * 0.0;
  ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  ROS_INFO_STREAM("CreateNewLevel  " << name);
  ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  tf::StampedTransform transform(
      tf::Transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                    tf::Vector3(0.0, x, 0.0)),
      ros::Time(0),
      fixed_frame_id_,
      name);
  PointCloud::Ptr pc_level{new PointCloud};
  pc_level->reserve(
      1000000); // to have capacity enough to not allocate memory all the time
  pc_level->header.frame_id = name;

  pcl::ModelCoefficients coeff;

  coeff.values.push_back(0.0f);
  coeff.values.push_back(0.0f);
  coeff.values.push_back(1.0f);
  coeff.values.push_back(99999.0f);
  std::vector<gu::Transform3> nodes;
  nodes.reserve(
      10000); // to have capacity enough to not allocate memory all the time
  std::vector<gu::Transform3> init_nodes;
  init_nodes.reserve(10);
  ros::Publisher pub =
      nh_.advertise<PointCloud>("multilevel_map/" + name, 1, false);
  levels_.emplace_back(
      Level{transform, pc_level, pub, nodes, init_nodes, coeff, false});
}

void PointCloudVisualizer::SegmentLevelsBasedOnGraph() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr graph_as_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  graph_as_cloud->reserve(pose_graph_.GetValues().size());
  std::vector<gtsam::Symbol> keyes;
  keyes.reserve(pose_graph_.GetValues().size());

  for (const auto& keyed_pose : pose_graph_.GetValues()) {
    const gtsam::Symbol key = keyed_pose.key;
    const gu::Transform3 pose =
        utils::ToGu(pose_graph_.GetPose(keyed_pose.key));
    pcl::PointXYZ pc;
    pc.x = pose.translation.X();
    pc.y = pose.translation.Y();
    pc.z = pose.translation.Z();
    graph_as_cloud->push_back(pc);
    keyes.push_back(key);
  }

  // Get segmentation ready
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1.5);
  //  seg.setMaxIterations(100);
  seg.setEpsAngle(0.1);
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));

  // Create pointcloud to publish inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int original_size(graph_as_cloud->height * graph_as_cloud->width);
  int n_planes(0);

  while (graph_as_cloud->height * graph_as_cloud->width > 20) {
    ROS_INFO_STREAM("Starting segmenting plane: " << n_planes);
    // Compute Standard deviation
    std::vector<gtsam::Symbol> keyes_updated;
    // Fit a plane
    seg.setInputCloud(graph_as_cloud);
    seg.segment(*inliers, *coefficients);
    ROS_INFO_STREAM("Level: "
                    << " " << coefficients->values[0] << " "
                    << coefficients->values[1] << " " << coefficients->values[2]
                    << " " << coefficients->values[3]);
    if (std::abs(coefficients->values[2]) < 0.97) {
      ROS_INFO_STREAM("Angle of the plane is off...");
      extract.setInputCloud(graph_as_cloud);
      extract.setIndices(inliers);

      pcl::PointIndices::Ptr all(new pcl::PointIndices);
      all->indices.resize(graph_as_cloud->size());
      std::iota(all->indices.begin(), all->indices.end(), 0);

      std::vector<int> diff;
      std::set_difference(all->indices.begin(),
                          all->indices.end(),
                          inliers->indices.begin(),
                          inliers->indices.end(),
                          std::inserter(diff, diff.begin()));
      keyes_updated.clear();
      for (const auto& diff_elem : diff) {
        keyes_updated.push_back(keyes[diff_elem]);
      }

      std::swap(keyes, keyes_updated);

      extract.setNegative(true);
      pcl::PointCloud<pcl::PointXYZ> cloudF;
      extract.filter(cloudF);
      graph_as_cloud->swap(cloudF);

      continue;
    }

    if (inliers->indices.size() < 20)
      break;

    if (n_planes >= levels_.size()) {
      CreateNewLevel();
    }

    double sigma(0);
    for (int i = 0; i < inliers->indices.size(); i++) {
      //      sigma += pow(err[i] - mean_error, 2);
      //      ROS_INFO_STREAM("index: " << inliers->indices[i]);
      //      // Get Point
      pcl::PointXYZ pt = graph_as_cloud->points[inliers->indices[i]];
      gu::Transform3 node;
      node.translation.data[0] = graph_as_cloud->points[inliers->indices[i]].x;
      node.translation.data[1] = graph_as_cloud->points[inliers->indices[i]].y;
      node.translation.data[2] = graph_as_cloud->points[inliers->indices[i]].z;
      levels_[n_planes].nodes_.push_back(node);
      PointCloud::Ptr temp_cloud(new PointCloud);
      GetTransformedPointCloudWorld(keyes[inliers->indices[i]],
                                    temp_cloud.get());
      *levels_[n_planes].points_ += *temp_cloud;
    }
    levels_[n_planes].coefficients_ = *coefficients;
    //    sigma = sqrt(sigma / inliers->indices.size());

    // Extract inliers

    extract.setInputCloud(graph_as_cloud);
    extract.setIndices(inliers);

    pcl::PointIndices::Ptr all(new pcl::PointIndices);
    all->indices.resize(graph_as_cloud->size());
    std::iota(all->indices.begin(), all->indices.end(), 0);

    std::vector<int> diff;
    std::set_difference(all->indices.begin(),
                        all->indices.end(),
                        inliers->indices.begin(),
                        inliers->indices.end(),
                        std::inserter(diff, diff.begin()));
    keyes_updated.clear();
    for (const auto& diff_elem : diff) {
      keyes_updated.push_back(keyes[diff_elem]);
    }

    std::swap(keyes, keyes_updated);

    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloudF;
    extract.filter(cloudF);
    graph_as_cloud->swap(cloudF);
    ROS_INFO_STREAM("Nodes in the level: "
                    << n_planes << " / " << levels_.size() << " "
                    << levels_[n_planes].nodes_.size()
                    << " Graph has: " << pose_graph_.GetValues().size());
    n_planes++;
  }

  std::sort(levels_.begin(), levels_.end(), [](const Level& x, const Level& y) {
    return (x.coefficients_.values[3] < y.coefficients_.values[3]);
  });
  ROS_INFO_STREAM("LEVELS SORTED:");
  for (const auto& level : levels_) {
    ROS_INFO_STREAM("Level: "
                    << " " << level.coefficients_.values[0] << " "
                    << level.coefficients_.values[1] << " "
                    << level.coefficients_.values[2] << " "
                    << level.coefficients_.values[3]);
  }
}

void PointCloudVisualizer::OptimizerUpdateCallback(
    const pose_graph_msgs::PoseGraphConstPtr& msg) {
  //  pose_graph_.UpdateFromMsg(msg);
  //  ROS_INFO_STREAM("OptimizerUpdateCallback!");
  //  for (auto& robot_point_cloud : robots_point_clouds_) {
  //    robot_point_cloud.second->clear();
  //  }

  //  for (auto& level : levels_) {
  //    level.nodes_.clear();
  //    level.points_->clear();
  //  }

  //  for (const auto& keyed_pose : pose_graph_.GetValues()) {
  //    const gtsam::Symbol key = keyed_pose.key;

  //    // Append the world-frame point cloud to the output.
  //    PointCloud::Ptr temp_cloud(new PointCloud);
  //    GetTransformedPointCloudWorld(key, temp_cloud.get());
  //    if (robots_point_clouds_.find(key.chr()) !=
  //    robots_point_clouds_.end())
  //    {
  //      *robots_point_clouds_.at(key.chr()) += *temp_cloud;
  //    }
  //  }
  //  SegmentLevelsBasedOnGraph();
  //  VisualizePointCloud();
}
