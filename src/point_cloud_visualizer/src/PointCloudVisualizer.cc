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

  PointCloud::Ptr scan(new PointCloud);
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

double CalculateEuclideanDistance(const gu::Transform3& current_pose,
                                  const gu::Transform3& node) {
  double dx2 = std::pow(current_pose.translation.X() - node.translation.X(), 2);
  double dy2 = std::pow(current_pose.translation.Y() - node.translation.Y(), 2);
  double dz2 = std::pow(current_pose.translation.Z() - node.translation.Z(), 2);

  return std::sqrt(dx2 + dy2 + dz2); // 1000.0 *
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

bool PointCloudVisualizer::AllLevelsAreInitialized() const {
  bool init = true;
  for (const auto& level : levels_) {
    init = init && level.IsInitialized();
  }
  return init;
}
size_t
PointCloudVisualizer::SelectLevelForNode2(const gtsam::Symbol current_key) {
  const gu::Transform3 current_pose =
      utils::ToGu(pose_graph_.GetPose(current_key));
  std::vector<std::pair<double, size_t>> potential_levels;
  ROS_INFO_STREAM("<=====================================================>");
  ROS_INFO_STREAM("Levels to check: " << levels_.size());
  if (AllLevelsAreInitialized()) {
    ROS_INFO_STREAM(" ALL NODES INITAILIZED");
    for (size_t i = 0; i < levels_.size(); ++i) {
      ROS_INFO_STREAM("Level "
                      << i << " has " << levels_[i].nodes_.size()
                      << " init nodes: " << levels_[i].init_nodes_.size());
      double distance =
          point2planedistnace(current_pose, levels_[i].coefficients_);
      ROS_INFO_STREAM("Distance: " << distance);
      if (distance < 2.0) {
        potential_levels.emplace_back(std::pair<double, size_t>(distance, i));
      }
    }
  } else {
    ROS_INFO_STREAM("NOT ALL NODES INITAILIZED");
    for (size_t i = 0; i < levels_.size(); ++i) {
      ROS_INFO_STREAM("Level "
                      << i << " has " << levels_[i].nodes_.size()
                      << " init nodes: " << levels_[i].init_nodes_.size());
      ROS_INFO_STREAM("Distance: "
                      << levels_[i].MinDistanceFromNodes(current_pose)
                      << " for " << i);
      potential_levels.emplace_back(std::pair<double, size_t>(
          levels_[i].MinDistanceFromNodes(current_pose), i));
    }
  }

  if (potential_levels.size() == 0) {
    selected_level = levels_.size();
    CreateNewLevel();
  } else {
    auto min_pair = *std::min_element(potential_levels.cbegin(),
                                      potential_levels.cend(),
                                      [](const std::pair<double, size_t>& lhs,
                                         const std::pair<double, size_t>& rhs) {
                                        return lhs.first < rhs.first;
                                      });
    ROS_INFO_STREAM("POTENTIAL LEVELS "
                    << potential_levels.size() << " LEVEL TO CHOOSE: "
                    << min_pair.first << " " << min_pair.second);
    selected_level = min_pair.second;
  }

  levels_[selected_level].UpdateNodeInTheLevel(current_pose);
  levels_[selected_level].EstimatePlane();

  for (size_t i = 0; i < levels_.size(); i++) {
    ROS_INFO_STREAM("Current "
                    "plane coefficients: "
                    << i << " : " << levels_[i].coefficients_.values[0] << " "
                    << levels_[i].coefficients_.values[1] << " "
                    << levels_[i].coefficients_.values[2] << " "
                    << levels_[i].coefficients_.values[3]);
  }
  return selected_level;
}

void PointCloudVisualizer::RefreshPlanes() {
  std::vector<size_t> to_remove;
  for (size_t i = 0; i < levels_.size(); i++) {
    if (levels_[i].AngleWithXYPlaneRad() > 0.75) {
      ROS_INFO_STREAM("REMOVING " << i << " DUE TO ANGLE");
      to_remove.push_back(i);
      continue;
    }
    if (std::find(to_remove.begin(), to_remove.end(), i) == to_remove.end()) {
      for (size_t j = i + 1; j < levels_.size(); j++) {
        if (std::abs(levels_[i].coefficients_.values[3] -
                     levels_[j].coefficients_.values[3]) < 0.5f) {
          to_remove.push_back(j);
          ROS_INFO_STREAM("REMOVING " << j << " DUE TO being close");
          levels_[i].nodes_.insert(levels_[i].nodes_.end(),
                                   levels_[j].nodes_.begin(),
                                   levels_[j].nodes_.end());
          *levels_[i].points_ += *levels_[j].points_;
          levels_[i].EstimatePlane();
        }
      }
    }
  }

  for (int i = to_remove.size() - 1; i >= 0; i--) {
    levels_.erase(levels_.begin() + to_remove[i]);
  }
  std::sort(levels_.begin(), levels_.end(), [](const Level& x, const Level& y) {
    return (x.coefficients_.values[3] < y.coefficients_.values[3]);
  });
}

void PointCloudVisualizer::AddPointCloudToCorrespondingLevel2(
    const gtsam::Symbol key, const PointCloud::Ptr& pc) {
  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %s does not exist in values in GetTransformedPointCloudWorld",
             gtsam::DefaultKeyFormatter(key).c_str());
    return;
  }

  size_t selected_level = SelectLevelForNode2(key);

  if (levels_[selected_level].init_nodes_.size() > 15) {
    *levels_[selected_level].points_ += *pc;
  }

  if (levels_[selected_level].nodes_.size() != 0) {
    if (levels_[selected_level].nodes_.size() % 5 == 0 and levels_.size() > 1) {
      //      ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      //      ROS_INFO_STREAM("REFRESHING PLANES!");
      //      ROS_INFO_STREAM("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      //      RefreshPlanes();
    }
  }
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

void PointCloudVisualizer::Level::EstimatePlane() {
  if (nodes_.size() == 0)
    return;
  if (nodes_.size() % 5 != 0)
    return;
  ROS_INFO_STREAM("ESTIMATING!");
  Eigen::MatrixXd A(nodes_.size(), 4);
  ROS_INFO_STREAM("ESTIMATING!");
  for (size_t i = 0; i < nodes_.size(); i++) {
    A(i, 0) = nodes_[i].translation.X();
    A(i, 1) = nodes_[i].translation.Y();
    A(i, 2) = nodes_[i].translation.Z();
    A(i, 3) = 1.0;
  }
  ROS_INFO_STREAM("ESTIMATING!");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  ROS_INFO_STREAM("SVD: \n " << svd.matrixV().col(3));
  coefficients_.values[0] = svd.matrixV().col(3)(0);
  coefficients_.values[1] = svd.matrixV().col(3)(1);
  coefficients_.values[2] = svd.matrixV().col(3)(2);
  coefficients_.values[3] = svd.matrixV().col(3)(3);

  auto norm = std::sqrt(std::pow(coefficients_.values[0], 2) +
                        std::pow(coefficients_.values[1], 2) +
                        std::pow(coefficients_.values[2], 2));

  for (size_t i = 0; i < coefficients_.values.size(); i++) {
    coefficients_.values[i] = coefficients_.values[i] / norm;
  }

  ROS_INFO_STREAM("Estimate plane: " << coefficients_.values[0] << " "
                                     << coefficients_.values[1] << " "
                                     << coefficients_.values[2] << " "
                                     << coefficients_.values[3]);
}

void PointCloudVisualizer::Level::UpdateNodeInTheLevel(
    const gu::Transform3& node) {
  if (init_nodes_.size() < 20) {
    init_nodes_.push_back(node);
  }
  if (init_nodes_.size() >= 15) {
    nodes_.push_back(node);
    bounding_box_.Update(Eigen::Vector3d(
        node.translation.X(), node.translation.Y(), node.translation.Z()));
  }
}

double PointCloudVisualizer::Level::MinDistanceFromNodes(
    const gu::Transform3& node) const {
  double min_distance = std::numeric_limits<double>::infinity();
  double distance = 0.0;
  for (size_t k = 0; k < init_nodes_.size(); ++k) {
    distance = CalculateEuclideanDistance(node, init_nodes_[k]);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  for (size_t k = 0; k < nodes_.size(); ++k) {
    distance = CalculateEuclideanDistance(node, nodes_[k]);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

double PointCloudVisualizer::Level::AngleWithXYPlaneRad() const {
  Eigen::Vector3d plane_normal{static_cast<double>(coefficients_.values[0]),
                               static_cast<double>(coefficients_.values[1]),
                               static_cast<double>(coefficients_.values[2])};
  plane_normal.normalize();
  return std::acos(plane_normal.dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
}

void PointCloudVisualizer::BoundingBox::Update(const Eigen::Vector3d& vec) {
  if (vec[0] > max_pt[0]) {
    max_pt[0] = vec[0];
  }
  if (vec[1] > max_pt[1]) {
    max_pt[1] = vec[1];
  }
  if (vec[2] > max_pt[2]) {
    max_pt[2] = vec[2];
  }
  if (vec[0] < min_pt[0]) {
    min_pt[0] = vec[0];
  }
  if (vec[1] < min_pt[1]) {
    min_pt[1] = vec[1];
  }
  if (vec[2] < min_pt[2]) {
    min_pt[2] = vec[2];
  }
}
