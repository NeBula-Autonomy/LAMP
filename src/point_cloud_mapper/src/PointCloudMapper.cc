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
 */

#include <parameter_utils/ParameterUtils.h>
#include <pcl/search/impl/search.hpp>
#include <point_cloud_mapper/PointCloudMapper.h>

namespace pu = parameter_utils;

PointCloudMapper::PointCloudMapper()
  : initialized_(false), map_updated_(false), incremental_unsubscribed_(false) {
  // Initialize map data container.
  map_data_.reset(new PointCloud);
}

PointCloudMapper::~PointCloudMapper() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

bool PointCloudMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMapper");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudMapper::LoadParameters(const ros::NodeHandle& n) {
  // Load fixed frame.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  map_data_->header.frame_id = fixed_frame_id_;

  // Load map parameters.
  if (!pu::Get("map/octree_resolution", octree_resolution_))
    return false;

  // Initialize the map octree.
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_data_);

  initialized_ = true;

  return true;
}

bool PointCloudMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  map_pub_ = nl.advertise<PointCloud>("octree_map", 10, false);
  incremental_map_pub_ =
      nl.advertise<PointCloud>("octree_map_updates", 10, false);
  map_frozen_pub_ = nl.advertise<PointCloud>("octree_map_frozen", 10, false);

  return true;
}

void PointCloudMapper::Reset() {
  map_data_.reset(new PointCloud);
  map_data_->header.frame_id = fixed_frame_id_;
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_data_);

  initialized_ = true;
}

bool PointCloudMapper::InsertPoints(const PointCloud::ConstPtr& points,
                                    PointCloud* incremental_points) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
    return false;
  }

  if (incremental_points == NULL) {
    ROS_ERROR("%s: Incremental point cloud argument is null.", name_.c_str());
    return false;
  }
  incremental_points->clear();

  // Try to get the map mutex from the publisher. If the publisher is using it,
  // we will just not insert this point cloud right now. It'll be added when the
  // map is regenerated by loop closure.
  if (map_mutex_.try_lock()) {
    double min_x, min_y, min_z, max_x, max_y, max_z;

    bool isInBox;
    // Iterate over points in the input point cloud, inserting them into the map
    // if there is not already a point in the same voxel.
    for (size_t ii = 0; ii < points->points.size(); ++ii) {
      const pcl::PointXYZ p = points->points[ii];
      map_octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
      isInBox = (p.x >= min_x && p.x <= max_x) &&
          (p.y >= min_y && p.y <= max_y) && (p.z >= min_z && p.z <= max_z);
      if (!isInBox || !map_octree_->isVoxelOccupiedAtPoint(p)) {
        // if (!map_octree_->isVoxelOccupiedAtPoint(p)) {
        map_octree_->addPointToCloud(p, map_data_);
        incremental_points->push_back(p);
        // if (p.x < min_x){min_x = p.x;}
        // if (p.y < min_y){min_y = p.y;}
        // if (p.z < min_z){min_z = p.z;}
        // if (p.x > max_x){max_x = p.x;}
        // if (p.y > max_y){max_y = p.y;}
        // if (p.z > max_z){max_z = p.z;}
      }
    }
    map_mutex_.unlock();
  } else {
    // This won't happen often.
    ROS_WARN(
        "%s: Failed to update map: map publisher has a hold of the thread. "
        "Turn off any subscriptions to the 3D map topic to prevent this from "
        "happening.",
        name_.c_str());
  }

  // Publish the incremental map update.
  incremental_points->header = points->header;
  incremental_points->header.frame_id = fixed_frame_id_;
  PublishMapUpdate(*incremental_points);

  map_updated_ = true;
  return true;
}

bool PointCloudMapper::ApproxNearestNeighbors(const PointCloud& points,
                                              PointCloud* neighbors) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
  }

  if (neighbors == NULL) {
    ROS_ERROR("%s: Output argument is null.", name_.c_str());
  }

  neighbors->points.clear();

  // Iterate over points in the input point cloud, finding the nearest neighbor
  // for every point and storing it in the output array.
  for (size_t ii = 0; ii < points.points.size(); ++ii) {
    // Search for nearest neighbor and store.
    float unused = 0.f;
    int result_index = -1;

    map_octree_->approxNearestSearch(points.points[ii], result_index, unused);
    if (result_index >= 0)
      neighbors->push_back(map_data_->points[result_index]);
  }

  return neighbors->points.size() > 0;
}

void PointCloudMapper::PublishMap() {
  if (initialized_ && map_updated_ && map_pub_.getNumSubscribers() > 0) {
    // Use a new thread to publish the map to avoid blocking main thread
    // on concurrent calls.
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
    publish_thread_ = std::thread(&PointCloudMapper::PublishMapThread, this);
  }
}

void PointCloudMapper::PublishMapThread() {
  map_mutex_.lock();
  map_pub_.publish(map_data_);

  // Don't publish again until we get another map update.
  map_updated_ = false;
  map_mutex_.unlock();
}

void PointCloudMapper::PublishMapFrozen() {
  ROS_INFO_STREAM("1");
  if (initialized_ && map_frozen_pub_.getNumSubscribers() > 0) {
    ROS_INFO_STREAM("2");
    // Use a new thread to publish the map to avoid blocking main thread
    // on concurrent calls.
    if (publish_frozen_thread_.joinable()) {
      ROS_INFO_STREAM("3");
      publish_frozen_thread_.join();
    }
    publish_frozen_thread_ = std::thread(&PointCloudMapper::PublishMapFrozenThread, this);
  }
}

void PointCloudMapper::PublishMapFrozenThread() {
  map_frozen_mutex_.lock();
  ROS_INFO_STREAM("Publishing frozen map");
  map_frozen_pub_.publish(map_data_);

  // Don't publish again until we get another map update.
  map_frozen_mutex_.unlock();
}

void PointCloudMapper::PublishMapUpdate(const PointCloud& incremental_points) {
  // Publish the incremental points for visualization.
  if (incremental_map_pub_.getNumSubscribers() > 0) {
    // Check if the incremental publisher was unsubscribed from recently (a user
    // might do this if a loop closure occured). If so, first draw the full map.
    if (incremental_unsubscribed_) {
      incremental_map_pub_.publish(*map_data_);
      incremental_unsubscribed_ = false;
    } else {
      incremental_map_pub_.publish(incremental_points);
    }
  } else {
    incremental_unsubscribed_ = true;
  }
}
