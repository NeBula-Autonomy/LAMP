// Copyright (C)
// All rights reserved.
//
// This file is part of a free software package: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation, either version 3 of the License,
// or at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script.
// Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts.

#ifndef _pose_graph_tools_h_
#define _pose_graph_tools_h_

#include "pose_graph_tools/PoseGraphToolsLib.h"

// ROS
#include <ros/package.h>
#include <ros/ros.h>

// boost
#include <boost/bind.hpp>

// point cloud mapper
#include <point_cloud_mapper/PointCloudMapper.h>
// dynamic reconfigure server include
#include <dynamic_reconfigure/server.h>
// pose graph message
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>

namespace pose_graph_tools {

// Typedef for stored point clouds.
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/**
 * \brief Algorithm PoseGraphToolsNode
 *
 */
class PoseGraphToolsNode {
 private:
  // Candidate node to apply the new transformation
  uint64_t node_candidate_key_;

  // Keyed scans
  std::map<uint64_t, PointCloud::ConstPtr> keyed_scans;

  // [publisher attributes]
  ros::Publisher pose_graph_out_publisher_;
  pose_graph_msgs::PoseGraph pose_graph_out_msg_;

  // [subscriber attributes]
  pose_graph_msgs::PoseGraph pose_graph_in_msg_;
  ros::Subscriber pose_graph_in_subscriber_;
  ros::Subscriber keyed_scan_subscriber_;
  ros::Subscriber selected_node_subscriber_;
  void pose_graph_in_callback(const pose_graph_msgs::PoseGraph::ConstPtr& msg);
  pthread_mutex_t pose_graph_in_mutex_;
  void pose_graph_in_mutex_enter(void);
  void pose_graph_in_mutex_exit(void);

  // Tools and functions for this node
  PoseGraphToolsLib pgt_lib_;

  // Mapper
  PointCloudMapper mapper_;

  Config config_;
  dynamic_reconfigure::Server<Config> dsrv_;


 public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  PoseGraphToolsNode(ros::NodeHandle& nh);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~PoseGraphToolsNode(void);

  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop rate attribute.
   */
  void mainNodeThread(void);

 protected:
  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   * \param config an object with new configuration from all algorithm
   *               parameters defined in the config file.
   * \param level  integer referring the level in which the configuration
   *               has been changed.
   */
  void DynRecCallback(Config& config, uint32_t level = 0);

  void KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& msg);

  void SelectNodeCallback(const std_msgs::String::ConstPtr& msg);

  // Map generation functions (regenerating map after modifying posegraph)
  bool ReGenerateMapPointCloud();

  // For combining all the scans together
  bool CombineKeyedScansWorld(PointCloud* points);

  // Transform the point cloud to world frame
  bool GetTransformedPointCloudWorld(const uint64_t& key, PointCloud* points);

  // For adding one scan to the map
  bool AddTransformedPointCloudToMap(const uint64_t& key);
};

}  // namespace pose_graph_tools

#endif
