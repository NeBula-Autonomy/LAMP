#include "pose_graph_tools/PoseGraphTools.h"

#include <limits>

#include <geometry_msgs/Point.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <utils/CommonFunctions.h>
#include <utils/PrefixHandling.h>

using Eigen::AngleAxisd;
using Eigen::Translation;
using Eigen::Vector3d;

namespace gu = geometry_utils;

namespace pose_graph_tools {

PoseGraphToolsNode::PoseGraphToolsNode(ros::NodeHandle& nh)
    : node_candidate_key_(0) {
  ROS_INFO("Initializing Pose Graph Tools");
  ROS_INFO("Pose Graph Tools: Initializing dynamic reconfigure");
  // Dynamic Reconfigure
  dynamic_reconfigure::Server<Config>::CallbackType dsrv_cb;
  dsrv_cb = boost::bind(&PoseGraphToolsNode::DynRecCallback, this, _1, _2);
  dsrv_.setCallback(dsrv_cb);
  ROS_INFO("Initializing mapper");
  mapper_.Initialize(nh);

  ROS_INFO("Pose Graph Tools: Setting publisher and subscriber");
  // [init publishers]
  this->pose_graph_out_publisher_ =
      nh.advertise<pose_graph_msgs::PoseGraph>("pose_graph_out", 1);

  // [init subscribers]
  this->pose_graph_in_subscriber_ = nh.subscribe(
      "pose_graph_in", 1, &PoseGraphToolsNode::pose_graph_in_callback, this);
  this->keyed_scan_subscriber_ = nh.subscribe(
      "keyed_scan", 1, &PoseGraphToolsNode::KeyedScanCallback, this);
  this->clicked_point_subscriber_ = nh.subscribe(
      "clicked_point", 1, &PoseGraphToolsNode::ClickedPointCallback, this);
  pthread_mutex_init(&this->pose_graph_in_mutex_, NULL);
}

PoseGraphToolsNode::~PoseGraphToolsNode(void) {
  // [free dynamic memory]
  pthread_mutex_destroy(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::mainNodeThread(void) {
  if (this->node_candidate_key_ != 0) {
    this->pgt_lib_.lock();
    // Get entries from dynamic reconfigure
    double dx = this->config_.dx;
    double dy = this->config_.dy;
    double dz = this->config_.dz;
    double droll = this->config_.droll;
    double dpitch = this->config_.dpitch;
    double dyaw = this->config_.dyaw;
    // Transform to eigen transform type
    HTransf d_pose = Translation<double, 3>(dx, dy, dz) *
                     AngleAxisd(dyaw / 180 * M_PI, Vector3d::UnitZ()) *
                     AngleAxisd(dpitch / 180 * M_PI, Vector3d::UnitY()) *
                     AngleAxisd(droll / 180 * M_PI, Vector3d::UnitX());
    ROS_DEBUG_STREAM("Modifying key: "
                     << gtsam::DefaultKeyFormatter(this->node_candidate_key_));
    // Update node and posegraph
    this->pose_graph_in_mutex_enter();
    this->pose_graph_out_msg_ = this->pgt_lib_.updateNodePosition(
        this->pose_graph_in_msg_, this->node_candidate_key_, d_pose);
    this->pose_graph_in_mutex_exit();
    this->pgt_lib_.unlock();
    ReGenerateMapPointCloud();
  } else {
    this->pose_graph_out_msg_ = this->pose_graph_in_msg_;
    mapper_.PublishMap();
  }
  this->pose_graph_out_publisher_.publish(this->pose_graph_out_msg_);
}

/*  [subscriber callbacks] */
void PoseGraphToolsNode::pose_graph_in_callback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  ROS_DEBUG("PoseGraphToolsNode::pose_graph_in_callback: New Message Received");
  this->pose_graph_in_mutex_enter();
  pose_graph_msgs::PoseGraphConstPtr old_graph =
      pose_graph_msgs::PoseGraphConstPtr(
          new pose_graph_msgs::PoseGraph(this->pose_graph_in_msg_));
  this->pgt_lib_.lock();
  // Merge newly recieved graph with corrected old graph
  this->pose_graph_in_msg_ = this->pgt_lib_.addNewIncomingGraph(msg, old_graph);
  this->pose_graph_lamp_ = *msg;
  this->pgt_lib_.unlock();
  this->pose_graph_in_mutex_exit();
}

void PoseGraphToolsNode::pose_graph_in_mutex_enter(void) {
  pthread_mutex_lock(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::pose_graph_in_mutex_exit(void) {
  pthread_mutex_unlock(&this->pose_graph_in_mutex_);
}

void PoseGraphToolsNode::DynRecCallback(Config& config, uint32_t level) {
  this->config_ = config;
  if (config.reset) {
    // Reset sliders
    this->dsrv_.getConfigDefault(config);
    this->dsrv_.updateConfig(config);
    this->config_ = config;
    // Reset and publish pose graph
    this->pose_graph_in_mutex_enter();
    this->pose_graph_in_msg_ = this->pose_graph_lamp_;
    this->pose_graph_out_msg_ = this->pose_graph_lamp_;
    this->pose_graph_in_mutex_exit();
    this->pgt_lib_.reset();
    ReGenerateMapPointCloud();
    this->pose_graph_out_publisher_.publish(this->pose_graph_out_msg_);
  }
}

void PoseGraphToolsNode::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& msg) {
  PointCloud::Ptr scan_ptr(new PointCloud);
  pcl::fromROSMsg(msg->scan, *scan_ptr);
  keyed_scans[msg->key] = scan_ptr;
  AddTransformedPointCloudToMap(msg->key);
}

void PoseGraphToolsNode::ClickedPointCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  geometry_msgs::Point p = msg->point;
  // Search for the closest node based on the x, y, selected
  double closest_dist = std::numeric_limits<double>::infinity();
  size_t idx_closest = 0; 
  this->pose_graph_in_mutex_enter();
  for (size_t i = 0; i < this->pose_graph_out_msg_.nodes.size(); i++) {
    geometry_msgs::Point node_position =
        this->pose_graph_out_msg_.nodes[i].pose.position;
    double dist = std::sqrt((node_position.x - p.x) * (node_position.y - p.y) +
                            (node_position.y - p.y) * (node_position.y - p.y) +
                            (node_position.z - p.z) * (node_position.z - p.z));
    if (dist < closest_dist) {
      idx_closest = i; 
      closest_dist = dist;
    }
  }
  this->pose_graph_in_mutex_exit();
  uint64_t candidate_key = this->pose_graph_out_msg_.nodes[idx_closest].key;
  // When switching to tune another key, update store current correction
  if (candidate_key != this->node_candidate_key_) {
    ROS_INFO_STREAM(
        "Reconfiguring key: " << gtsam::DefaultKeyFormatter(candidate_key));
    this->pose_graph_in_mutex_enter();
    this->pose_graph_in_msg_ = this->pose_graph_out_msg_;
    this->pose_graph_in_mutex_exit();
    this->node_candidate_key_ = candidate_key;
    // Reset sliders
    this->dsrv_.getConfigDefault(this->config_);
    this->dsrv_.updateConfig(this->config_);
  }
}

//------------------------------------------------------------------------------------------
// Map generation functions
//------------------------------------------------------------------------------------------

bool PoseGraphToolsNode::ReGenerateMapPointCloud() {
  // Reset the map
  mapper_.Reset();
  // Combine the keyed scans with the latest node values
  PointCloud::Ptr regenerated_map(new PointCloud);
  CombineKeyedScansWorld(regenerated_map.get());
  // Insert points into the map (publishes incremental point clouds)
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());
  // Publish map
  mapper_.PublishMap();
}

// For combining all the scans together
bool PoseGraphToolsNode::CombineKeyedScansWorld(PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("Pose Graph Tools: Output point cloud container is null.");
    return false;
  }
  points->points.clear();

  // Iterate over poses in the graph, transforming their corresponding laser
  // scans into world frame and appending them to the output.
  for (const auto& node : this->pose_graph_out_msg_.nodes) {
    uint64_t key = node.key;

    PointCloud::Ptr scan_world(new PointCloud());

    // Transform the body-frame scan into world frame.
    GetTransformedPointCloudWorld(key, scan_world.get());

    // Append the world-frame point cloud to the output.
    *points += *scan_world;
  }
}

// Transform the point cloud to world frame
bool PoseGraphToolsNode::GetTransformedPointCloudWorld(const uint64_t& key,
                                                       PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("Pose Graph Tools: Output point cloud container is null.");
    return false;
  }
  points->points.clear();

  gu::Transform3 pose;
  for (size_t i = 0; i < this->pose_graph_out_msg_.nodes.size(); i++) {
    if (this->pose_graph_out_msg_.nodes[i].key == key) {
      pose =
          utils::ToGu(utils::ToGtsam(this->pose_graph_out_msg_.nodes[i].pose));
      break;
    }
  }

  Eigen::Matrix4d b2w;
  b2w.setZero();
  b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
  b2w.block(0, 3, 3, 1) = pose.translation.Eigen();
  b2w(3, 3) = 1;

  Eigen::Quaterniond quat(pose.rotation.Eigen());
  quat.normalize();
  b2w.block(0, 0, 3, 3) = quat.matrix();
  // Check if scan exists for key
  if (keyed_scans.find(key) == keyed_scans.end()) {
    ROS_DEBUG_STREAM("Key " << gtsam::DefaultKeyFormatter(key)
                            << " does not have a scan");
    return false;
  }
  // Transform the body-frame scan into world frame.
  pcl::transformPointCloud(*keyed_scans[key], *points, b2w);
}

// For adding one scan to the map
bool PoseGraphToolsNode::AddTransformedPointCloudToMap(const uint64_t& key) {
  PointCloud::Ptr points(new PointCloud);

  GetTransformedPointCloudWorld(key, points.get());

  ROS_DEBUG_STREAM("Points size is: " << points->points.size()
                                     << ", in AddTransformedPointCloudToMap");

  // Add to the map
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(points, unused.get());

  return true;
}

}  // namespace pose_graph_tools