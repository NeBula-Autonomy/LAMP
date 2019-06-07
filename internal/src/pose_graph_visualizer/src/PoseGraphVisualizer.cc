#include <pose_graph_visualizer/PoseGraphVisualizer.h>

#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/Empty.h>
#include <parameter_utils/ParameterUtils.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <fstream>

#include <time.h>

namespace pu = parameter_utils;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

inline geometry_msgs::Point tfpoint2msg(const tf::Vector3 &v) {
  geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

geometry_msgs::Point PoseGraphVisualizer::GetPositionMsg(unsigned int key) const {
  return tfpoint2msg(keyed_poses_.at(key).getOrigin());
}

bool PoseGraphVisualizer::Initialize(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
{
  ROS_INFO("PoseGraphVisualizer: Initializing");
  name_ = ros::names::append(pnh.getNamespace(), "PoseGraphVisualizer");

  if (!LoadParameters(pnh))
  {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(nh, pnh))
  {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  ROS_INFO("PoseGraphVisualizer: Initializing complete");

  return true;
}

bool PoseGraphVisualizer::LoadParameters(const ros::NodeHandle &n)
{
  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;

  // Load proximity threshold from LaserLoopClosure node
  if (!pu::Get("proximity_threshold", proximity_threshold_)) return false;

  //Initialize interactive marker server
  if (publish_interactive_markers_)
  {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "interactive_node", "", false));
  }
  return true;
}

bool PoseGraphVisualizer::RegisterCallbacks(const ros::NodeHandle &nh_, const ros::NodeHandle &pnh_)
{
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle pnh(pnh_);

  ros::NodeHandle nh(nh_);

  highlight_node_srv_ = pnh.advertiseService("highlight_node",
                                            &PoseGraphVisualizer::HighlightNodeService, this);
  highlight_edge_srv_ = pnh.advertiseService("highlight_edge",
                                            &PoseGraphVisualizer::HighlightEdgeService, this);

  odometry_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
  loop_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
  graph_node_pub_ =
      pnh.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
  graph_node_id_pub_ =
      pnh.advertise<visualization_msgs::Marker>("graph_node_ids", 10, false);
  keyframe_node_pub_ =
      pnh.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
  closure_area_pub_ =
      pnh.advertise<visualization_msgs::Marker>("closure_area", 10, false);
  highlight_pub_ =
      pnh.advertise<visualization_msgs::Marker>("confirm_edge", 10, false);

  keyed_scan_sub_ =
      nh.subscribe<pose_graph_msgs::KeyedScan>("blam_slam/keyed_scans", 10,
                                               &PoseGraphVisualizer::KeyedScanCallback, this);
  pose_graph_sub_ =
      nh.subscribe<pose_graph_msgs::PoseGraph>("blam_slam/pose_graph", 10,
                                               &PoseGraphVisualizer::PoseGraphCallback, this);
  pose_graph_edge_sub_ =
      nh.subscribe<pose_graph_msgs::PoseGraphEdge>("blam_slam/pose_graph_edge", 10,
                                                   &PoseGraphVisualizer::PoseGraphEdgeCallback, this);
  pose_graph_node_sub_ =
      nh.subscribe<pose_graph_msgs::PoseGraphNode>("blam_slam/pose_graph_node", 10,
                                               &PoseGraphVisualizer::PoseGraphNodeCallback, this);

  return true;
}

void PoseGraphVisualizer::PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr &msg) {
  keyed_poses_.clear();
  odometry_edges_.clear();
  for (const auto &msg_node : msg->nodes) {
    tf::Pose pose;
    tf::poseMsgToTF(msg_node.pose, pose);
    keyed_poses_[msg_node.key] = pose;
  }

  for (const auto &msg_edge : msg->edges) {
    odometry_edges_.emplace_back(std::make_pair(msg_edge.key_from, msg_edge.key_to));
  }

  VisualizePoseGraph();
}

void PoseGraphVisualizer::PoseGraphNodeCallback(const pose_graph_msgs::PoseGraphNode::ConstPtr &msg) {
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);
  keyed_poses_[msg->key] = pose;
}

void PoseGraphVisualizer::PoseGraphEdgeCallback(const pose_graph_msgs::PoseGraphEdge::ConstPtr &msg) {
  odometry_edges_.emplace_back(std::make_pair(msg->key_from, msg->key_to));
}

void PoseGraphVisualizer::KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr &msg)
{
  const unsigned int key = msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end())
  {
    ROS_ERROR("%s: Key %u already has a laser scan.", name_.c_str(), key);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg->scan, *scan);

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (key == 0)
  {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key, stamp));
    stamps_keyed_.insert(std::pair<double, unsigned int>(stamp.toSec(), key));
  }

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<unsigned int, PointCloud::ConstPtr>(key, scan));
}

std::string GenerateKey(unsigned int key1, unsigned int key2) {
  return std::to_string(key1) + '|' + std::to_string(key2);
}

bool PoseGraphVisualizer::HighlightEdge(unsigned int key1, unsigned int key2)
{
  ROS_INFO("Highlighting factor between %i and %i.", key1, key2);

  if (!KeyExists(key1) || !KeyExists(key2))
  {
    ROS_WARN("Key %i or %i does not exist.", key1, key2);
    return false;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_ + "edge" + GenerateKey(key1, key2);
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.scale.x = 0.05;

  m.points.push_back(GetPositionMsg(key1));
  m.points.push_back(GetPositionMsg(key2));
  highlight_pub_.publish(m);

  HighlightNode(key1);
  HighlightNode(key2);

  return true;
}

bool PoseGraphVisualizer::HighlightNode(unsigned int key)
{
  ROS_INFO("Highlighting node %i.", key);

  if (!KeyExists(key))
  {
    ROS_WARN("Key %i does not exist.", key);
    return false;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_ + "node" + std::to_string(key);
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.scale.x = 0.27;
  m.scale.y = 0.27;
  m.scale.z = 0.27;
  m.id = 0;
  m.pose.position = GetPositionMsg(key);
  highlight_pub_.publish(m);

  return true;
}

void PoseGraphVisualizer::UnhighlightEdge(unsigned int key1, unsigned int key2)
{
  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_ + "edge" + GenerateKey(key1, key2);
  m.id = 0;
  if (key1 == key2 && key1 == 0)
    m.action = visualization_msgs::Marker::DELETEALL;
  else
    m.action = visualization_msgs::Marker::DELETE;
  highlight_pub_.publish(m);

  UnhighlightNode(key1);
  UnhighlightNode(key2);
}

void PoseGraphVisualizer::UnhighlightNode(unsigned int key)
{
  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_ + "node" + std::to_string(key);
  m.id = 0;
  if (key == 0)
    m.action = visualization_msgs::Marker::DELETEALL;
  else
    m.action = visualization_msgs::Marker::DELETE;
  highlight_pub_.publish(m);
}

bool PoseGraphVisualizer::HighlightNodeService(
  pose_graph_visualizer::HighlightNodeRequest &request,
  pose_graph_visualizer::HighlightNodeResponse &response) {
  if (request.highlight) {
    response.success = HighlightNode(request.key);
  } else {
    UnhighlightNode(request.key);
    response.success = true;
  }
  return true;
}

bool PoseGraphVisualizer::HighlightEdgeService(
  pose_graph_visualizer::HighlightEdgeRequest &request,
  pose_graph_visualizer::HighlightEdgeResponse &response) {
  if (request.highlight) {
    response.success = HighlightEdge(request.key_from, request.key_to);
  } else {
    UnhighlightEdge(request.key_from, request.key_to);
    response.success = true;
  }
  return true;
}

// Interactive Marker Menu
void PoseGraphVisualizer::MakeMenuMarker(const tf::Pose &position, const std::string &id_number)
{
  interactive_markers::MenuHandler menu_handler;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = PoseGraphVisualizer::fixed_frame_id_;
  int_marker.scale = 1.0;
  tf::poseTFToMsg(position, int_marker.pose);
  int_marker.name = id_number;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  server->insert(int_marker);
  menu_handler.apply(*server, int_marker.name);
}

void PoseGraphVisualizer::VisualizePoseGraph()
{
  // Publish odometry edges.
  if (odometry_edge_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii)
    {
      const auto key1 = odometry_edges_[ii].first;
      const auto key2 = odometry_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1));
      m.points.push_back(GetPositionMsg(key2));
    }
    odometry_edge_pub_.publish(m);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.0;
    m.color.g = 0.2;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii)
    {
      const auto key1 = loop_edges_[ii].first;
      const auto key2 = loop_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1));
      m.points.push_back(GetPositionMsg(key2));
    }
    loop_edge_pub_.publish(m);
  }

  // Publish nodes in the pose graph.
  if (graph_node_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 2;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.3;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    for (const auto &keyedPose : keyed_poses_)
    {
      m.points.push_back(GetPositionMsg(keyedPose.first));
    }
    graph_node_pub_.publish(m);
  }

  // Publish node IDs in the pose graph.
  if (graph_node_id_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;

    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    m.scale.z = 0.02; // Only Scale z is used - height of capital A in the text

    int id_base = 100;
    int counter = 0;
    for (const auto &keyedPose : keyed_poses_)
    {
      tf::poseTFToMsg(keyedPose.second, m.pose);
      // Display text for the node
      m.text = std::to_string(keyedPose.first);
      m.id = id_base + keyedPose.first;
      graph_node_id_pub_.publish(m);
    }
  }

  // Publish keyframe nodes in the pose graph.
  if (keyframe_node_pub_.getNumSubscribers() > 0)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 3;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.3;
    m.color.a = 0.8;
    m.scale.x = 0.25;
    m.scale.y = 0.25;
    m.scale.z = 0.25;

    for (const auto &keyedScan : keyed_scans_) {
      m.points.push_back(GetPositionMsg(keyedScan.first));
    }
    keyframe_node_pub_.publish(m);
  }

  
  // Draw a sphere around the current sensor frame to show the area in which we
  // are checking for loop closures.
  if (closure_area_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = base_frame_id_;
    m.ns = base_frame_id_;
    m.id = 4;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;
    m.color.r = 0.0;
    m.color.g = 0.4;
    m.color.b = 0.8;
    m.color.a = 0.4;
    m.scale.x = proximity_threshold_ * 2.0;
    m.scale.y = proximity_threshold_ * 2.0;
    m.scale.z = proximity_threshold_ * 2.0;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    closure_area_pub_.publish(m);
  }

  //Interactive Marker
  if (publish_interactive_markers_)
  {
    for (const auto &keyed_pose : keyed_poses_)
    {
      if (keyed_pose.first % 1 == 0)
      {
        MakeMenuMarker(keyed_pose.second, std::to_string(keyed_pose.first));
      }
    }
    if (server != nullptr){
      server->applyChanges();
    }
  }
}
