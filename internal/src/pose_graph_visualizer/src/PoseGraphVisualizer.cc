#include <pose_graph_visualizer/PoseGraphVisualizer.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

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

geometry_msgs::Point PoseGraphVisualizer::GetPositionMsg(
    long unsigned int key,
    const std::map<long unsigned int, tf::Pose>& poses) const {
      ROS_INFO("In getPositionMsg");
  return tfpoint2msg(poses.at(key).getOrigin());
}

bool PoseGraphVisualizer::Initialize(const ros::NodeHandle &nh,
                                     const ros::NodeHandle &pnh) {
  ROS_INFO("PoseGraphVisualizer: Initializing");
  name_ = ros::names::append(pnh.getNamespace(), "PoseGraphVisualizer");

  if (!LoadParameters(pnh)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(nh, pnh)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  ROS_INFO("PoseGraphVisualizer: Initializing complete");

  return true;
}

bool PoseGraphVisualizer::LoadParameters(const ros::NodeHandle &n) {
  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("frame_id/artifacts_in_global", artifacts_in_global_))
    return false;

  // Load proximity threshold from LaserLoopClosure node
  if (!pu::Get("proximity_threshold", proximity_threshold_))
    return false;

  // Initialize interactive marker server
  if (publish_interactive_markers_) {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "interactive_node", "", false));
  }
  return true;
}

bool PoseGraphVisualizer::RegisterCallbacks(const ros::NodeHandle &nh_,
                                            const ros::NodeHandle &pnh_) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle pnh(pnh_);

  ros::NodeHandle nh(nh_);

  highlight_node_srv_ = pnh.advertiseService(
      "highlight_node", &PoseGraphVisualizer::HighlightNodeService, this);
  highlight_edge_srv_ = pnh.advertiseService(
      "highlight_edge", &PoseGraphVisualizer::HighlightEdgeService, this);

  odometry_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
  loop_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
  artifact_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("artifact_edges", 10, false);
  uwb_edge_pub_ =
      pnh.advertise<visualization_msgs::Marker>("uwb_edges", 10, false);
  uwb_node_pub_ =
      pnh.advertise<visualization_msgs::Marker>("uwb_nodes", 10, false);
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
  artifact_marker_pub_ =
      pnh.advertise<visualization_msgs::Marker>("artifact_markers", 10, false);

  keyed_scan_sub_ = nh.subscribe<pose_graph_msgs::KeyedScan>(
      "blam_slam/keyed_scans", 10, &PoseGraphVisualizer::KeyedScanCallback,
      this);
  pose_graph_sub_ = nh.subscribe<pose_graph_msgs::PoseGraph>(
      "blam_slam/pose_graph", 10, &PoseGraphVisualizer::PoseGraphCallback,
      this);
  pose_graph_edge_sub_ = nh.subscribe<pose_graph_msgs::PoseGraphEdge>(
      "blam_slam/pose_graph_edge", 10,
      &PoseGraphVisualizer::PoseGraphEdgeCallback, this);
  pose_graph_node_sub_ = nh.subscribe<pose_graph_msgs::PoseGraphNode>(
      "blam_slam/pose_graph_node", 10,
      &PoseGraphVisualizer::PoseGraphNodeCallback, this);
  erase_posegraph_sub_ =
      nh.subscribe<std_msgs::Bool>("blam_slam/erase_posegraph",
                                   10,
                                   &PoseGraphVisualizer::ErasePosegraphCallback,
                                   this);

  remove_factor_viz_sub_ = nh.subscribe<std_msgs::Bool>(
      "blam_slam/remove_factor_viz",
      10,
      &PoseGraphVisualizer::RemoveFactorVizCallback,
      this);

  artifact_sub_ = nh.subscribe("blam_slam/artifact_global",
                               10,
                               &PoseGraphVisualizer::ArtifactCallback,
                               this);

  return true;
}

void PoseGraphVisualizer::PoseGraphCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr &msg) {
  if (!msg->incremental) {
    keyed_poses_.clear();
    odometry_edges_.clear();
  }
  for (const pose_graph_msgs::PoseGraphNode &msg_node : msg->nodes) {
    tf::Pose pose;
    tf::poseMsgToTF(msg_node.pose, pose);

    gtsam::Symbol sym_key(gtsam::Key(msg_node.key));

    // ROS_INFO_STREAM("Symbol key is " << gtsam::DefaultKeyFormatter(sym_key));
    // ROS_INFO_STREAM("Symbol key (directly) is "
    //                 << gtsam::DefaultKeyFormatter(msg_node.key));

    // ROS_INFO_STREAM("Symbol key (int) is " << msg_node.key);

    // Add UUID if an artifact or uwb node
    if (sym_key.chr() == ('l' || 'm' || 'n' || 'o' || 'p')) { 
      ROS_INFO_STREAM("Have an artifact node with key " << gtsam::DefaultKeyFormatter(sym_key));
      // Artifact
      keyed_artifact_poses_[msg_node.key] = pose;

      // node.ID = artifacts_[keyed_pose.key].msg.id;
      artifact_id2key_hash_[msg_node.ID] = msg_node.key;
      artifact_key2id_hash_[msg_node.key] = msg_node.ID;
      continue;
    }

    if (sym_key.chr() == 'u') {
      // UWB
      // TODO implement UWB logic
      // cast uint64_t from the message to an long unsigned int to ensure
      // comparisons are correct (do this explicitly here)
      keyed_uwb_poses_[static_cast<long unsigned int>(msg_node.key)] = pose;
      // node.ID = uwb_key2id_hash_[keyed_pose.key];
      continue;
    }

    // Fill pose nodes (representing the robot position)
    keyed_poses_[msg_node.key] = pose;

    keyed_stamps_.insert(std::pair<long unsigned int, ros::Time>(
        msg_node.key, msg_node.header.stamp));
    stamps_keyed_.insert(std::pair<double, long unsigned int>(
        msg_node.header.stamp.toSec(), msg_node.key));
  }

  for (const auto &msg_edge : msg->edges) {
    if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ODOM) {
      odometry_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      artifact_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB) {
      // TODO send only incremental UWB edges (if msg.incremental is true)
      // uwb_edges_.clear();
      bool found = false;
      for (const auto& edge : uwb_edges_) {
        // cast to long unsigned int to ensure comparisons are correct
        if (edge.first == static_cast<long unsigned int>(msg_edge.key_from) &&
            edge.second == static_cast<long unsigned int>(msg_edge.key_to)) {
          found = true;
          ROS_DEBUG("PGV: UWB edge from %u to %u already exists.",
                    msg_edge.key_from,
                    msg_edge.key_to);
          break;
        }
      }
      // avoid duplicate UWB edges
      if (!found) {
        uwb_edges_.emplace_back(
            std::make_pair(static_cast<long unsigned int>(msg_edge.key_from),
                           static_cast<long unsigned int>(msg_edge.key_to)));
        ROS_INFO("PGV: Adding new UWB edge from %u to %u.",
                 msg_edge.key_from,
                 msg_edge.key_to);
      }
    }
  }

  VisualizePoseGraph();
}

void PoseGraphVisualizer::PoseGraphNodeCallback(
    const pose_graph_msgs::PoseGraphNode::ConstPtr &msg) {
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);
  keyed_poses_[msg->key] = pose;
  keyed_stamps_.insert(
      std::pair<long unsigned int, ros::Time>(msg->key, msg->header.stamp));
  stamps_keyed_.insert(std::pair<double, long unsigned int>(
      msg->header.stamp.toSec(), msg->key));
}

void PoseGraphVisualizer::PoseGraphEdgeCallback(
    const pose_graph_msgs::PoseGraphEdge::ConstPtr &msg) {
  odometry_edges_.emplace_back(std::make_pair(msg->key_from, msg->key_to));
}

void PoseGraphVisualizer::ErasePosegraphCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  const bool erase_all = msg->data;

  // This gets called at restart and load to initialize everything before
  // loading the graph
  if (erase_all == true) {
    keyed_scans_.clear();
    keyed_stamps_.clear();
    stamps_keyed_.clear();

    loop_edges_.clear();
    odometry_edges_.clear();
    if (publish_interactive_markers_) {
      server.reset(new interactive_markers::InteractiveMarkerServer(
          "interactive_node", "", false));
    }
  }
}

// Callback function to remove the visialization of the edge between factors
void PoseGraphVisualizer::RemoveFactorVizCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  const bool removefactor = msg->data;

  // This gets called after remove factor, to remove the visualization of the
  // edge between the nodes
  if (removefactor == true) {
    loop_edges_.clear();
  }
}

void PoseGraphVisualizer::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr &msg) {
  const long unsigned int key = msg->key;
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_ERROR("%s: Key %u already has a laser scan.", name_.c_str(), key);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg->scan, *scan);

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (key == 0) {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    keyed_stamps_.insert(std::pair<long unsigned int, ros::Time>(key, stamp));
    stamps_keyed_.insert(
        std::pair<double, long unsigned int>(stamp.toSec(), key));
  }

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  keyed_scans_.insert(
      std::pair<long unsigned int, PointCloud::ConstPtr>(key, scan));
}

std::string GenerateKey(long unsigned int key1, long unsigned int key2) {
  return std::to_string(key1) + '|' + std::to_string(key2);
}

gtsam::Key PoseGraphVisualizer::GetKeyAtTime(const ros::Time &stamp) const {
  ROS_INFO("Get pose key closest to input time %f ", stamp.toSec());

  auto iterTime = stamps_keyed_.lower_bound(
      stamp.toSec()); // First key that is not less than timestamp

  // std::cout << "Got iterator at lower_bound. Input: " << stamp.toSec() << ",
  // found " << iterTime->first << std::endl;

  // TODO - interpolate - currently just take one
  double t2 = iterTime->first;
  double t1 = std::prev(iterTime, 1)->first;

  // std::cout << "Time 1 is: " << t1 << ", Time 2 is: " << t2 << std::endl;

  long unsigned int key;

  if (t2 - stamp.toSec() < stamp.toSec() - t1) {
    // t2 is closer - use that key
    // std::cout << "Selecting later time: " << t2 << std::endl;
    key = iterTime->second;
  } else {
    // t1 is closer - use that key
    // std::cout << "Selecting earlier time: " << t1 << std::endl;
    key = std::prev(iterTime, 1)->second;
    iterTime--;
  }
  // std::cout << "Key is: " << key << std::endl;
  if (iterTime == std::prev(stamps_keyed_.begin())) {
    // ROS_WARN("Invalid time for graph (before start of graph range). Choosing
    // next value");
    iterTime++;
    // iterTime = stamps_keyed_.begin();
    key = iterTime->second;
  } else if (iterTime == stamps_keyed_.end()) {
    ROS_WARN(
        "Invalid time for graph (past end of graph range). take latest pose");
  }

  return key;
}

gu::Transform3 PoseGraphVisualizer::GetPoseAtKey(const gtsam::Key &key) const {
  // Get the pose at that key
  if (keyed_poses_.find(key) == keyed_poses_.end()) {
    ROS_WARN("PGV: Key %u does not exist in GetPoseAtKey", key);
    return gu::Transform3();
  }
  const tf::Pose &pose = keyed_poses_.at(key);
  Eigen::Quaterniond quat;
  // TODO convert tf pose's rotation to Eigen
  // tf::quaternionTfToEigen(pose.getRotation(), quat);
  gu::Transform3 result(
      gu::Vec3(pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ()),
      gu::Quat(quat.w(), quat.x(), quat.y(), quat.z()));
  return result;
}

Eigen::Vector3d
PoseGraphVisualizer::GetArtifactPosition(const gtsam::Key artifact_key) const {
  const auto pose = GetPoseAtKey(artifact_key);
  return pose.translation.Eigen();
}

void PoseGraphVisualizer::ArtifactCallback(const core_msgs::Artifact &msg) {
  // Subscribe to artifact messages, include in pose graph, publish global
  // position

  std::cout << "Artifact message received is for id " << msg.id << std::endl;
  std::cout << "\t Parent id: " << msg.parent_id << std::endl;
  std::cout << "\t Confidence: " << msg.confidence << std::endl;
  std::cout << "\t Position:\n[" << msg.point.point.x << ", "
            << msg.point.point.y << ", " << msg.point.point.z << "]"
            << std::endl;
  std::cout << "\t Label: " << msg.label << std::endl;

  // Check for NaNs and reject
  if (std::isnan(msg.point.point.x) || std::isnan(msg.point.point.y) ||
      std::isnan(msg.point.point.z)) {
    ROS_WARN("NAN positions input from artifact message - ignoring");
    return;
  }

  ArtifactInfo artifactinfo;
  artifactinfo.msg = msg;
  ROS_INFO_STREAM("Artifact parent UUID is " << msg.parent_id);
  artifacts_[msg.parent_id] = artifactinfo;

  // // Delay getting graph key until we have it.
  // ROS_INFO_STREAM("Artifact key is " << artifact_id2key_hash_[msg.id]);
  // artifacts_[artifact_id2key_hash_[msg.id]] = artifactinfo;
  // VisualizeArtifacts();
}

bool PoseGraphVisualizer::HighlightEdge(long unsigned int key1,
                                        long unsigned int key2) {
  ROS_INFO("Highlighting factor between %i and %i.", key1, key2);

  if (!KeyExists(key1) || !KeyExists(key2)) {
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

  m.points.push_back(GetPositionMsg(key1, keyed_poses_));
  m.points.push_back(GetPositionMsg(key2, keyed_poses_));
  highlight_pub_.publish(m);

  HighlightNode(key1);
  HighlightNode(key2);

  return true;
}

bool PoseGraphVisualizer::HighlightNode(long unsigned int key) {
  ROS_INFO("Highlighting node %i.", key);

  if (!KeyExists(key)) {
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
  m.pose.position = GetPositionMsg(key, keyed_poses_);
  highlight_pub_.publish(m);

  return true;
}

void PoseGraphVisualizer::UnhighlightEdge(long unsigned int key1,
                                          long unsigned int key2) {
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

void PoseGraphVisualizer::UnhighlightNode(long unsigned int key) {
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
  unsigned char prefix_from = request.prefix_from[0];
  unsigned int key_from = request.key_from;
  gtsam::Symbol id_from (prefix_from,key_from);

  unsigned char prefix_to = request.prefix_to[0]; 
  unsigned int key_to = request.key_to;
  gtsam::Symbol id_to (prefix_to,key_to);

  if (request.highlight) {
    response.success = HighlightEdge(id_from, id_to);
  } else {
    UnhighlightEdge(id_from, id_to);
    response.success = true;
  }
  return true;
}

// Interactive Marker Menu
void PoseGraphVisualizer::MakeMenuMarker(const tf::Pose &position,
                                         const std::string &id_number) {
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

void PoseGraphVisualizer::VisualizePoseGraph() {
  // Publish odometry edges.
  if (odometry_edge_pub_.getNumSubscribers() > 0) {
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

    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      const auto key1 = odometry_edges_[ii].first;
      const auto key2 = odometry_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1, keyed_poses_));
      m.points.push_back(GetPositionMsg(key2, keyed_poses_));
    }
    odometry_edge_pub_.publish(m);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_.getNumSubscribers() > 0) {
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

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
      const auto key1 = loop_edges_[ii].first;
      const auto key2 = loop_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1, keyed_poses_));
      m.points.push_back(GetPositionMsg(key2, keyed_poses_));
    }
    loop_edge_pub_.publish(m);
  }

  // Publish artifact edges.
  if (artifact_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 2;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.2;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.6;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < artifact_edges_.size(); ++ii) {
      const auto key1 = artifact_edges_[ii].first;
      const auto key2 = artifact_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1, keyed_poses_));
      m.points.push_back(GetPositionMsg(key2, keyed_artifact_poses_));
    }
    artifact_edge_pub_.publish(m);
  }

  // Publish UWB edges.
  if (uwb_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 3;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < uwb_edges_.size(); ++ii) {
      const auto key1 = uwb_edges_[ii].first;
      const auto key2 = uwb_edges_[ii].second;

      m.points.push_back(GetPositionMsg(key1, keyed_poses_));
      m.points.push_back(GetPositionMsg(key2, keyed_uwb_poses_));
      ROS_INFO("PGV sends UWB edge %u", ii);
    }
    uwb_edge_pub_.publish(m);
  }

  // Publish nodes in the pose graph.
  if (graph_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 4;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.3;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    for (const auto &keyedPose : keyed_poses_) {
      m.points.push_back(GetPositionMsg(keyedPose.first, keyed_poses_));
    }
    graph_node_pub_.publish(m);
  }

  // Publish node IDs in the pose graph.
  if (graph_node_id_pub_.getNumSubscribers() > 0) {
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
    for (const auto &keyedPose : keyed_poses_) {
      tf::poseTFToMsg(keyedPose.second, m.pose);
      // Display text for the node
      m.text = std::to_string(keyedPose.first);
      m.id = id_base + keyedPose.first;
      graph_node_id_pub_.publish(m);
    }
  }

  // Publish keyframe nodes in the pose graph.
  if (keyframe_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 4;
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
      m.points.push_back(GetPositionMsg(keyedScan.first, keyed_poses_));
    }
    keyframe_node_pub_.publish(m);
  }

  // Draw a sphere around the current sensor frame to show the area in which we
  // are checking for loop closures.
  if (closure_area_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = base_frame_id_;
    m.ns = base_frame_id_;
    m.id = 5;
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

  // Publish UWB nodes.
  if (uwb_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 6;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.4;
    m.scale.x = 0.5;
    m.scale.y = 0.5;
    m.scale.z = 0.5;

    for (const auto &keyedPose : keyed_uwb_poses_) {
      m.points.push_back(GetPositionMsg(keyedPose.first, keyed_uwb_poses_));
    }
    uwb_node_pub_.publish(m);
  }

  // Publish Artifacts
  if (artifact_marker_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = "artifact";
    // ROS_INFO("Publishing artifacts!");
    for (const auto& keyedPose : keyed_artifact_poses_) {
      ROS_INFO_STREAM("Iterator first is: " << keyedPose.first);
      m.header.stamp = ros::Time::now();

      // get the gtsam key
      gtsam::Key key(keyedPose.first);
      m.id = key;

      ROS_INFO_STREAM("Artifact key (raw) is: " << keyedPose.first);
      ROS_INFO_STREAM("Artifact key is " << key);
      ROS_INFO_STREAM("Artifact hash key is "
                      << gtsam::DefaultKeyFormatter(key));
      if (gtsam::Symbol(key).chr() != ('l' || 'm' || 'n' || 'o' || 'p')) {
        ROS_WARN("ERROR - have a non-landmark ID");
        ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(key));
        continue;
      }

      // TODO only publish what has changed
      ROS_INFO_STREAM("Artifact key to publish is "
                      << gtsam::DefaultKeyFormatter(key));

      ROS_INFO_STREAM(
          "ID from key for the artifact is: " << artifact_key2id_hash_[key]);

      // Get the artifact information
      ArtifactInfo art = artifacts_[artifact_key2id_hash_[key]];

      // Populate the artifact marker
      VisualizeSingleArtifact(m, art);

      // Publish
      artifact_marker_pub_.publish(m);
    }
  }

  // Interactive markers.
  if (publish_interactive_markers_) {
    for (const auto &keyed_pose : keyed_poses_) {
      gtsam::Symbol key_id = gtsam::Symbol(keyed_pose.first);
      std::string robot_id = std::string(key_id);
      MakeMenuMarker(keyed_pose.second, robot_id);
    }
    if (server != nullptr) {
      server->applyChanges();
    }
  }
}

void PoseGraphVisualizer::VisualizeSingleArtifact(visualization_msgs::Marker& m,
                                                  const ArtifactInfo& art) {
  // Get class of artifact
  std::string artifact_label = art.msg.label;

  // TODO consider whether we should get the position from the graph
  // artifact_position = GetArtifactPosition(key);
  // geometry_msgs::Point pos = art.msg.point.point;
  // std::cout << "\t Position:\n[" << art.msg.point.point.x << ", "
  //           << art.msg.point.point.y << ", " << art.msg.point.point.z << "]"
  //           << std::endl;
  // pos.x = artifact_position[0];
  // pos.y = artifact_position[1];
  // pos.z = artifact_position[2];
  m.pose.position = art.msg.point.point;

  m.header.frame_id = "world";
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.35f;
  m.scale.y = 0.35f;
  m.scale.z = 0.35f;
  m.color.a = 1.0f;

  if (artifact_label == "backpack") {
    std::cout << "backpack marker" << std::endl;
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.type = visualization_msgs::Marker::CUBE;
  }
  if (artifact_label == "fire extinguisher") {
    std::cout << "fire extinguisher marker" << std::endl;
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.75f;
    m.type = visualization_msgs::Marker::SPHERE;
  }
  if (artifact_label == "drill") {
    std::cout << "drill marker" << std::endl;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.type = visualization_msgs::Marker::CYLINDER;
  }
  if (artifact_label == "survivor") {
    std::cout << "survivor marker" << std::endl;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.scale.x = 1.0f;
    m.scale.y = 1.0f;
    m.scale.z = 1.0f;
    m.type = visualization_msgs::Marker::CYLINDER;
  }
  if (artifact_label == "cellphone") {
    std::cout << "cellphone marker" << std::endl;
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 0.2f;
    m.scale.x = 0.35f;
    m.scale.y = 0.7f;
    m.scale.z = 0.1f;
    m.type = visualization_msgs::Marker::CUBE;
  }
}

void PoseGraphVisualizer::VisualizeArtifacts() {
  // Publish Marker with new position
  visualization_msgs::Marker marker;

  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();

  Eigen::Vector3d artifact_position;
  std::string artifact_label;

  // loop through values
  for (auto it = artifacts_.begin(); it != artifacts_.end(); it++) {
    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "artifact";
    marker.id = artifact_id2key_hash_[it->first];
    marker.action = visualization_msgs::Marker::ADD;

    ROS_INFO_STREAM("Iterator first is: " << it->first);

    gtsam::Key key(artifact_id2key_hash_[it->first]);
    ROS_INFO_STREAM("Artifact hash key is "
                    << gtsam::DefaultKeyFormatter(key));
    if (gtsam::Symbol(key).chr() != ('l' || 'm' || 'n' || 'o' || 'p')) {
      ROS_WARN("ERROR - have a non-landmark ID");
      ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(key));
      continue;
    }

    // TODO only publish what has changed
    ROS_INFO_STREAM("Artifact key to publish is "
                    << gtsam::DefaultKeyFormatter(key));
    // artifact_position = GetArtifactPosition(key);
    artifact_label = it->second.msg.label;

    // geometry_msgs::Point pos = it->second.msg.point.point;
    // pos.x = artifact_position[0];
    // pos.y = artifact_position[1];
    // pos.z = artifact_position[2];
    marker.pose.position = it->second.msg.point.point;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35f;
    marker.scale.y = 0.35f;
    marker.scale.z = 0.35f;
    marker.color.a = 1.0f;

    if (artifact_label == "backpack") {
      std::cout << "backpack marker" << std::endl;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CUBE;
    }
    if (artifact_label == "fire extinguisher") {
      std::cout << "fire extinguisher marker" << std::endl;
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.75f;
      marker.type = visualization_msgs::Marker::SPHERE;
    }
    if (artifact_label == "drill") {
      std::cout << "drill marker" << std::endl;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
    if (artifact_label == "survivor") {
      std::cout << "survivor marker" << std::endl;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.scale.x = 1.0f;
      marker.scale.y = 1.0f;
      marker.scale.z = 1.0f;
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
    // marker.lifetime = ros::Duration();

    artifact_marker_pub_.publish(marker);
  }
}
