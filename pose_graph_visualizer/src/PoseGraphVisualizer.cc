#include <pose_graph_visualizer/PoseGraphVisualizer.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <lamp_utils/PrefixHandling.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>

#include <fstream>

#include <time.h>

namespace pu = parameter_utils;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

std::string GenerateKey(gtsam::Key key1, gtsam::Key key2) {
  return std::to_string(key1) + '|' + std::to_string(key2);
}

geometry_msgs::Point PoseGraphVisualizer::GetPositionMsg(gtsam::Key key) const {
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

bool PoseGraphVisualizer::Initialize(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& pnh) {
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

bool PoseGraphVisualizer::LoadParameters(const ros::NodeHandle& n) {
  // Load frame ids.
  if (!pu::Get("frame_id/fixed", pose_graph_.fixed_frame_id))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("b_artifacts_in_global", artifacts_in_global_))
    return false;
  if (!pu::Get("b_use_base_reconciliation", b_use_base_reconciliation_))
    return false;
  // Names of all robots for base station to subscribe to
  if (!pu::Get("robot_names", robot_names_)) {
    ROS_ERROR("%s: No robot names provided to base station.", name_.c_str());
    return false;
  } else {
    for (auto s : robot_names_) {
      ROS_INFO_STREAM("Registered new robot: " << s);
    }
  }

  // Load proximity threshold from LaserLoopClosure node
  if (!pu::Get("proximity_threshold", proximity_threshold_))
    return false;

  if (!pu::Get("use_realistic_artifact_models", use_realistic_artifact_models_))
    return false;

  if (!pu::Get("b_scale_artifacts_with_confidence",
               b_scale_artifacts_with_confidence_))
    return false;

  if (!pu::Get("artifact_confidence_limit", artifact_confidence_limit_))
    return false;

  // Initialize interactive marker server
  if (publish_interactive_markers_) {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "interactive_node", "", false));
  }
  return true;
}

bool PoseGraphVisualizer::RegisterCallbacks(const ros::NodeHandle& nh_,
                                            const ros::NodeHandle& pnh_) {
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
  uwb_edge_between_pub_ =
      pnh.advertise<visualization_msgs::Marker>("uwb_edges_between", 10, false);
  uwb_node_pub_ =
      pnh.advertise<visualization_msgs::Marker>("uwb_nodes", 10, false);
  graph_node_pub_ =
      pnh.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
  graph_node_id_pub_ =
      pnh.advertise<visualization_msgs::Marker>("graph_node_ids", 10, false);

  closure_area_pub_ =
      pnh.advertise<visualization_msgs::Marker>("closure_area", 10, false);
  highlight_pub_ =
      pnh.advertise<visualization_msgs::Marker>("confirm_edge", 10, false);
  artifact_marker_pub_ =
      pnh.advertise<visualization_msgs::Marker>("artifact_markers", 10, true);
  stair_marker_pub_ =
      pnh.advertise<visualization_msgs::Marker>("stair_markers", 10, false);
  artifact_id_marker_pub_ = pnh.advertise<visualization_msgs::Marker>(
      "artifact_id_markers", 10, true);

  keyed_scan_sub_ = nh.subscribe<pose_graph_msgs::KeyedScan>(
      "lamp/keyed_scans", 10, &PoseGraphVisualizer::KeyedScanCallback, this);
  pose_graph_sub_ = nh.subscribe<pose_graph_msgs::PoseGraph>(
      "lamp/pose_graph", 10, &PoseGraphVisualizer::PoseGraphCallback, this);
  pose_graph_edge_sub_ = nh.subscribe<pose_graph_msgs::PoseGraphEdge>(
      "lamp/pose_graph_edge",
      10,
      &PoseGraphVisualizer::PoseGraphEdgeCallback,
      this);
  pose_graph_node_sub_ = nh.subscribe<pose_graph_msgs::PoseGraphNode>(
      "lamp/pose_graph_node",
      10,
      &PoseGraphVisualizer::PoseGraphNodeCallback,
      this);
  erase_posegraph_sub_ =
      nh.subscribe<std_msgs::Bool>("lamp/erase_posegraph",
                                   10,
                                   &PoseGraphVisualizer::ErasePosegraphCallback,
                                   this);

  remove_factor_viz_sub_ = nh.subscribe<std_msgs::Bool>(
      "lamp/remove_factor_viz",
      10,
      &PoseGraphVisualizer::RemoveFactorVizCallback,
      this);

  ignore_artifact_sub_ = nh.subscribe<std_msgs::String>(
      "lamp/ignore_artifact",
      10,
      &PoseGraphVisualizer::IgnoreArtifactCallback,
      this);
  revive_artifact_sub_ = nh.subscribe<std_msgs::String>(
      "lamp/revive_artifact",
      10,
      &PoseGraphVisualizer::ReviveArtifactCallback,
      this);

  // Create subscribers for each robot
  if (!b_use_base_reconciliation_) {
    ROS_INFO("Using topics directly from the robot");
    for (std::string robot : robot_names_) {
      // artifact subs
      artifact_sub_ = nh.subscribe("/" + robot + "/artifact",
                                   10,
                                   &PoseGraphVisualizer::ArtifactCallback,
                                   this);

      // Store the subscribers
      subscribers_artifacts_.push_back(artifact_sub_);
    }
  } else {
    ROS_INFO("Using the result from base station reconciliation");
    // Add for case where base station reconciliation is used
    artifact_sub_ = nh.subscribe("reconciled_artifact",
                                 10,
                                 &PoseGraphVisualizer::ArtifactCallback,
                                 this);
    subscribers_artifacts_.push_back(artifact_sub_);
  }

  return true;
}

void PoseGraphVisualizer::PoseGraphCallback(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg) {
  if (!msg->incremental) {
    pose_graph_.Reset();
  }
  // ROS_INFO("PGV: updating pose graph from message");
  pose_graph_.UpdateFromMsg(msg);
  for (const pose_graph_msgs::PoseGraphNode& msg_node : msg->nodes) {
    tf::Pose pose;
    tf::poseMsgToTF(msg_node.pose, pose);

    gtsam::Symbol sym_key(gtsam::Key(msg_node.key));

    // ROS_INFO_STREAM("Symbol key is " << gtsam::DefaultKeyFormatter(sym_key));
    // ROS_INFO_STREAM("Symbol key (directly) is "
    //                 << gtsam::DefaultKeyFormatter(msg_node.key));

    // ROS_INFO_STREAM("Symbol key (int) is " << msg_node.key);

    // Add UUID if an artifact or uwb node
    if (lamp_utils::IsArtifactPrefix(sym_key.chr())) {
      // ROS_INFO_STREAM("Have an artifact node with key "
      //                 << gtsam::DefaultKeyFormatter(sym_key));

      // node.ID = artifacts_[keyed_pose.key].msg.id;
      artifact_id2key_hash_[msg_node.ID] = gtsam::Symbol(msg_node.key);
      artifact_key2id_hash_[gtsam::Symbol(msg_node.key)] = msg_node.ID;
      continue;
    }
  }

  VisualizePoseGraph();
}

void PoseGraphVisualizer::PoseGraphNodeCallback(
    const pose_graph_msgs::PoseGraphNode::ConstPtr& msg) {
  pose_graph_.TrackNode(*msg);
}

void PoseGraphVisualizer::PoseGraphEdgeCallback(
    const pose_graph_msgs::PoseGraphEdge::ConstPtr& msg) {
  pose_graph_.TrackFactor(*msg);
}

void PoseGraphVisualizer::ErasePosegraphCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  const bool erase_all = msg->data;

  // This gets called at restart and load to initialize everything before
  // loading the graph
  if (erase_all == true) {
    pose_graph_.Reset();
    if (publish_interactive_markers_) {
      server.reset(new interactive_markers::InteractiveMarkerServer(
          "interactive_node", "", false));
    }
  }
}

// Callback function to remove the visualization of the edge between factors
void PoseGraphVisualizer::RemoveFactorVizCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  const bool removefactor = msg->data;

  // This gets called after remove factor, to remove the visualization of the
  // edge between the nodes
  if (removefactor == true) {
    // TODO remove all LOOPCLOSE edges from pose_graph_.edges_new_?
    // loop_edges_.clear();
  }
}

void PoseGraphVisualizer::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& msg) {
  const gtsam::Key& key = msg->key;
  if (pose_graph_.HasScan(key)) {
    ROS_ERROR("%s: Key %lu already has a laser scan.", name_.c_str(), key);
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

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  pose_graph_.InsertKeyedScan(key, scan);
}

Eigen::Vector3d
PoseGraphVisualizer::GetArtifactPosition(const gtsam::Key& artifact_key) const {
  const auto pose = pose_graph_.GetPose(artifact_key);
  Eigen::Vector3d v(pose.x(), pose.y(), pose.z());
  return v;
}

void PoseGraphVisualizer::ArtifactCallback(const artifact_msgs::Artifact& msg) {
  // Subscribe to artifact messages, include in pose graph, publish global
  // position
  // ROS_WARN("HAVE ARTIFACT IN PGV");
  // std::cout << "[PoseGraphVisualizer] Artifact message received is for id "
  // << msg.id << std::endl; std::cout << "\t Parent id: " << msg.parent_id <<
  // std::endl; std::cout << "\t Confidence: " << msg.confidence << std::endl;
  // std::cout << "\t Position:\n[" << msg.point.point.x << ", "
  //           << msg.point.point.y << ", " << msg.point.point.z << "]"
  //           << std::endl;
  // std::cout << "\t Label: " << msg.label << std::endl;

  // Ignore if the confidence is too low
  if (msg.confidence < artifact_confidence_limit_) {
    ROS_WARN("Artifact below confidence limit, not showing in rviz");
    return;
  }

  // Check for NaNs and reject
  if (std::isnan(msg.point.point.x) || std::isnan(msg.point.point.y) ||
      std::isnan(msg.point.point.z)) {
    ROS_WARN("NAN positions input from artifact message - ignoring");
    return;
  }

  // Update id to parent ID mapping
  artifact_ID2ParentID_[msg.id] = msg.parent_id;

  ArtifactInfo artifactinfo;
  artifactinfo.msg = msg;

  if (artifacts_.count(msg.parent_id)) {
    // ROS_DEBUG("Have a repeat observation of an existing artifact");
    if (msg.confidence > artifacts_[msg.parent_id].msg.confidence) {
      // ROS_DEBUG_STREAM("Updating artifact visualization, with larger
      // confidence.\n Increasing from " << msg.confidence << " to " <<
      // artifacts_[msg.parent_id].msg.confidence);
      artifacts_[msg.parent_id] = artifactinfo;

      // Update current parent id to id mapping
      current_parentID2ID_[msg.parent_id] = msg.id;
    } else {
      // ROS_INFO_STREAM("Keeping old artifact with confidence " <<
      // artifacts_[msg.parent_id].msg.confidence << ", which is more than new
      // confidence: " << msg.confidence);
    }
  } else {
    // ROS_INFO("New artifact");
    artifacts_[msg.parent_id] = artifactinfo;
    current_parentID2ID_[msg.parent_id] = msg.id;
  }

  // ROS_INFO_STREAM("Artifact parent UUID is " << msg.parent_id);

  // // Delay getting graph key until we have it.
  // ROS_INFO_STREAM("Artifact key is " << artifact_id2key_hash_[msg.id]);
  // artifacts_[artifact_id2key_hash_[msg.id]] = artifactinfo;
  // VisualizeArtifacts();
}

void PoseGraphVisualizer::IgnoreArtifactCallback(
    const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Remove artifact %s from markers", msg->data.c_str());
  if (!IsArtifactBlacklisted(msg->data)) {
    artifact_parentID_blacklist_.push_back(msg->data);
  }
}

void PoseGraphVisualizer::ReviveArtifactCallback(
    const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Revert artifact %s to markers", msg->data.c_str());
  artifact_parentID_blacklist_.erase(
      std::remove(artifact_parentID_blacklist_.begin(),
                  artifact_parentID_blacklist_.end(),
                  msg->data),
      artifact_parentID_blacklist_.end());
}

bool PoseGraphVisualizer::HighlightEdge(gtsam::Key key1, gtsam::Key key2) {
  ROS_INFO("Highlighting factor between %lu and %lu.", key1, key2);

  if (!pose_graph_.HasKey(key1) || !pose_graph_.HasKey(key2)) {
    ROS_WARN("Key %lu or %lu does not exist.", key1, key2);
    return false;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = pose_graph_.fixed_frame_id;
  m.ns = pose_graph_.fixed_frame_id + "edge" + GenerateKey(key1, key2);
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

bool PoseGraphVisualizer::HighlightNode(gtsam::Key key) {
  ROS_INFO("Highlighting node %lu.", key);

  if (!pose_graph_.HasKey(key)) {
    ROS_WARN("Key %lu does not exist.", key);
    return false;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = pose_graph_.fixed_frame_id;
  m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
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

void PoseGraphVisualizer::UnhighlightEdge(gtsam::Key key1, gtsam::Key key2) {
  visualization_msgs::Marker m;
  m.header.frame_id = pose_graph_.fixed_frame_id;
  m.ns = pose_graph_.fixed_frame_id + "edge" + GenerateKey(key1, key2);
  m.id = 0;
  if (key1 == key2 && key1 == 0)
    m.action = visualization_msgs::Marker::DELETEALL;
  else
    m.action = visualization_msgs::Marker::DELETE;
  highlight_pub_.publish(m);

  UnhighlightNode(key1);
  UnhighlightNode(key2);
}

void PoseGraphVisualizer::UnhighlightNode(gtsam::Key key) {
  visualization_msgs::Marker m;
  m.header.frame_id = pose_graph_.fixed_frame_id;
  m.ns = pose_graph_.fixed_frame_id + "node" + std::to_string(key);
  m.id = 0;
  if (key == 0)
    m.action = visualization_msgs::Marker::DELETEALL;
  else
    m.action = visualization_msgs::Marker::DELETE;
  highlight_pub_.publish(m);
}

bool PoseGraphVisualizer::HighlightNodeService(
    pose_graph_visualizer::HighlightNodeRequest& request,
    pose_graph_visualizer::HighlightNodeResponse& response) {
  if (request.highlight) {
    response.success = HighlightNode(request.key);
  } else {
    UnhighlightNode(request.key);
    response.success = true;
  }
  return true;
}

bool PoseGraphVisualizer::HighlightEdgeService(
    pose_graph_visualizer::HighlightEdgeRequest& request,
    pose_graph_visualizer::HighlightEdgeResponse& response) {
  unsigned char prefix_from = request.prefix_from[0];
  unsigned int key_from = request.key_from;
  gtsam::Symbol id_from(prefix_from, key_from);

  unsigned char prefix_to = request.prefix_to[0];
  unsigned int key_to = request.key_to;
  gtsam::Symbol id_to(prefix_to, key_to);

  if (request.highlight) {
    response.success = HighlightEdge(id_from, id_to);
  } else {
    UnhighlightEdge(id_from, id_to);
    response.success = true;
  }
  return true;
}

// Interactive Marker Menu
void PoseGraphVisualizer::MakeMenuMarker(const gtsam::Pose3& pose,
                                         const std::string& id_number) {
  interactive_markers::MenuHandler menu_handler;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = PoseGraphVisualizer::pose_graph_.fixed_frame_id;
  int_marker.scale = 1.0;
  int_marker.pose = lamp_utils::GtsamToRosMsg(pose);
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
  visualization_msgs::Marker odometry_edges;
  visualization_msgs::Marker loop_edges;
  visualization_msgs::Marker artifact_edges;
  visualization_msgs::Marker uwb_edges;
  visualization_msgs::Marker uwb_between_edges;

  visualization_msgs::Marker* edge_target;
  // TODO visualize new edges/nodes or all?
  for (const auto& edge : pose_graph_.GetNewEdges()) {
    edge_target = nullptr;
    switch (edge.type) {
    case pose_graph_msgs::PoseGraphEdge::ODOM:
      edge_target = &odometry_edges;
      break;
    case pose_graph_msgs::PoseGraphEdge::LOOPCLOSE:
      edge_target = &loop_edges;
      break;
    case pose_graph_msgs::PoseGraphEdge::ARTIFACT:
      edge_target = &artifact_edges;
      break;
    case pose_graph_msgs::PoseGraphEdge::UWB_RANGE:
      edge_target = &uwb_edges;
      break;
    case pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN:
      edge_target = &uwb_between_edges;
      break;
    }
    // TODO - look at how to handle Priors - a small edge - 1 m down in z
    if (edge_target) {
      edge_target->points.push_back(GetPositionMsg(edge.key_from));
      edge_target->points.push_back(GetPositionMsg(edge.key_to));
    }
  }

  // Publish odometry edges.
  if (odometry_edge_pub_.getNumSubscribers() > 0) {
    odometry_edges.header.frame_id = pose_graph_.fixed_frame_id;
    odometry_edges.ns = pose_graph_.fixed_frame_id;
    odometry_edges.id = 0;
    odometry_edges.action = visualization_msgs::Marker::ADD;
    odometry_edges.type = visualization_msgs::Marker::LINE_LIST;
    odometry_edges.color.r = 1.0;
    odometry_edges.color.g = 0.0;
    odometry_edges.color.b = 0.0;
    odometry_edges.color.a = 0.8;
    odometry_edges.scale.x = 0.02;
    odometry_edge_pub_.publish(odometry_edges);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_.getNumSubscribers() > 0) {
    loop_edges.header.frame_id = pose_graph_.fixed_frame_id;
    loop_edges.ns = pose_graph_.fixed_frame_id;
    loop_edges.id = 1;
    loop_edges.action = visualization_msgs::Marker::ADD;
    loop_edges.type = visualization_msgs::Marker::LINE_LIST;
    loop_edges.color.r = 0.0;
    loop_edges.color.g = 0.2;
    loop_edges.color.b = 1.0;
    loop_edges.color.a = 0.8;
    loop_edges.scale.x = 0.02;
    loop_edge_pub_.publish(loop_edges);
  }

  // Publish artifact edges.
  if (artifact_edge_pub_.getNumSubscribers() > 0) {
    artifact_edges.header.frame_id = pose_graph_.fixed_frame_id;
    artifact_edges.ns = pose_graph_.fixed_frame_id;
    artifact_edges.id = 2;
    artifact_edges.action = visualization_msgs::Marker::ADD;
    artifact_edges.type = visualization_msgs::Marker::LINE_LIST;
    artifact_edges.color.r = 0.2;
    artifact_edges.color.g = 1.0;
    artifact_edges.color.b = 0.0;
    artifact_edges.color.a = 0.6;
    artifact_edges.scale.x = 0.02;
    artifact_edge_pub_.publish(artifact_edges);
  }

  // Publish UWB edges.
  if (uwb_edge_pub_.getNumSubscribers() > 0) {
    // visualization_msgs::Marker m;
    uwb_edges.header.frame_id = pose_graph_.fixed_frame_id;
    uwb_edges.ns = pose_graph_.fixed_frame_id;
    uwb_edges.id = 3;
    uwb_edges.action = visualization_msgs::Marker::ADD;
    uwb_edges.type = visualization_msgs::Marker::LINE_LIST;
    uwb_edges.color.r = 0.0;
    uwb_edges.color.g = 1.0;
    uwb_edges.color.b = 0.0;
    uwb_edges.color.a = 0.8;
    uwb_edges.scale.x = 0.02;
    uwb_edge_pub_.publish(uwb_edges);
  }

  // Publish UWB between edges.
  if (uwb_edge_between_pub_.getNumSubscribers() > 0) {
    // visualization_msgs::Marker m;
    uwb_between_edges.header.frame_id = pose_graph_.fixed_frame_id;
    uwb_between_edges.ns = pose_graph_.fixed_frame_id;
    uwb_between_edges.id = 4;
    uwb_between_edges.action = visualization_msgs::Marker::ADD;
    uwb_between_edges.type = visualization_msgs::Marker::LINE_LIST;
    uwb_between_edges.color.r = 0.0;
    uwb_between_edges.color.g = 1.0;
    uwb_between_edges.color.b = 0.0;
    uwb_between_edges.color.a = 0.8;
    uwb_between_edges.scale.x = 0.04;
    uwb_edge_between_pub_.publish(uwb_between_edges);
  }

  visualization_msgs::Marker generic_nodes;
  visualization_msgs::Marker uwb_nodes;

  // Store keys for generic (non-UWB, non-artifact) nodes for text markers
  std::set<gtsam::Symbol> pose_keys;

  for (const auto& node : pose_graph_.GetNewNodes()) {
    tf::Pose pose;
    tf::poseMsgToTF(node.pose, pose);

    gtsam::Symbol sym_key(gtsam::Key(node.key));

    if (sym_key.chr() == 'u') {
      // UWB
      uwb_nodes.points.push_back(GetPositionMsg(node.key));
      continue;
    }

    if (lamp_utils::IsArtifactPrefix(sym_key.chr())) {
      // handle artifacts separately (below)
      continue;
    }

    // Fill pose nodes (representing the robot position)
    generic_nodes.points.push_back(GetPositionMsg(node.key));
    pose_keys.insert(sym_key);
  }

  // Publish nodes in the pose graph.
  if (graph_node_pub_.getNumSubscribers() > 0) {
    generic_nodes.header.frame_id = pose_graph_.fixed_frame_id;
    generic_nodes.ns = pose_graph_.fixed_frame_id;
    generic_nodes.id = 4;
    generic_nodes.action = visualization_msgs::Marker::ADD;
    generic_nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    generic_nodes.color.r = 0.3;
    generic_nodes.color.g = 0.0;
    generic_nodes.color.b = 1.0;
    generic_nodes.color.a = 0.8;
    generic_nodes.scale.x = 0.1;
    generic_nodes.scale.y = 0.1;
    generic_nodes.scale.z = 0.1;
    graph_node_pub_.publish(generic_nodes);
  }

  // Publish UWB nodes.
  if (uwb_node_pub_.getNumSubscribers() > 0) {
    uwb_nodes.header.frame_id = pose_graph_.fixed_frame_id;
    uwb_nodes.ns = pose_graph_.fixed_frame_id;
    uwb_nodes.id = 6;
    uwb_nodes.action = visualization_msgs::Marker::ADD;
    uwb_nodes.type = visualization_msgs::Marker::CUBE_LIST;
    uwb_nodes.color.r = 0.0;
    uwb_nodes.color.g = 1.0;
    uwb_nodes.color.b = 0.0;
    uwb_nodes.color.a = 0.4;
    uwb_nodes.scale.x = 0.5;
    uwb_nodes.scale.y = 0.5;
    uwb_nodes.scale.z = 0.5;
    uwb_node_pub_.publish(uwb_nodes);
  }

  // Publish text markers for node IDs in the pose graph.
  if (graph_node_id_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = pose_graph_.fixed_frame_id;
    m.ns = pose_graph_.fixed_frame_id;

    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    m.scale.z = 0.02; // Only Scale z is used - height of capital A in the text

    int id_base = 100;
    int counter = 0;
    for (const auto& key : pose_keys) {
      m.pose = lamp_utils::GtsamToRosMsg(pose_graph_.GetPose(key));
      // Display text for the node
      m.text = std::to_string(key);
      m.id = id_base + key;
      graph_node_id_pub_.publish(m);
    }
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

  // Publish Artifacts
  if (artifact_marker_pub_.getNumSubscribers() > 0 ||
      artifact_id_marker_pub_.getNumSubscribers() > 0 ||
      stair_marker_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m, m_id;
    m.header.frame_id = pose_graph_.fixed_frame_id;
    m_id.header.frame_id = m.header.frame_id;
    m.ns = "artifact";
    m_id.ns = "artifact_id";

    // ROS_INFO("Publishing artifacts!");
    // TODO - use the pose from the pose-graph for the artifacts
    for (const auto& entry : artifact_key2id_hash_) {
      m.header.stamp = ros::Time::now();
      m_id.header.stamp = ros::Time::now();

      // get the gtsam key
      gtsam::Key key(entry.first);

      // ROS_INFO_STREAM("[Publishing Artifact] Artifact key (raw) is: " <<
      // entry.second); ROS_INFO_STREAM("Artifact key is " << key);
      // ROS_INFO_STREAM("Artifact hash key is "
      //                 << gtsam::DefaultKeyFormatter(key));
      if (!lamp_utils::IsArtifactPrefix(gtsam::Symbol(key).chr())) {
        ROS_WARN("ERROR - have a non-landmark ID");
        ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(key));
        continue;
      }

      // Check that we have recieved the artifact
      if (artifact_ID2ParentID_.count(entry.second) == 0) {
        ROS_DEBUG_STREAM("Have not recevied artifact message for artifact "
                         << gtsam::DefaultKeyFormatter(key)
                         << " that is in the graph yet. Have ID: "
                         << entry.second);
        continue;
      }

      // Get the parent ID
      std::string parent_id = artifact_ID2ParentID_[entry.second];

      // Continue only if this ID is what is linked to an ative parent ID
      if (entry.second != current_parentID2ID_[parent_id]) {
        ROS_DEBUG_STREAM("Artifact with ID: "
                         << entry.second
                         << " is not the active artifact for parent id: "
                         << parent_id);
        continue;
      }

      // TODO only publish what has changed
      // ROS_INFO_STREAM("Artifact key to publish is "
      //                 << gtsam::DefaultKeyFormatter(key));

      // ROS_INFO_STREAM("ID from key for the artifact is: " << entry.second <<
      // ", with parent id " << parent_id);

      // Get the artifact information
      if (artifacts_.find(parent_id) == artifacts_.end()) {
        ROS_DEBUG_STREAM(
            "No artifact info for artifact that is in the graph, key: "
            << gtsam::DefaultKeyFormatter(key));
        continue;
      }

      int id_marker = parent_id2int_(parent_id);

      m.id = id_marker;
      m_id.id = id_marker;

      ArtifactInfo art = artifacts_[parent_id];

      // Update the artifact position from the graph
      art.msg.point.point = GetPositionMsg(gtsam::Symbol(key));
      art.msg.point.header.stamp = ros::Time::now();

      if (art.msg.confidence < 60.0) {
        confidence_scale_ = 0.90;
      } else {
        // Scale by confidence
        confidence_scale_ =
            0.90 + (art.msg.confidence - 60.0) * (2.0 - 0.9) / 40.0;
      }

      // Populate the artifact marker
      if (use_realistic_artifact_models_) {
        VisualizeSingleRealisticArtifact(m, art);
      } else {
        VisualizeSingleSimpleArtifact(m, art);
      }

      VisualizeSingleArtifactId(m_id, art);

      // Add or delete markers depending on user rejection status
      if (IsArtifactBlacklisted(art.msg.parent_id)) {
        m.action = visualization_msgs::Marker::DELETE;
        m_id.action = visualization_msgs::Marker::DELETE;
      } else {
        m.action = visualization_msgs::Marker::ADD;
        m_id.action = visualization_msgs::Marker::ADD;
      }

      // Publish
      if (art.msg.label == "Negative Obstacle") {
        stair_marker_pub_.publish(m);
      } else {
        artifact_marker_pub_.publish(m);
        artifact_id_marker_pub_.publish(m_id);
      }
    }
  }

  // Interactive markers.
  if (publish_interactive_markers_) {
    for (const auto& keyed_pose :
         pose_graph_.GetNewValues()) { // TODO - just generic nodes
      gtsam::Symbol key_id = gtsam::Symbol(keyed_pose.key);
      std::string robot_id = std::string(key_id);
      MakeMenuMarker(pose_graph_.GetPose(keyed_pose.key), robot_id);
    }
    if (server != nullptr) {
      server->applyChanges();
    }
  }
}

void PoseGraphVisualizer::VisualizeSingleArtifactId(
    visualization_msgs::Marker& m, const ArtifactInfo& art) {
  std::string id = art.msg.id;
  m.pose.position = art.msg.point.point;
  m.pose.position.z = m.pose.position.z + 1.5;
  m.pose.position.x = m.pose.position.x + 2.5;
  m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  if (id.find("PhoneArtifact") != std::string::npos) {
    m.text = id;
  } else {
    m.text = id.substr(0, 8);
  }
  m.header.frame_id = "world";
  std::string artifact_label = art.msg.label;

  if (artifact_label == "Backpack") {
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
  } else if (artifact_label == "Fire Extinguisher") {
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.75f;
  } else if (artifact_label == "Drill") {
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
  } else if (artifact_label == "Survivor") {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
  } else if (artifact_label == "Cell Phone") {
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 0.7f;
  } else if (artifact_label == "Gas") {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
  } else if (artifact_label == "Vent") {
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
  } else {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
  }

  m.color.a = 1.0f;
  m.scale.z = 2.5f;
  return;
}

void PoseGraphVisualizer::VisualizeSingleRealisticArtifact(
    visualization_msgs::Marker& m, const ArtifactInfo& art) {
  // Get class of artifact
  std::string artifact_label = art.msg.label;

  m.pose.position = art.msg.point.point;

  m.header.frame_id = "world";
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  float scale = 0.95;
  if (b_scale_artifacts_with_confidence_) {
    scale = confidence_scale_;
  }
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;

  // Please note: if color is set to anything other than 0,0,0,0 for
  // the meshes, the meshes will ignore their texture materials and
  // be tinted with the assigned color and alpha
  m.mesh_use_embedded_materials = true;
  m.color.r = 0.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.color.a = 0.0f;
  m.type = visualization_msgs::Marker::MESH_RESOURCE;

  if (artifact_label == "Backpack") {
    m.mesh_resource =
        "package://pose_graph_visualizer/meshes/backpack/backpack.dae";
  } else if (artifact_label == "Fire Extinguisher") {
    m.mesh_resource =
        "package://pose_graph_visualizer/meshes/extinguisher/extinguisher.dae";
  } else if (artifact_label == "Drill") {
    m.mesh_resource = "package://pose_graph_visualizer/meshes/drill/drill.dae";
  } else if (artifact_label == "Survivor") {
    m.mesh_resource =
        "package://pose_graph_visualizer/meshes/survivor/survivor.dae";
  } else if (artifact_label == "Cell Phone") {
    m.mesh_resource = "package://pose_graph_visualizer/meshes/phone/phone.dae";
  } else if (artifact_label == "Gas") {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m.type = visualization_msgs::Marker::SPHERE;
    m.mesh_resource = "package://pose_graph_visualizer/meshes/gas.dae";
  } else if (artifact_label == "Negative Obstacle") {
    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m.type = visualization_msgs::Marker::CUBE;
  } else if (artifact_label == "Vent") {
    m.mesh_resource = "package://pose_graph_visualizer/meshes/vent/vent.dae";
  } else {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.scale.x = 1.0f;
    m.scale.y = 1.0f;
    m.scale.z = 1.0f;
    m.type = visualization_msgs::Marker::CUBE;
  }
}

void PoseGraphVisualizer::VisualizeSingleSimpleArtifact(
    visualization_msgs::Marker& m, const ArtifactInfo& art) {
  // Get class of artifact
  std::string artifact_label = art.msg.label;

  // ROS_INFO_STREAM("Artifact label for visualization is: " << artifact_label);

  m.pose.position = art.msg.point.point;

  m.header.frame_id = "world";
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;
  float scale = 0.95;
  if (b_scale_artifacts_with_confidence_) {
    scale = confidence_scale_;
  }
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.a = 1.0f;

  if (artifact_label == "Backpack") {
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.type = visualization_msgs::Marker::CUBE;
  } else if (artifact_label == "Fire Extinguisher") {
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.75f;
    m.type = visualization_msgs::Marker::SPHERE;
  } else if (artifact_label == "Drill") {
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.type = visualization_msgs::Marker::CYLINDER;
  } else if (artifact_label == "Survivor") {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.scale.x = 1.2f;
    m.scale.y = 1.2f;
    m.scale.z = 1.2f;
    m.type = visualization_msgs::Marker::CYLINDER;
  } else if (artifact_label == "Cell Phone") {
    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 0.7f;
    m.scale.x = 0.55f;
    m.scale.y = 1.2f;
    m.scale.z = 0.3f;
    m.type = visualization_msgs::Marker::CUBE;
  } else if (artifact_label == "Gas") {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.type = visualization_msgs::Marker::SPHERE;
  } else if (artifact_label == "Negative Obstacle") {
    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m.type = visualization_msgs::Marker::CUBE;
  } else if (artifact_label == "Vent") {
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.type = visualization_msgs::Marker::SPHERE;
  } else {
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.scale.x = 1.0f;
    m.scale.y = 1.0f;
    m.scale.z = 1.0f;
    m.type = visualization_msgs::Marker::CUBE;
  }

  //   std::cout << "Fiducial marker" << std::endl;
  // m.color.r = 1.0f;
  // m.color.g = 1.0f;
  // m.color.b = 1.0f;
  // m.scale.x = 0.15f;
  // m.scale.y = 0.7f;
  // m.scale.z = 0.7f;
  // m.type = visualization_msgs::Marker::CUBE;
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

    // ROS_INFO_STREAM("Iterator first is: " << it->first);

    gtsam::Key key(artifact_id2key_hash_[it->first]);
    // ROS_INFO_STREAM("Artifact hash key is " <<
    // gtsam::DefaultKeyFormatter(key));
    if (gtsam::Symbol(key).chr() != 'A' && gtsam::Symbol(key).chr() != 'B' &&
        gtsam::Symbol(key).chr() != 'C' && gtsam::Symbol(key).chr() != 'D' &&
        gtsam::Symbol(key).chr() != 'E' && gtsam::Symbol(key).chr() != 'F' &&
        gtsam::Symbol(key).chr() != 'G' && gtsam::Symbol(key).chr() != 'H' &&
        gtsam::Symbol(key).chr() != 'I' && gtsam::Symbol(key).chr() != 'J' &&
        gtsam::Symbol(key).chr() != 'K' && gtsam::Symbol(key).chr() != 'L' &&
        gtsam::Symbol(key).chr() != 'M' && gtsam::Symbol(key).chr() != 'X') {
      ROS_WARN("ERROR - have a non-landmark ID");
      ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(key));
      continue;
    }

    // TODO only publish what has changed
    // ROS_INFO_STREAM("Artifact key to publish is "
    // << gtsam::DefaultKeyFormatter(key));
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

    if (artifact_label == "Backpack") {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CUBE;
    }
    if (artifact_label == "Fire Extinguisher") {
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.75f;
      marker.type = visualization_msgs::Marker::SPHERE;
    }
    if (artifact_label == "Drill") {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
    if (artifact_label == "Survivor") {
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
