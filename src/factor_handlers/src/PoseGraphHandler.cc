/*
 * Copyright Notes
 *
 * Authors: Alex Stephens       (alex.stephens@jpl.nasa.gov)
 */

// Includes
#include <factor_handlers/PoseGraphHandler.h>

PoseGraphHandler::PoseGraphHandler() { }

PoseGraphHandler::~PoseGraphHandler() { }

bool PoseGraphHandler::Initialize(const ros::NodeHandle& n, std::vector<std::string> robot_names) {
  name_ = ros::names::append(n.getNamespace(), "PoseGraphHandler");

  // Robots to subscribe to
  for (std::string robot : robot_names) {
    AddRobot(robot);
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }

  return true;
}

bool PoseGraphHandler::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("LoadParameters method called in PoseGraphHandler");

  return true;
}

bool PoseGraphHandler::AddRobot(std::string robot) {
  
  if (robot_names_.find(robot) != robot_names_.end()) {
    ROS_WARN_STREAM("Robot " << robot << "already registered.");
    return false;
  }

  robot_names_.insert(robot);

  return true;
}

bool PoseGraphHandler::RegisterCallbacks(const ros::NodeHandle& n) {

  ROS_INFO("%s: Registering callbacks in PoseGraphHandler",
           name_.c_str());

  ros::NodeHandle nl(n);

  ros::Subscriber pose_graph_sub;
  ros::Subscriber keyed_scan_sub;

  // Create subscribers for each robot
  for (std::string robot : robot_names_) {

    // Pose graph
    pose_graph_sub = nl.subscribe<pose_graph_msgs::PoseGraph>(
        "/" + robot + "/lamp/pose_graph_incremental",
        100000,
        &PoseGraphHandler::PoseGraphCallback,
        this);

    // Keyed scans
    keyed_scan_sub = nl.subscribe<pose_graph_msgs::KeyedScan>(
        "/" + robot + "/lamp/keyed_scans",
        100000,
        &PoseGraphHandler::KeyedScanCallback,
        this);

    // Store the subscribers
    subscribers_posegraph.push_back(pose_graph_sub);
    subscribers_keyedscan.push_back(keyed_scan_sub);
  }

  return true;
}

bool PoseGraphHandler::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Keyed scans are republished at the base station as soon as they are received
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 1000000, false);
  return true;
}

std::shared_ptr<FactorData> PoseGraphHandler::GetData() {

  // Main interface with lamp for getting new pose graphs
  std::shared_ptr<PoseGraphData> output_data = std::make_shared<PoseGraphData>(data_);

  // Clear the stored data
  ResetGraphData();

  return output_data;
}

void PoseGraphHandler::ResetGraphData() {
  data_.b_has_data = false; 
  data_.type = "posegraph";
  data_.graphs.clear();
  data_.scans.clear();
}

void PoseGraphHandler::PoseGraphCallback(const pose_graph_msgs::PoseGraph::ConstPtr& msg) {

  data_.b_has_data = true;
  data_.graphs.push_back(msg);

  std::unordered_set<uint64_t> repeated_keys;
  for (const auto& node : msg->nodes){
      if (node.ID == "odom_node") {
          //check we only increase by one if it's on the same robot
          gtsam::Symbol node_symbol(node.key);
          if (last_odom_node_key_from_robot_.count(node_symbol.chr()) > 0){
              uint64_t cur_key = node.key;
              uint64_t last_key = last_odom_node_key_from_robot_[node_symbol.chr()];
              if (cur_key != last_key) { // this gets handled by the repeated key
                  if (cur_key < last_key) {
                      ROS_WARN_STREAM(
                              "PoseGraphHandler: Pose graph nodes arriving out of order for " << node_symbol.chr()
                                                                                              << ". Last key "
                                                                                              << last_key
                                                                                              << ", Current Key: "
                                                                                              << cur_key);
                  }
                  if (cur_key != (last_key + 1)) {
                      ROS_WARN_STREAM("PoseGraphHandler: Pose graph current key does not increase by 1 for "
                                              << node_symbol.chr() << ". Last Key " << last_key << ", Current Key:"
                                              << cur_key);
                  }
              }
          }
          last_odom_node_key_from_robot_[node_symbol.chr()] = node_symbol.key();
      }
      //check that the key hasn't already been sent
      if (pose_graph_node_keys_.count(node.key) > 0) {
          repeated_keys.insert(node.key);
      } else {
          pose_graph_node_keys_.insert(node.key);

      }
  }
  if (!repeated_keys.empty()) {
      std::stringstream ss;

      ss << repeated_keys.size() << " duplicate keys, the keys are ";
      for (const auto& key : repeated_keys){
          ss << key << " ";
      }
      ROS_WARN_STREAM("PoseGraphHandler:" << ss.str());
  }
}

void PoseGraphHandler::KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& msg) {

  data_.b_has_data = true;
  data_.scans.push_back(msg);

  // Republish from base station
  keyed_scan_pub_.publish(msg);
  if (keyed_scans_keys_.count(msg->key) > 0){
      ROS_INFO_STREAM("PoseGraphHandler: Repeated keyed Scan for key " << msg->key);
  } else {
      keyed_scans_keys_.insert(msg->key);

      gtsam::Symbol node_symbol(msg->key);
      if (last_keyed_scan_key_from_robot_.count(node_symbol.chr()) > 0) {
          uint64_t cur_key = node_symbol.key();
          uint64_t last_key = last_keyed_scan_key_from_robot_[node_symbol.chr()];
          if (cur_key < last_key) {
              ROS_WARN_STREAM(
                      "PoseGraphHandler: Keyed scans arriving out of order for " << node_symbol.chr() << ". Last key "
                                                                                 << last_key << ", Current Key: "
                                                                                 << cur_key);
          }
          if (cur_key != (last_key + 1)) {
              ROS_WARN_STREAM("PoseGraphHandler: Keyed scan key does not increase by 1 for " << node_symbol.chr()
                                                                                             << ". Last Key "
                                                                                             << last_key
                                                                                             << ", Current Key:"
                                                                                             << cur_key);
          }
      }

      last_keyed_scan_key_from_robot_[node_symbol.chr()] = node_symbol.key();
  }
}