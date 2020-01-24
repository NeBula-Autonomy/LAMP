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
        10000,
        &PoseGraphHandler::PoseGraphCallback,
        this);

    // Keyed scans
    keyed_scan_sub = nl.subscribe<pose_graph_msgs::KeyedScan>(
        "/" + robot + "/lamp/keyed_scans",
        10000,
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
  keyed_scan_pub_ = nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);
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
}

void PoseGraphHandler::KeyedScanCallback(const pose_graph_msgs::KeyedScan::ConstPtr& msg) {

  data_.b_has_data = true;
  data_.scans.push_back(msg);

  // Republish from base station
  keyed_scan_pub_.publish(msg);
}