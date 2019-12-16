/*
 * Copyright Notes
 *
 * Authors: Alex Stephens       (alex.stephens@jpl.nasa.gov)
 */

// Includes
#include <factor_handlers/RobotPoseHandler.h>

RobotPoseHandler::RobotPoseHandler() { }

RobotPoseHandler::~RobotPoseHandler() { }

bool RobotPoseHandler::Initialize(const ros::NodeHandle& n, std::vector<std::string> robot_names) {
  name_ = ros::names::append(n.getNamespace(), "RobotPoseHandler");

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

bool RobotPoseHandler::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("LoadParameters method called in RobotPoseHandler");

  return true;
}

bool RobotPoseHandler::AddRobot(std::string robot) {
  
  if (robot_names_.find(robot) != robot_names_.end()) {
    ROS_WARN_STREAM("Robot " << robot << "already registered.");
    return false;
  }

  robot_names_.insert(robot);

  return true;
}

bool RobotPoseHandler::RegisterCallbacks(const ros::NodeHandle& n) {

  ROS_INFO("%s: Registering callbacks in RobotPoseHandler",
           name_.c_str());

  ros::NodeHandle nl(n);

  ros::Subscriber pose_sub_;

  // Create subscribers for each robot
  for (std::string robot : robot_names_) {

    pose_sub_ = nl.subscribe<geometry_msgs::PoseStamped>(
        "/" + robot + "/lamp/lamp_pose",
        10,
        boost::bind(&RobotPoseHandler::PoseCallback, this, _1, robot));
        // &RobotPoseHandler::PoseCallback, 
        // this);

    // Store the subscribers
    subscribers_pose_.push_back(pose_sub_);
  }

  return true;
}

bool RobotPoseHandler::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);
  return true;
}

void RobotPoseHandler::ResetPoseData() {
  data_.b_has_data = false;
  data_.poses.clear();
  data_.type = "pose";
}

std::shared_ptr<FactorData> RobotPoseHandler::GetData() {

  // Main interface with lamp for getting new pose graphs
  std::shared_ptr<RobotPoseData> output_data = std::make_shared<RobotPoseData>(data_);
  ResetPoseData();

  return output_data;
}

void RobotPoseHandler::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, std::string robot) {
  ROS_INFO_STREAM("In RobotPoseHandler::PoseCallback");
// void RobotPoseHandler::PoseCallback(const geometry_msgs::PoseStamped& msg, std::string robot) {

  // ROS_INFO_STREAM("Received pose from robot: " << robot);

  PoseData new_data;
  new_data.stamp = msg->header.stamp;
  new_data.pose = utils::ToGtsam(msg->pose);

  // Overwrite previous data from this robot with the newest entry
  data_.poses[robot] = new_data;
  data_.b_has_data = true;
}