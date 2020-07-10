/*
 * Copyright Notes
 * Authors: Yun Chang      (yunchang@mit.edu)
 */

#include <factor_handlers/StationaryHandler.h>

namespace pu = parameter_utils;

// Constructor and Destructor
// -------------------------------------------------------------

StationaryHandler::StationaryHandler() : currently_stationary_(true) {
  ROS_INFO("StationaryHandler Class Constructor");
}

StationaryHandler::~StationaryHandler() {
  ROS_INFO("StationaryHandler Class Destructor");
}

// Initialization
// -------------------------------------------------------------------------

bool StationaryHandler::Initialize(const ros::NodeHandle& n) {
  ROS_INFO("StationaryHandler - Initialize");
  name_ = ros::names::append(n.getNamespace(), "StationaryHandler");
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

bool StationaryHandler::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("StationaryHandler - LoadParameters");
  if (!pu::Get("stationary_noise_sigma", noise_sigma_)) return false;
  return true;
}

bool StationaryHandler::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks in StationaryHandler",
           name_.c_str());
  ros::NodeHandle nl(n);
  stationary_sub_ = nl.subscribe(
      "stationary_topic", 1000, &StationaryHandler::StationaryCallback, this);
  return true;
}

// Callback
// -------------------------------------------------------------------------------

void StationaryHandler::StationaryCallback(
    const StationaryMessage::ConstPtr& msg) {
  if (msg->status == 0) {
    if (!currently_stationary_) {
      ROS_INFO("Robot stopped. Creating attitude factor...");
      // We want to place the stationary factors when the robot stops
      last_detection_ =
          StationaryData(msg->header.stamp, msg->average_acceleration);
      has_data_ = true;
    }
    currently_stationary_ = true;
  } else {
    currently_stationary_ = false;
  }
}

// LAMP Interface
// -------------------------------------------------------------------------
std::shared_ptr<FactorData> StationaryHandler::GetData() {
  std::shared_ptr<ImuData> factors_output = std::make_shared<ImuData>();
  factors_output->b_has_data = false;

  if (has_data_) {
    ROS_INFO("New attitude factor in StationaryHandler.");
    ImuFactor new_factor(CreateAttitudeFactor(last_detection_.second));
    factors_output->b_has_data = true;
    factors_output->type = "imu";
    factors_output->factors.push_back(new_factor);
  } else {
    factors_output->b_has_data = false;
  }
  has_data_ = false;
  return factors_output;
}

gtsam::Pose3AttitudeFactor StationaryHandler::CreateAttitudeFactor(
    const geometry_msgs::Vector3& gravity_vec) const {
  gtsam::Point3 gravity(gravity_vec.x, gravity_vec.y, gravity_vec.z);
  gtsam::Unit3 gravity_dir(gravity.normalize());
  gtsam::SharedNoiseModel model =
      gtsam::noiseModel::Isotropic::Sigma(2, noise_sigma_);
  gtsam::Pose3AttitudeFactor factor(query_key_, gravity_dir, model);
  return factor;
}

bool StationaryHandler::SetKeyForImuAttitude(const gtsam::Symbol& key) {
  query_key_ = key;
  if (query_key_ == key) {
    return true;
  } else {
    ROS_WARN("Could not store received symbol into protected class member");
    return false;
  }
}
