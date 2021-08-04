/*
 * Copyright Notes
 * Authors: Yun Chang          (yunchang@mit.edu)
 */

#ifndef STATIONARY_HANDLER_H
#define STATIONARY_HANDLER_H

#include <factor_handlers/LampDataHandlerBase.h>
#include <geometry_msgs/Vector3.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <localizer_zero_velocity_detector/Stationary.h>

typedef localizer_zero_velocity_detector::Stationary StationaryMessage;
typedef std::pair<ros::Time, geometry_msgs::Vector3> StationaryData;

class StationaryHandler : public LampDataHandlerBase {
  friend class StationaryHandlerTest;

 public:
  StationaryHandler();
  ~StationaryHandler();

  bool Initialize(const ros::NodeHandle& n);

  // LAMP Interface
  std::shared_ptr<FactorData> GetData() override;

  bool SetKeyForImuAttitude(const gtsam::Symbol& key);
  bool CheckKeyRecency(const gtsam::Symbol& key);
  bool has_data_;

 protected:
  std::string name_;

  ros::Subscriber stationary_sub_;
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  void StationaryCallback(const StationaryMessage::ConstPtr& msg);

  // Quaternions
  bool GetGravityAtTime(const ros::Time& stamp,
                        geometry_msgs::Vector3& gravity) const;

  // Factors
  gtsam::Pose3AttitudeFactor CreateAttitudeFactor(
      const geometry_msgs::Vector3& gravity_vec) const;
  void ResetFactorData();

  gtsam::Symbol query_key_;
  double noise_sigma_;

  bool currently_stationary_;
  StationaryData last_detection_;
  int key_step_threshold_;
};

#endif