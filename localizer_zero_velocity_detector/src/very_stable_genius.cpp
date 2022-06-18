#include <iostream>
#include <boost/circular_buffer.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/Imu.h>
#include <very_stable_genius/vec3.hpp>
#include <very_stable_genius/very_stable_genius.hpp>

namespace very_stable_genius {
  
  VeryStableGenius::VeryStableGenius(const std::string &yaml_cfg_filename) {
    // Read parameters from yaml file
    parseConfig(yaml_cfg_filename);
    imu_circular_buffer_.set_capacity(imu_rate_hz_ * observation_period_s_);
  }

  VeryStableGenius::VeryStableGenius() {
    // Default parameters if a yaml file is not provided
    imu_rate_hz_ = 50.0; 
    observation_period_s_ = 3.0; 
    imu_max_rate_x_ = 0.025;
    imu_max_rate_y_ = 0.025; 
    imu_max_rate_z_ = 0.025;
    imu_max_accel_x_ = 0.75;
    imu_max_accel_y_ = 0.5;
    imu_max_accel_z_ = 0.5;
    imu_circular_buffer_.set_capacity(imu_rate_hz_ * observation_period_s_);
  }
  
  bool VeryStableGenius::parseConfig(const std::string &filename) {
    YAML::Node config = YAML::LoadFile(filename);
    // Check version
     int version = config["version"].as<int>();
    if (1 == version) {
      imu_max_rate_x_ = config["imu_max_rate_x"].as<double>();
      imu_max_rate_y_ = config["imu_max_rate_y"].as<double>();
      imu_max_rate_z_ = config["imu_max_rate_z"].as<double>();
      imu_max_accel_x_ = config["imu_max_accel_x"].as<double>();
      imu_max_accel_y_ = config["imu_max_accel_y"].as<double>();
      imu_max_accel_z_ = config["imu_max_accel_z"].as<double>();
      imu_rate_hz_ = config["imu_rate_hz"].as<double>();    
      observation_period_s_ = config["observation_period_s"].as<double>();
    } else {
      throw std::runtime_error("Unrecognized yaml version number in " +
                               filename + ": " + std::to_string(version));
    }
   return true;
  }

  void VeryStableGenius::addImuMeasurement(const ImuMeasurement &measurement) {
    imu_circular_buffer_.push_back(measurement);
  }
  
  void VeryStableGenius::addImuMeasurement(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_circular_buffer_.push_back(ImuMeasurement(msg->header.stamp.toSec(),
                                                  Vec3(msg->linear_acceleration.x,
                                                       msg->linear_acceleration.y,
                                                       msg->linear_acceleration.z),
                                                  Vec3(msg->angular_velocity.x,
                                                       msg->angular_velocity.y,
                                                       msg->angular_velocity.z)));
  }
  
  int VeryStableGenius::getStatus() {
    return getStatus(NULL);
  }

  int VeryStableGenius::getStatus(Vec3 *accel_avg_in) {
    if (!imu_circular_buffer_.full()) {
      // We haven't received observation_period_s worth of measurements, so return initializing state
      // ROS_INFO_STREAM("Buffer has " << imu_circular_buffer_.size() << " elements out of " << imu_circular_buffer_.capacity());
      return INITIALIZING;
    } else {
      // Take average of all measurements in the circular buffer
      // Determine maximum difference from average. If any measurement exceeds our difference threshold, we consider the robot to be moving (nonstationary)
      Vec3 accel_avg;
      Vec3 accel_max = imu_circular_buffer_[0].accel;
      Vec3 accel_min = imu_circular_buffer_[0].accel;
      Vec3 accel_max_diff;
      Vec3 gyro_avg;
      Vec3 gyro_max = imu_circular_buffer_[0].gyro;
      Vec3 gyro_min = imu_circular_buffer_[0].gyro;
      Vec3 gyro_max_diff;
      
      for (auto &measurement : imu_circular_buffer_) {
        accel_avg += measurement.accel;
        gyro_avg += measurement.gyro;
        accel_max = Vec3::max(accel_max, measurement.accel);
        accel_min = Vec3::min(accel_min, measurement.accel);
        gyro_max = Vec3::max(gyro_max, measurement.gyro);
        gyro_min = Vec3::min(gyro_min, measurement.gyro);
      }
      accel_avg = accel_avg / static_cast<double>(imu_circular_buffer_.size());
      gyro_avg = gyro_avg / static_cast<double>(imu_circular_buffer_.size());

      accel_max_diff = Vec3::max(Vec3::abs(accel_max - accel_avg),
                                 Vec3::abs(accel_avg - accel_min));
      gyro_max_diff = Vec3::max(Vec3::abs(gyro_max - gyro_avg),
                                Vec3::abs(gyro_avg - gyro_min));

      if (NULL != accel_avg_in) {
        *accel_avg_in = accel_avg;
      }
      
      // If any differences exceed our threshold, we consider it nonstationary
      if ((accel_max_diff.x > imu_max_accel_x_) ||
          (accel_max_diff.y > imu_max_accel_y_) ||
          (accel_max_diff.z > imu_max_accel_z_) ||
          (gyro_max_diff.x >  imu_max_rate_x_) ||
          (gyro_max_diff.y >  imu_max_rate_y_) ||
          (gyro_max_diff.z >  imu_max_rate_z_)) {
        return NONSTATIONARY;
      } else {
        return STATIONARY;        
      }
    }
  }

}
