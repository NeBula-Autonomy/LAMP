#ifndef __VERY_STABLE_GENIUS_H__
#define __VERY_STABLE_GENIUS_H__

#include <iostream>
#include <boost/circular_buffer.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/Imu.h>
#include <very_stable_genius/vec3.hpp>

namespace very_stable_genius {
  
  enum Status {
    INITIALIZING, /// Circular buffer hasn't reached capacity yet. Expect this status code for the first N seconds.
    STATIONARY,   /// Robot is stationary (not moving). If reported, very likely that the robot is stationary.
    NONSTATIONARY /// Robot is nonstationary (moving). If reported, it's possible that the robot is either stationary or nonstationary due to lag from the circular buffer. Do not use this status to determine if the robot is moving. 
  };

  enum ErrorCode {
    ERROR = -1,
    SUCCESS = 0
  };

  struct ImuMeasurement {
    ImuMeasurement(double timestamp, Vec3 accel, Vec3 gyro)
      : timestamp(timestamp), accel(accel), gyro(gyro) {}
    
    double timestamp;
    Vec3 accel;
    Vec3 gyro;
  };

  class VeryStableGenius {
  public:
    VeryStableGenius(const std::string &yaml_cfg_filename); /// Reads parameters from a yaml config file
    VeryStableGenius(); /// Uses default parameters if a yaml file is not provided. Generally only useful for testing.
    bool parseConfig(const std::string &filename); /// Parses configuration parameters from a yaml file
    void addImuMeasurement(const ImuMeasurement &measurement); /// Add an IMU measurement to the circular buffer
    void addImuMeasurement(const sensor_msgs::Imu::ConstPtr &msg); /// Add an IMU measurement to the circular buffer
    int getStatus(); /// Compute and return a Status code
    int getStatus(Vec3 *accel_avg_in); /// Compute and return a Status code and an averaged accelerometer reading

  private:
    double imu_rate_hz_;          /// IMU message publishing rate, in Hz. Currently 50Hz on Husky2
    double observation_period_s_; /// How long does the robot need to be stationary to be declared stationary?
    double imu_max_rate_x_;       /// Maximum allowed difference between measurement and average until considered moving
    double imu_max_rate_y_;       /// Maximum allowed difference between measurement and average until considered moving
    double imu_max_rate_z_;       /// Maximum allowed difference between measurement and average until considered moving
    double imu_max_accel_x_;      /// Maximum allowed difference between measurement and average until considered moving
    double imu_max_accel_y_;      /// Maximum allowed difference between measurement and average until considered moving
    double imu_max_accel_z_;      /// Maximum allowed difference between measurement and average until considered moving
    boost::circular_buffer<ImuMeasurement> imu_circular_buffer_;  /// Circular buffer where IMU messages are stored
  };

}

#endif
