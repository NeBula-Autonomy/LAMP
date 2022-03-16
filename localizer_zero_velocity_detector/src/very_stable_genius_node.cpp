#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <very_stable_genius/very_stable_genius.hpp>
#include <localizer_zero_velocity_detector/Stationary.h>

using namespace very_stable_genius;
using namespace localizer_zero_velocity_detector;

class VeryStableGeniusNode {
public:
  VeryStableGeniusNode () {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Obtain YAML config filename
    std::string cfg_filename;
    if (!pnh.getParam("config", cfg_filename)) {
      throw std::runtime_error("Yaml config filename not provided");
    }

    // Set up circular buffer
    vsg_ = VeryStableGenius(cfg_filename);

    // Set up ROS publishers and subscribers
    imu_sub_ = nh.subscribe("imu", 200, &VeryStableGeniusNode::imuCallback, this);
    bool_status_pub_ = nh.advertise<std_msgs::Bool>("stationary", 10);
    full_status_pub_ = nh.advertise<Stationary>("stationary_accel", 10);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    vsg_.addImuMeasurement(msg);

    Vec3 accel_avg;
    int status = vsg_.getStatus(&accel_avg);

    std_msgs::Bool bool_msg;
    
    Stationary full_msg;
    full_msg.header.stamp = msg->header.stamp;
    full_msg.header.frame_id = msg->header.frame_id;
    
    if (INITIALIZING == status) {
      ROS_INFO_STREAM_ONCE("Initializing: requires a few seconds of data");
      full_msg.status = Stationary::INITIALIZING;
      full_msg.average_acceleration.x = std::numeric_limits<double>::quiet_NaN();
      full_msg.average_acceleration.y = std::numeric_limits<double>::quiet_NaN();
      full_msg.average_acceleration.z = std::numeric_limits<double>::quiet_NaN();
      full_status_pub_.publish(full_msg);      
    }
    else if (STATIONARY == status) {
      full_msg.status = Stationary::STATIONARY;
      full_msg.average_acceleration.x = accel_avg.x;
      full_msg.average_acceleration.y = accel_avg.y;
      full_msg.average_acceleration.z = accel_avg.z;
      full_status_pub_.publish(full_msg);
      
      bool_msg.data = true;      
      bool_status_pub_.publish(bool_msg);
    }
    else if (NONSTATIONARY == status) {
      full_msg.status = Stationary::NONSTATIONARY;
      full_msg.average_acceleration.x = std::numeric_limits<double>::quiet_NaN();
      full_msg.average_acceleration.y = std::numeric_limits<double>::quiet_NaN();
      full_msg.average_acceleration.z = std::numeric_limits<double>::quiet_NaN();
      full_status_pub_.publish(full_msg);
      
      bool_msg.data = false;
      bool_status_pub_.publish(bool_msg); 
    }
    else {
      ROS_ERROR_STREAM("Unrecognized status: " << status);
    }
  }
  
private:
  VeryStableGenius vsg_;
  ros::Subscriber imu_sub_;
  ros::Publisher bool_status_pub_;
  ros::Publisher full_status_pub_;  
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "very_stable_genius_node");
  ros::NodeHandle nh;
  VeryStableGeniusNode vsg_node;
  ros::spin();
  return 0;
}
