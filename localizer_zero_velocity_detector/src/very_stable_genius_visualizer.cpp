#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <very_stable_genius/very_stable_genius.hpp>

#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

/// This node was mainly written for visualization and as an example of how to use the library. Users will probably want to interface with the library, not this node. 

using namespace very_stable_genius;

visualization_msgs::Marker makeArrow(const std::string &marker_namespace, const Vec3 &gravity, std_msgs::ColorRGBA color) { 
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.ns = marker_namespace;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.x = 0.1;
  marker.scale.y = 0.2;
  marker.scale.z = 0;
  marker.points.clear();

  geometry_msgs::Point origin, gravity_direction;
  origin.x = 0;
  origin.y = 0;
  origin.z = 0;
  gravity_direction.x = -gravity.x;
  gravity_direction.y = -gravity.y;
  gravity_direction.z = -gravity.z;
  
  marker.points.push_back(origin);
  marker.points.push_back(gravity_direction);
  marker.color = color;
  
  return marker;
}

class VeryStableGeniusNode {
public:
  VeryStableGeniusNode () : status_(-1) {
    // Obtain YAML config filename
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string cfg_filename;
    if (!pnh.getParam("config", cfg_filename)) {
      throw std::runtime_error("Yaml config filename not provided");
    }

    // Set up circular buffer
    vsg_ = VeryStableGenius(cfg_filename);

    // Set up ROS publishers and subscribers
    imu_sub_ = nh.subscribe("imu", 200, &VeryStableGeniusNode::imuCallback, this);
    status_pub_ = nh.advertise<std_msgs::Float32>("stationary", 100); // Debug topic

    gravity_pub_ = nh.advertise<visualization_msgs::Marker>("gravity", 100); // Debug topic
    average_gravity_pub_ = nh.advertise<visualization_msgs::Marker>("average_gravity", 100); // Debug topic
    
    sample_time_ = ros::Time::now();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    vsg_.addImuMeasurement(msg);

    Vec3 accel_avg;
    int status = vsg_.getStatus(&accel_avg);
    
    double diff = ros::Time::now().toSec() - sample_time_.toSec();
    std_msgs::Float32 status_msg;
    
    { // Publish raw accelerometer arrow visualization
      Vec3 gravity(msg->linear_acceleration.x,
                   msg->linear_acceleration.y,
                   msg->linear_acceleration.z);      
      std_msgs::ColorRGBA color;
      color.a = 0.5;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      visualization_msgs::Marker gravity_arrow = makeArrow("gravity", gravity, color);
      gravity_pub_.publish(gravity_arrow);
    }
    
    if (INITIALIZING == status) {
      ROS_WARN_STREAM("INITIALIZING");
      status_msg.data = 0;
      status_pub_.publish(status_msg);
    }
    else if (STATIONARY == status) {      
      ROS_WARN_STREAM("STATIONARY");
      status_msg.data = 5; // Arbitrary, chosen for visualization
      status_pub_.publish(status_msg);
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      ROS_INFO_STREAM("Average gravity: " << accel_avg.x << " " <<
                      accel_avg.y << " " <<
                      accel_avg.z);
      visualization_msgs::Marker average_gravity_arrow = makeArrow("average_gravity", accel_avg, color);
      average_gravity_pub_.publish(average_gravity_arrow);
    }
    else if (NONSTATIONARY == status) {
      ROS_WARN_STREAM("NONSTATIONARY");
      status_msg.data = 7; // Arbitrary, chosen for visualization
      status_pub_.publish(status_msg);
    }
    else {
      ROS_ERROR_STREAM("Unrecognized status: " << status);
    }
  }
  
private:
  VeryStableGenius vsg_;
  ros::Subscriber imu_sub_;
  ros::Publisher status_pub_;
  int status_;
  ros::Time sample_time_;
  ros::Publisher gravity_pub_;
  ros::Publisher average_gravity_pub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "very_stable_genius_node");
  ros::NodeHandle nh;
  VeryStableGeniusNode vsg_node;
  ros::spin();
  return 0;
}
