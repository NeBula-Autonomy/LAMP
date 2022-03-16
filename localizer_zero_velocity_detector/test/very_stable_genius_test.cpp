#include <ros/ros.h>
#include <rosbag/bag.h>
#include <very_stable_genius/very_stable_genius.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace very_stable_genius;

int main() {
  // open benchmark data
  
  // open rosbag
  rosbag::Bag bag;
  bag.open("benchmark.bag", rosbag::bagmode::Read);

  VeryStableGenius vsg;
  
  std::vector<std::string> topics;
  topics.push_back(std::string("/husky2/vn100/imu"));
  /*
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  foreach(rosbag::MessageInstance const m, view) {
    sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
    if (msg == NULL) { continue; }
    vsg.addImuMeasurement(msg);
    vsg.getStatus();
  }
  */
  bag.close();


}


