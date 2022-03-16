/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <lamp/LampBaseStation.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp");
  ros::NodeHandle n("~");

  LampBaseStation lbs;
  if (!lbs.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize LAMP Base Station.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
