/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <ros/ros.h>
#include <lamp/LampBaseStation.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp");
  ros::NodeHandle n("~");

  LampBaseStation lbs;
  if (!lbs.Initialize(n, false /* online processing */)) {
    ROS_ERROR("%s: Failed to initialize LAMP Base Station.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
