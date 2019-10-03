/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <ros/ros.h>
#include <lamp/LampRobot.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp");
  ros::NodeHandle n("~");

  LampRobot lr;
  if (!lr.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize LAMP.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
