/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <loop_closure/LaserLoopClosure.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp");
  ros::NodeHandle n("~");

  LaserLoopClosure llc(n);
  if (!llc.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize LAMP.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
