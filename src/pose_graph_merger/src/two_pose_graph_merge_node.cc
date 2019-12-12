/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <pose_graph_merger/TwoPoseGraphMerge.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "two_pg_merge");
  ros::NodeHandle n("~");

  TwoPoseGraphMerge tpgm;
  if (!tpgm.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize TwoPoseGraphMerge.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
