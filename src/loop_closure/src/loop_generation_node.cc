/*
 * Copyright Notes
 *
 * Authors: Benjamin Morrell    (benjamin.morrell@jpl.nasa.gov)
 */

#include <loop_closure/LaserLoopClosure.h>
#include <loop_closure/ProximityLoopGeneration.h>
#include <ros/ros.h>

namespace lc = lamp_loop_closure;

int main(int argc, char** argv) {
  ros::init(argc, argv, "loop_generation");
  ros::NodeHandle n("~");

  lc::ProximityLoopGeneration loop_gen;
  if (!loop_gen.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize Loop Candidate Generation module. ",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
